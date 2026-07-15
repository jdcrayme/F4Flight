// f4flight - digi/ground/ground_ops.cpp
//
// Ground operations implementation: takeoff, landing, taxi.
//
// Simplified from FreeFalcon's landme.cpp (4,778 LOC). This implementation
// handles the core takeoff and landing sequences with ATC coordination via
// the MessageBus.

#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/atc/taxi_graph.h"  // Round-2 fix: RunTaxi uses TaxiGraph
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/digi/steering.h"  // for headingError

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// Constants
static constexpr double kTaxiSpeedKts = 15.0;       // taxi speed
static constexpr double kRotationSpeedFraction = 0.9; // V_R = 0.9 * stallSpeed
static constexpr double kClimboutAltFt = 500.0;      // gear up altitude
static constexpr double kFlareAltFt = 100.0;          // flare start altitude AGL (was 50 — too low for high descent rates)
static constexpr double kApproachGlideslope = 3.0;    // degrees
static constexpr double kApproachSpeedFraction = 1.3; // V_approach = 1.3 * stallSpeed
static constexpr double kRolloutBrakeSpeed = 80.0;    // kts — start braking below this

void ProcessATCMessages(DigiState& digi, Mailbox& mailbox) {
    while (auto msg = mailbox.pop()) {
        switch (msg->type) {
            case MessageType::ATCClearedTakeoff:
                digi.ag.groundOps.hasTakeoffClearance = true;
                digi.ag.groundOps.assignedRunway = atc::runwayFromMessage(*msg);
                digi.ag.groundOps.phase = GroundOpsPhase::LiningUp;
                break;

            case MessageType::ATCClearedLanding:
                digi.ag.groundOps.hasLandingClearance = true;
                digi.ag.groundOps.assignedRunway = atc::runwayFromMessage(*msg);
                break;

            case MessageType::ATCHoldShort:
                digi.ag.groundOps.hasTakeoffClearance = false;
                digi.ag.groundOps.phase = GroundOpsPhase::HoldingShort;
                break;

            case MessageType::ATCGoAround:
                digi.ag.groundOps.hasLandingClearance = false;
                digi.ag.groundOps.phase = GroundOpsPhase::Approach;
                break;

            case MessageType::ATCTaxiInstruction: {
                double tx, ty;
                atc::taxiPointFromMessage(*msg, tx, ty);
                digi.ag.groundOps.runwayThresholdX = tx;
                digi.ag.groundOps.runwayThresholdY = ty;
                digi.ag.groundOps.phase = GroundOpsPhase::TaxiToRunway;
                break;
            }

            default:
                break;
        }
    }
}

void RunTaxi(DigiState& digi, const AircraftState& as,
             FcsState& fcsState, double dt) {
    auto& go = digi.ag.groundOps;

    // Round-2 structural fix (Rec 4 / Bug K): RunTaxi was previously dead
    // code — it always steered toward (runwayThresholdX, runwayThresholdY)
    // regardless of any taxi graph. Now if a TaxiGraph is set, we follow
    // the BFS path node-by-node. If no graph is set, fall back to the old
    // direct-to-threshold behavior (so existing hosts that don't supply a
    // graph keep working).

    double targetX = go.runwayThresholdX;
    double targetY = go.runwayThresholdY;

    if (go.taxiGraph != nullptr && go.targetTaxiNode >= 0) {
        // Re-compute path if target changed or we have no path yet.
        if (go.taxiPath.empty() ||
            (go.taxiPathIdx == 0 && go.currentTaxiNode != go.targetTaxiNode &&
             (!go.taxiPath.empty() && go.taxiPath.back() != go.targetTaxiNode))) {
            // BFS from currentTaxiNode to targetTaxiNode.
            // (Casts int → TaxiNodeId; both are int.)
            go.taxiPath = go.taxiGraph->findPath(go.currentTaxiNode,
                                                  go.targetTaxiNode);
            go.taxiPathIdx = (go.taxiPath.size() > 1) ? 1 : 0;
        }

        // If we have a path, steer toward the next node in it.
        if (!go.taxiPath.empty() && go.taxiPathIdx < go.taxiPath.size()) {
            const auto& node = go.taxiGraph->node(go.taxiPath[go.taxiPathIdx]);
            targetX = node.position.x;
            targetY = node.position.y;

            // Check if we've reached this node; if so, advance the path.
            const double dxN = targetX - as.kin.x;
            const double dyN = targetY - as.kin.y;
            const double distN = std::sqrt(dxN * dxN + dyN * dyN);
            if (distN < 50.0 && go.taxiPathIdx + 1 < go.taxiPath.size()) {
                ++go.taxiPathIdx;
                go.currentTaxiNode = go.taxiPath[go.taxiPathIdx];
            }
        }
    }

    // Simple taxi: steer toward target point at taxi speed
    const double dx = targetX - as.kin.x;
    const double dy = targetY - as.kin.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 50.0) {
        // Reached target — stop
        digi.commands.throttle = 0.0;
        digi.commands.pStick = 0.0;
        digi.commands.yPedal = 0.0;
        digi.commands.wheelBrakes = true;  // hold position with brakes
        return;
    }
    digi.commands.wheelBrakes = false;

    // Steer toward target. Use body yaw (psi) for the heading error —
    // sigma (velocity heading) is unreliable at low speed. The EOM ground
    // clamp uses yPedal for nose-wheel steering, so we command the pedals
    // (not the roll stick) to steer on the ground.
    //
    // BUG FIX (paired with EOM fix): previously used rStick for ground
    // steering, which only worked because the EOM had a bug (it used
    // rStick instead of yPedal for NWS). With the EOM fixed, the digi must
    // use yPedal for ground steering.
    const double desHeading = std::atan2(dy, dx);
    const double headingErr = headingError(desHeading, as.kin.psi);

    // Direct yPedal command: full deflection at 30° error.
    // The EOM nose-wheel steering will turn the aircraft.
    //
    // SIGN NOTE: headingErr > 0 means we need to turn LEFT (increase psi in
    // NED CCW frame). Left turn requires LEFT pedal = negative yPedal
    // (per PilotInput convention: -1 = full left, +1 = full right).
    // The EOM uses `psi -= ypedal * rate`, so ypedal < 0 → psi increases →
    // left turn. Hence the negation: `yPedal = -headingErr * scale`.
    digi.commands.yPedal = std::max(-1.0, std::min(1.0, -headingErr * RTD / 30.0));
    digi.commands.rStick = 0.0;  // no aileron on the ground

    // Throttle for taxi speed — simple proportional controller.
    // MachHold's sqrt mapping and integral windup are ill-suited for
    // the very low taxi speeds. Just command a small throttle proportional
    // to the speed error, and cut to zero when at taxi speed.
    const double speedErr = kTaxiSpeedKts - as.vcas;
    if (speedErr > 5.0) {
        digi.commands.throttle = std::min(0.3, speedErr * 0.01);
    } else if (speedErr < -5.0) {
        digi.commands.throttle = 0.0;
        digi.commands.wheelBrakes = true;  // brake if too fast
    } else {
        digi.commands.throttle = std::max(0.0, speedErr * 0.02);
    }

    // Wings level, hold altitude (on ground)
    fcsState.maxRoll = 0.0;
    digi.commands.pStick = 0.0;
}

void RunTakeoff(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ) {
    (void)dt;  // takeoff uses discrete phase transitions, not dt
    auto& go = digi.ag.groundOps;

    switch (go.phase) {
        case GroundOpsPhase::Idle:
            // Start takeoff sequence
            go.phase = GroundOpsPhase::TakeoffRoll;
            go.takeoffRollStart = simTime;
            go.gearRetracted = false;
            break;

        case GroundOpsPhase::Parking:
        case GroundOpsPhase::RequestTaxi:
        case GroundOpsPhase::TaxiToRunway:
            // Taxi to the runway threshold using RunTaxi.
            // RunTaxi steers toward (runwayThresholdX, runwayThresholdY)
            // at taxi speed. When it arrives (dist < 50 ft), it stops and
            // sets wheel brakes. The brain then transitions to HoldingShort
            // (or directly to LiningUp if clearance is already granted).
            RunTaxi(digi, as, fcsState, dt);
            // Check if we've arrived at the threshold
            {
                const double dxT = go.runwayThresholdX - as.kin.x;
                const double dyT = go.runwayThresholdY - as.kin.y;
                const double distT = std::sqrt(dxT * dxT + dyT * dyT);
                if (distT < 50.0) {
                    if (go.hasTakeoffClearance) {
                        go.phase = GroundOpsPhase::LiningUp;
                    } else {
                        go.phase = GroundOpsPhase::HoldingShort;
                    }
                }
            }
            break;

        case GroundOpsPhase::HoldingShort:
            // Hold position and wait for takeoff clearance
            digi.commands.throttle = 0.0;
            digi.commands.pStick = 0.0;
            digi.commands.yPedal = 0.0;
            digi.commands.rStick = 0.0;
            digi.commands.wheelBrakes = true;
            // Auto-grant clearance after 2 seconds (simplified — real ATC
            // would send an ATCClearedTakeoff message via the MessageBus)
            if (simTime > 2.0) {
                go.hasTakeoffClearance = true;
                go.phase = GroundOpsPhase::LiningUp;
            }
            break;

        case GroundOpsPhase::LiningUp:
            // Taxi onto runway centerline and line up with runway heading
            {
                const double dxL = go.runwayThresholdX - as.kin.x;
                const double dyL = go.runwayThresholdY - as.kin.y;
                const double distL = std::sqrt(dxL * dxL + dyL * dyL);
                if (distL < 10.0) {
                    // On the threshold — start takeoff roll
                    go.phase = GroundOpsPhase::TakeoffRoll;
                    go.takeoffRollStart = simTime;
                    go.gearRetracted = false;
                } else {
                    // Taxi to the exact threshold point
                    RunTaxi(digi, as, fcsState, dt);
                }
            }
            break;

        case GroundOpsPhase::TakeoffRoll: {
            // Full throttle (1.5 = AB if available, 1.0 = MIL otherwise)
            digi.commands.throttle = 1.5;

            // Keep straight on runway heading. Below 30 kts the velocity
            // heading (sigma) is unreliable (EOM singularity at 0 speed),
            // so don't steer. Above 30 kts, use body yaw (psi) instead of
            // sigma — psi is the actual aircraft heading and is stable at
            // all speeds, while sigma (velocity-vector heading) is noisy
            // at low speed and drifts with minor lateral velocity.
            //
            // BUG FIX (paired with EOM fix): use yPedal (rudder pedal) for
            // NWS, not rStick (roll stick). The EOM was previously buggy
            // and used rStick for NWS; the digi was compensating. Now both
            // are fixed: EOM uses yPedal, digi commands yPedal.
            if (as.vcas > 30.0) {
                const double headingErr = headingError(go.runwayHeading, as.kin.psi);
                // Sign: headingErr > 0 = need left turn = left pedal (yPedal < 0).
                // See RunTaxi for the full sign-convention note.
                digi.commands.yPedal = std::max(-1.0, std::min(1.0, -headingErr * RTD / 20.0));
            } else {
                digi.commands.yPedal = 0.0;
            }
            digi.commands.rStick = 0.0;  // no aileron during takeoff roll

            // Hold attitude neutral on the ground.
            digi.commands.pStick = 0.0;
            fcsState.maxRoll = 0.0;

            // Check for rotation speed
            // V_R = 1.2 × stallSpeed (real-world rotation speed is typically
            // 1.2-1.3 × V_s). The previous 1.1× was too low for light
            // fighters (F-15C, F-5, MiG-21) — they would rotate and lift off
            // at marginal speed, then settle back onto the runway because
            // they couldn't sustain the climb at low airspeed.
            const double rotationSpeed = 1.2 * (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);
            if (as.vcas >= rotationSpeed) {
                go.phase = GroundOpsPhase::Rotation;
            }
            break;
        }

        case GroundOpsPhase::Rotation: {
            // Pull up — rotate to climb attitude.
            //
            // The previous code used SetPstick(3.0, maxGs, GCommand) which
            // passes through the sqrt G-command mapping: stickCmd = sqrt((3-1)/(9-1))
            // = 0.5. At high speed the low-speed authority factor saturates at
            // 1.0, giving pstick = 0.5 — too weak to rotate the nosewheel off
            // the ground. The aircraft sat on the runway accelerating to 600+
            // kts without lifting off.
            //
            // Fix: set pStick directly to 0.8 (strong but not max deflection).
            // This produces enough elevator authority to rotate at V_R.
            digi.commands.throttle = 1.5;
            digi.commands.pStick = 0.8;
            fcsState.maxRoll = 0.0;

            // Check if airborne (altitude > groundZ + 10ft)
            const double rotAltAGL = -as.kin.z - groundZ;
            if (rotAltAGL > 10.0) {
                go.phase = GroundOpsPhase::AfterTakeoff;
                go.gearRetracted = false;
            }
            break;
        }

        case GroundOpsPhase::AfterTakeoff: {
            // Standard departure procedure:
            //   1. Gear up on positive rate of climb (VVI > 0)
            //   2. Hold wings level + runway heading until 50 ft AGL
            //      (departure end altitude for most procedures)
            //   3. Above 50 ft: transition to HeadingAndAltitudeHold for
            //      climbout to 1500 ft AGL
            //
            // The previous code called HeadingAndAltitudeHold immediately
            // after liftoff, which caused the aircraft to bank hard if there
            // was any heading error from the takeoff roll — the "immediately
            // banks away as soon as it's airborne" behavior.

            // Gear up on positive rate of climb
            const double altAGL_takeoff = -as.kin.z - groundZ;
            if (!go.gearRetracted && as.kin.zdot < -5.0) {
                // zdot < 0 = climbing (NED z-down). Require > 5 ft/s = 300 fpm.
                go.gearRetracted = true;
            }

            digi.commands.throttle = 1.5;  // full throttle for climb

            if (altAGL_takeoff < 50.0) {
                // Below 50 ft AGL: hold wings level + runway heading.
                // Direct commands (no FCS gain dependency) for stability.
                fcsState.maxRoll = 0.0;
                digi.commands.rStick = std::max(-0.3, std::min(0.3,
                    -as.kin.phi * RTD * 2.0 * DTR));
                // Pitch: for the first few seconds after liftoff, command a
                // strong pitch-up (pStick=0.6) to establish a climb attitude.
                // This is especially important for low-T/W aircraft (J-7,
                // MiG-21, Q-5) that can't sustain a climb at marginal speed
                // — they need 10-12° pitch ASAP or they settle back.
                // Once theta reaches ~8°, switch to a pitch-hold controller
                // to maintain the attitude.
                if (as.kin.theta < 8.0 * DTR) {
                    // Build pitch attitude aggressively
                    digi.commands.pStick = 0.6;
                } else {
                    // Hold ~10° pitch attitude
                    const double targetPitch = 10.0 * DTR;
                    double pitchErr = targetPitch - as.kin.theta;
                    digi.commands.pStick = std::max(-0.3, std::min(0.8, pitchErr * 3.0));
                }
            } else {
                // Above 50 ft: transition to heading + altitude hold
                const double targetAlt = groundZ + 1500.0;
                ManeuverPrimitives::HeadingAndAltitudeHold(
                    go.runwayHeading, targetAlt, digi, as,
                    FlightControlSystem{}, fcsState, digi.config.maxGs);
            }
            break;
        }

        default:
            break;
    }
}

void RunLanding(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ) {
    (void)simTime;
    auto& go = digi.ag.groundOps;

    const double altAGL = -as.kin.z - groundZ;

    switch (go.phase) {
        case GroundOpsPhase::Idle:
            go.phase = GroundOpsPhase::Approach;
            go.approachStartAlt = altAGL;
            go.gearDeployed = false;
            break;

        case GroundOpsPhase::Approach: {
            // Deploy gear below 500 ft
            if (!go.gearDeployed && altAGL < 500.0) {
                go.gearDeployed = true;
            }

            // Fly toward runway threshold, descending on 3° glideslope.
            //
            // Use a direct descent-rate controller instead of TrackPointLanding.
            // The TrackPointLanding primitive's elevation-angle tracker has
            // too little gain at long range, causing the aircraft to oscillate
            // above and below the glideslope. Instead, we:
            //   1. Command a -3° flight path angle (GammaHold)
            //   2. Steer toward the runway threshold (heading)
            //   3. Hold approach speed (throttle + speed brakes)
            // This produces a stable, smooth 3° descent.
            const double dx = go.runwayThresholdX - as.kin.x;
            const double dy = go.runwayThresholdY - as.kin.y;
            const double distToThreshold = std::sqrt(dx * dx + dy * dy);

            // Approach speed
            const double approachSpeed = kApproachSpeedFraction *
                (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);

            // Steer toward the runway threshold (heading)
            const double desHeading = std::atan2(dy, dx);
            const double headingErr = headingError(desHeading, as.kin.sigma);
            // Use proportional roll command (same as HeadingAndAltitudeHold)
            digi.commands.rStick = std::max(-1.0, std::min(1.0, headingErr * RTD * 2.0 * DTR));

            // Command -3° flight path angle (descending glideslope)
            // GammaHold commands a target gamma; -3° = descending
            ManeuverPrimitives::GammaHold(-kApproachGlideslope, digi, as, digi.config.maxGs);

            // Throttle: hold approach speed
            const double eProp = approachSpeed - as.vcas;
            if (eProp >= 150.0) {
                digi.commands.throttle = 1.5;  // burner
            } else if (eProp < -20.0) {
                digi.commands.throttle = 0.0;  // idle
            } else {
                digi.commands.throttle = std::max(0.0, std::min(0.5, eProp * 0.01));
            }

            // Extend speed brakes if significantly above approach speed
            if (as.vcas > approachSpeed + 30.0) {
                digi.commands.speedBrakeCmd = 1.0;  // full extend
            } else if (as.vcas > approachSpeed + 10.0) {
                digi.commands.speedBrakeCmd = 0.5;  // half extend
            } else {
                digi.commands.speedBrakeCmd = -1.0;  // retract (clean for flare)
            }

            // Check for flare altitude
            if (altAGL < kFlareAltFt) {
                go.phase = GroundOpsPhase::Flare;
                go.flareStartAlt = altAGL;
            }
            break;
        }

        case GroundOpsPhase::Flare: {
            // Flare: gradually arrest the descent rate for a gentle touchdown.
            //
            // The flare uses a target descent rate that decreases with altitude:
            //   At 100 ft: target = 15 ft/s (normal approach descent)
            //   At 50 ft:  target = 10 ft/s
            //   At 20 ft:  target = 5 ft/s
            //   At 5 ft:   target = 2 ft/s (gentle touchdown)
            // This prevents the aircraft from leveling off too high (which
            // keeps heavy aircraft airborne) and prevents slamming into the
            // ground. The pitch attitude naturally rises as the descent
            // rate is arrested, giving a main-gear-first touchdown.
            digi.commands.throttle = 0.0;  // idle — no thrust during flare

            // Target descent rate decreases with altitude
            const double targetDescentRate = std::max(2.0,
                15.0 * std::max(0.0, altAGL) / kFlareAltFt);
            // NED: zdot > 0 = descending. Error = actual - target.
            const double descentErr = as.kin.zdot - targetDescentRate;
            // Positive error = descending too fast → pitch up
            digi.commands.pStick = std::max(-0.3, std::min(0.8, descentErr * 0.03));

            // Wings level
            fcsState.maxRoll = 0.0;
            const double rollDeg = as.kin.phi * RTD;
            digi.commands.rStick = std::max(-1.0, std::min(1.0, -rollDeg * 2.0 * DTR));

            // Check for touchdown
            if (altAGL < 5.0) {
                go.phase = GroundOpsPhase::Touchdown;
                go.touchdownSpeed = as.vcas;
            }
            break;
        }

        case GroundOpsPhase::Touchdown: {
            // Main gear on ground — hold the flare attitude briefly so the
            // nose settles gently, then transition to rollout.
            // The previous code immediately set pStick=0, which dropped the
            // nose and caused nose-first touchdown.
            digi.commands.throttle = 0.0;

            // Hold 3° nose-up for a brief moment (nose settling)
            const double targetPitch = 3.0 * DTR;
            const double pitchErr = targetPitch - as.kin.theta;
            digi.commands.pStick = std::max(-0.2, std::min(0.5, pitchErr * 2.0));
            fcsState.maxRoll = 0.0;
            digi.commands.rStick = 0.0;
            digi.commands.yPedal = 0.0;

            // Transition to rollout after 1 second (nose has settled)
            go.touchdownTimer += dt;
            if (go.touchdownTimer > 1.0) {
                go.phase = GroundOpsPhase::Rollout;
            }
            break;
        }

        case GroundOpsPhase::Rollout: {
            // Decelerate — full brakes + speed brakes + idle.
            //
            // Hold a slight nose-up attitude (2°) during rollout to keep the
            // nose gear off the ground until the aircraft slows below 80 kts.
            // Real aircraft hold the nose up after touchdown — the nose gear
            // settles naturally as speed bleeds off and the elevator loses
            // authority. The previous code set pStick=0 immediately, which
            // let the ground clamp force theta to -2° (nose-first).
            digi.commands.throttle = 0.0;
            digi.commands.wheelBrakes = (as.vcas > 5.0);  // release brakes below 5 kts
            digi.commands.speedBrakeCmd = (as.vcas > 30.0) ? 1.0 : -1.0;  // full extend then retract

            // Keep straight on runway heading (NWS via yPedal)
            const double headingErr = headingError(go.runwayHeading, as.kin.psi);
            // Sign: headingErr > 0 = need left turn = left pedal (yPedal < 0).
            digi.commands.yPedal = std::max(-1.0, std::min(1.0, -headingErr * RTD / 20.0));
            digi.commands.rStick = 0.0;  // no aileron on rollout

            // Hold 2° nose-up while fast (nose gear off ground), relax to
            // level below 80 kts (nose settles naturally as elevator loses
            // authority).
            if (as.vcas > 80.0) {
                const double targetPitch = 2.0 * DTR;
                const double pitchErr = targetPitch - as.kin.theta;
                digi.commands.pStick = std::max(-0.2, std::min(0.5, pitchErr * 2.0));
            } else {
                // Below 80 kts — let the nose settle to level
                digi.commands.pStick = 0.0;
            }
            fcsState.maxRoll = 0.0;

            // Check if stopped or slow enough to vacate
            if (as.vcas < 10.0) {
                go.phase = GroundOpsPhase::VacatingRunway;
            }
            break;
        }

        case GroundOpsPhase::VacatingRunway: {
            // Taxi off runway — for simplicity, just hold position
            digi.commands.throttle = 0.0;
            digi.commands.pStick = 0.0;
            digi.commands.rStick = 0.0;
            digi.commands.yPedal = 0.0;
            // In a full impl, this would taxi to a runway exit node
            break;
        }

        default:
            break;
    }
}

} // namespace digi
} // namespace f4flight
