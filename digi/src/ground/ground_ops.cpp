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
#include "f4flight/digi/wingman/wingman_state.h"  // receiveOrders (Flight* msg dispatch)
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
static constexpr double kFlareAltFt = 150.0;          // flare start altitude AGL
static constexpr double kApproachGlideslope = 3.0;    // degrees
static constexpr double kApproachSpeedFraction = 1.3; // V_approach = 1.3 * stallSpeed
static constexpr double kRolloutBrakeSpeed = 80.0;    // kts — start braking below this

void ProcessATCMessages(DigiState& digi, Mailbox& mailbox) {
    // Process ALL incoming messages: ATC clearances, flight commands (lead →
    // wingman), and threat calls (between flight members). The function name
    // is historical — it originally only handled ATC, but the brain's mailbox
    // receives every message type and silently dropping non-ATC messages was
    // a real bug (Flight commands from the lead were lost, ThreatCall spikes
    // were never acted on). Now all message types are handled in one pass.
    while (auto msg = mailbox.pop()) {
        switch (msg->type) {
            // --- ATC messages (existing) ---
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

            // --- Flight commands (lead → wingman) ---
            // Delegate to receiveOrders(), which maps each command to
            // WingmanState field updates. receiveOrders returns false for
            // non-wingman messages, so we use the return value to decide
            // whether to fall through to the ThreatCall handler below.
            //
            // BUG FIX: previously, ProcessATCMessages had a `default: break;`
            // that silently dropped ALL Flight* and ThreatCall* messages.
            // Flight commands from the lead were lost, so wingmen never
            // received break/engage/rejoin/formation orders. Now all
            // FlightCmd* messages are routed to receiveOrders().
            case MessageType::FlightCmdEngage:
            case MessageType::FlightCmdEngageMyTarget:
            case MessageType::FlightCmdBreak:
            case MessageType::FlightCmdRejoin:
            case MessageType::FlightCmdWedge:
            case MessageType::FlightCmdTrail:
            case MessageType::FlightCmdSpread:
            case MessageType::FlightCmdEchelon:
            case MessageType::FlightCmdFingerFour:
            case MessageType::FlightCmdRTB:
            case MessageType::FlightCmdJettison:
            case MessageType::FlightCmdECMOn:
            case MessageType::FlightCmdECMOff:
            case MessageType::FlightCmdRadarOn:
            case MessageType::FlightCmdRadarOff:
            case MessageType::FlightCmdWeaponsHold:
            case MessageType::FlightCmdWeaponsFree:
            case MessageType::FlightCmdPromote:
            // Round-5 additions: tactical maneuver commands.
            case MessageType::FlightCmdClearSix:
            case MessageType::FlightCmdPosthole:
            case MessageType::FlightCmdChainsaw:
            case MessageType::FlightCmdSSOffset:
            case MessageType::FlightCmdFlex:
            case MessageType::FlightCmdPince:
            // Round-5 additions: formation spacing commands.
            case MessageType::FlightCmdKickout:
            case MessageType::FlightCmdCloseup:
            case MessageType::FlightCmdToggleSide:
            case MessageType::FlightCmdIncreaseRelAlt:
            case MessageType::FlightCmdDecreaseRelAlt:
                receiveOrders(digi.formation.wingman, *msg);
                break;

            // --- Threat calls (between flight members) ---
            // Store the bearing + type so the host/SensorFusion can resolve
            // it to an entity. The brain itself doesn't resolve threat calls
            // to entities (it would need to search the SensorPicture by
            // bearing, which is a host-side concern). The host reads
            // digi.threat.threatCallBearing + threatCallType each frame and
            // injects the resolved entity via FrameInputs.injectedThreat.
            //
            // The bearing is in msg->payload.heading (radians, relative to
            // self heading). The type is stored as the int cast of MessageType
            // so the host can distinguish Spike/Missile/SAM/BuddySpike.
            case MessageType::ThreatCallSpike:
            case MessageType::ThreatCallMissile:
            case MessageType::ThreatCallSAM:
            case MessageType::ThreatCallBuddySpike:
                digi.threat.threatCallBearing = msg->payload.heading;
                digi.threat.threatCallType = static_cast<int>(msg->type);
                break;

            // --- Flight reports (wingman → lead) ---
            // These are informational — the lead brain can read them to
            // track wingman status. For now, we don't act on them (a future
            // CommandFlight port will use them for lead decision-making).
            case MessageType::FlightReportBingo:
            case MessageType::FlightReportWinchester:
            case MessageType::FlightReportSplash:
            case MessageType::FlightReportBandit:
            case MessageType::FlightReportTally:
            case MessageType::FlightReportNoJoy:
            case MessageType::FlightReportFlameout:
            case MessageType::FlightReportRequestHelp:
                // Informational — no action yet. A future CommandFlight port
                // will consume these for lead decision-making.
                break;

            default:
                break;
        }
    }
}

void RunTaxi(DigiState& digi, const AircraftState& as,
             FcsState& fcsState, double dt) {
    (void)dt;  // reserved for future speed-proportional taxi steering
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

            // Deploy flaps for approach. FreeFalcon's TrackPointLanding calls
            // af->SetFlaps(true) (mnvers.cpp:107). Flaps increase CLmax (lower
            // stall speed) and CD (steeper descent at lower speed). Without
            // flaps, the approach speed is too high and the flare can't arrest
            // the descent. Half flaps for approach, full flaps below 200 ft.
            if (altAGL < 200.0) {
                digi.commands.tefCmd = 1.0;  // full flaps
                digi.commands.lefCmd = 1.0;
            } else {
                digi.commands.tefCmd = 0.5;  // approach flaps (half)
                digi.commands.lefCmd = 0.5;
            }

            // Glideslope beam tracking.
            //
            // The previous code used GammaHold(-3°) — a flight-path-angle
            // controller that does NOT reference the actual glideslope beam.
            // If the aircraft's gamma diverged from -3° (due to speed changes,
            // initial condition mismatch, or Phugoid transients), GammaHold
            // would blindly hold -3° while the aircraft flew steeper and
            // steeper. The aircraft would arrive at flare altitude with a
            // 80+ ft/s descent rate (6x the correct 15 ft/s), and the flare
            // couldn't arrest it — producing a nose-first "landing" with no
            // flare.
            //
            // FIX: compute the desired altitude from the distance to the
            // threshold and the 3° glideslope angle, then track that altitude
            // with a PD controller (proportional on altitude error, derivative
            // on descent rate). This is a beam-tracking approach: if the
            // aircraft is above the beam, it descends faster; if below, it
            // levels off. The aircraft stays on the beam regardless of speed
            // or gamma transients.
            const double dx = go.runwayThresholdX - as.kin.x;
            const double dy = go.runwayThresholdY - as.kin.y;
            const double distToThreshold = std::sqrt(dx * dx + dy * dy);

            // Desired altitude on the 3° glideslope beam.
            // At the threshold (dist=0), desired alt = groundZ (touchdown).
            // At 3 NM (18228 ft), desired alt = 955 ft AGL.
            const double desiredAltAGL = distToThreshold * std::tan(kApproachGlideslope * DTR);

            // Glideslope beam tracker with Phugoid damping.
            //
            // This is a PD controller on altitude error:
            //   alterr = (beam_alt - actual_alt)              [P term]
            //          + Kd * (beam_descent_rate - zdot)      [D term]
            //
            // The P term brings the aircraft back to the beam. The D term
            // damps the approach: if descending faster than the beam rate,
            // reduce the descent; if slower, increase it.
            //
            // NED: z is negative (altitude = -z), zdot > 0 = descending.
            // beam_alt_AGL + as.kin.z = beam_alt - actual_alt (both AGL).
            // beam_descent_rate = V * sin(3°) ≈ 15 ft/s at approach speed.
            //
            // The GammaHold integrator is cleared each frame to prevent windup.
            const double altError = desiredAltAGL + as.kin.z;  // + = below beam
            const double beamDescentRate = as.kin.vt * std::sin(kApproachGlideslope * DTR);
            const double descentRateError = beamDescentRate - as.kin.zdot;  // + = too slow
            const double alterr = altError * 1.0 + descentRateError * 2.0;

            // Approach speed
            const double approachSpeed = kApproachSpeedFraction *
                (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);

            // Steer toward the runway threshold (heading)
            const double desHeading = std::atan2(dy, dx);
            const double headingErr = headingError(desHeading, as.kin.sigma);
            digi.commands.rStick = std::max(-1.0, std::min(1.0, headingErr * RTD * 2.0 * DTR));

            // GammaHold with the PD error. Gain 0.015: 1° gamma per ~67 ft of
            // altitude error (or 33 ft/s of descent rate error). This is
            // gentler than the previous 0.15 — the PD formulation already
            // provides the error amplification, so the GammaHold gain just
            // scales it to a gamma command.
            digi.nav.gammaHoldIError = 0.0;  // pure P, no integrator windup
            ManeuverPrimitives::GammaHold(alterr * 0.015, digi, as, digi.config.maxGs);
            // Phugoid damper: damps the long-period pitch oscillation.
            ManeuverPrimitives::PhugoidDamper(digi, as);

            // Throttle: hold approach speed.
            // Use an auto-throttle integrator (like FreeFalcon's TrackPointLanding)
            // for smooth throttle management. The integrator learns the trim
            // throttle for the current drag configuration (gear + speed brakes).
            //
            // The descent trades altitude for speed, so the autothrottle must
            // aggressively reduce power when above approach speed. The previous
            // thresholds (+150 kts for burner, -100 kts for idle) were far too
            // loose — the aircraft would arrive at flare altitude 25+ kts too
            // fast because the throttle didn't go to idle soon enough.
            const double eProp = approachSpeed - as.vcas;
            if (eProp >= 50.0) {
                // Way too slow — add power (but not full burner on approach)
                digi.commands.throttle = std::min(1.0, 0.3 + eProp * 0.01);
            } else if (eProp <= -15.0) {
                // Too fast — idle. The descent will keep adding speed, so we
                // must cut power early to arrive at approach speed.
                digi.commands.throttle = 0.0;
            } else {
                // PI throttle: proportional + integral on speed error,
                // with vtDot feedback for damping.
                const double vtDotClamped = std::max(-10.0, std::min(10.0, as.vtDot));
                digi.nav.autoThrottle += (eProp - vtDotClamped * 5.0) * 0.001 * dt;
                digi.nav.autoThrottle = std::max(0.0, std::min(0.5, digi.nav.autoThrottle));
                digi.commands.throttle = std::max(0.0, std::min(0.6,
                    eProp * 0.015 + digi.nav.autoThrottle));
            }

            // Extend speed brakes if above approach speed.
            // Aggressive thresholds: deploy at +5 kts (half) and +10 kts (full).
            // The descent adds 25+ kts from the Phugoid, so we must deploy
            // early to arrive at flare altitude at the correct speed.
            if (as.vcas > approachSpeed + 10.0) {
                digi.commands.speedBrakeCmd = 1.0;  // full extend
            } else if (as.vcas > approachSpeed + 5.0) {
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
            // Flare: gradually arrest the descent rate AND raise the pitch
            // attitude for a gentle main-gear-first touchdown.
            //
            // The flare has two objectives:
            //   1. Arrest the descent rate (target decreases with altitude)
            //   2. Raise the pitch attitude to a touchdown attitude (5° nose up)
            //
            // The previous code only targeted descent rate — the pitch attitude
            // stayed low because the descent-rate controller doesn't naturally
            // produce a nose-up attitude (it just reduces the descent). Real
            // pilots flare to a PITCH ATTITUDE, not a descent rate — they pull
            // the nose up to ~5° and hold it there as the aircraft settles.
            //
            // This implementation blends both: the descent-rate controller
            // provides the primary pitch command, and a pitch-attitude target
            // adds a nose-up bias that increases as the aircraft gets close to
            // the ground. The pitch-attitude target ensures the aircraft
            // touches down nose-up (main gear first), not nose-down.
            //
            // THROTTLE MANAGEMENT: the previous code set throttle=0 (idle) for
            // the entire flare. This caused the aircraft to decelerate rapidly
            // (195→81 kts), and at 81 kts the elevator had no authority to
            // raise the nose — producing a nose-down touchdown. Real pilots
            // hold approach power until the flare is established, then smoothly
            // reduce to idle. We hold a small throttle (15%) above 30 ft to
            // maintain elevator authority, then idle below 30 ft.
            digi.commands.speedBrakeCmd = -1.0;  // retract brakes for flare
            digi.commands.tefCmd = 1.0;  // full flaps for flare
            digi.commands.lefCmd = 1.0;

            // Throttle: hold approach power to maintain elevator authority.
            // The key insight: at low speed the elevator has almost no
            // authority (the FCS clamps ptcmd to gsAvail, which is tiny at
            // low qbar). We MUST keep the speed above ~130 kts through the
            // flare or the nose won't come up. But too much power causes a
            // float (the aircraft won't descend). Balance: hold 55% power
            // to maintain speed without floating. Below 5 ft, reduce to
            // idle for touchdown. If speed drops below approach-20, add power.
            //
            // CRITICAL: do NOT cut throttle to 0 abruptly at 10 ft — the
            // sudden drag causes a 20+ kt speed drop in 2 seconds, and at
            // 80 kts the elevator has no authority to hold the nose up.
            // Instead, hold a small amount of power (20%) until 5 ft, then
            // idle. This keeps the speed above the elevator-authority floor
            // (~100 kts for most fighters) through the touchdown.
            const double approachSpeed = kApproachSpeedFraction *
                (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);
            if (altAGL < 5.0) {
                digi.commands.throttle = 0.0;  // idle for touchdown
            } else if (as.vcas < approachSpeed - 20.0) {
                // Too slow — add power to maintain elevator authority
                digi.commands.throttle = 0.8;
            } else if (altAGL < 15.0) {
                // Close to ground — hold 20% power to prevent speed drop
                digi.commands.throttle = 0.2;
            } else {
                // Hold approach power (55%) — enough for authority, not
                // enough to float. The descent-rate controller will still
                // bring the aircraft down.
                digi.commands.throttle = 0.55;
            }

            // Target descent rate decreases with altitude
            const double targetDescentRate = std::max(2.0,
                15.0 * std::max(0.0, altAGL) / kFlareAltFt);
            // NED: zdot > 0 = descending. Error = actual - target.
            const double descentErr = as.kin.zdot - targetDescentRate;
            // Descent-rate pitch command (primary).
            // Positive error = descending too fast → pitch up.
            const double descentPstick = descentErr * 0.05;

            // Pitch attitude target: 10° nose up at touchdown, ramping from 0°
            // at flare-start altitude to 10° at ground level. This biases the
            // pitch command nose-up so the aircraft settles in a main-gear-
            // first attitude. 10° is a typical fighter touchdown attitude.
            const double flareProgress = std::max(0.0, std::min(1.0,
                1.0 - altAGL / kFlareAltFt));  // 0 at flare start, 1 at ground
            const double targetPitchDeg = 10.0 * flareProgress;  // 0° → 10°
            const double pitchErr = targetPitchDeg - as.kin.theta * RTD;
            // Pitch-attitude command (secondary, adds to descent command).
            // Gain 0.08: aggressive nose-up command to overcome the low-speed
            // elevator authority problem.
            const double pitchPstick = pitchErr * 0.08;

            // Combined pitch command: descent-rate + pitch-attitude.
            // CLAMP TO NOSE-UP: during flare, never command nose-down. The
            // minimum pstick ramps up with flare progress to force the nose
            // up as the aircraft settles.
            const double minFlarePstick = 0.15 + 0.35 * flareProgress;  // 0.15 → 0.50
            digi.commands.pStick = std::max(minFlarePstick, std::min(1.0,
                descentPstick + pitchPstick));

            // Wings level
            fcsState.maxRoll = 0.0;
            const double rollDeg = as.kin.phi * RTD;
            digi.commands.rStick = std::max(-1.0, std::min(1.0, -rollDeg * 2.0 * DTR));

            // Check for touchdown
            if (altAGL < 5.0) {
                go.phase = GroundOpsPhase::Touchdown;
                go.touchdownSpeed = as.vcas;
                go.touchdownDescentRate = as.kin.zdot;
                go.touchdownPitch = as.kin.theta;
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
