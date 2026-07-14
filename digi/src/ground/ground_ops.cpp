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
static constexpr double kFlareAltFt = 50.0;           // flare start altitude AGL
static constexpr double kApproachGlideslope = 3.0;    // degrees
static constexpr double kApproachSpeedFraction = 1.3; // V_approach = 1.3 * stallSpeed
static constexpr double kRolloutBrakeSpeed = 80.0;    // kts — start braking below this

void ProcessATCMessages(DigiState& digi, Mailbox& mailbox) {
    while (auto msg = mailbox.pop()) {
        switch (msg->type) {
            case MessageType::ATCClearedTakeoff:
                digi.groundOps.hasTakeoffClearance = true;
                digi.groundOps.assignedRunway = atc::runwayFromMessage(*msg);
                digi.groundOps.phase = GroundOpsPhase::LiningUp;
                break;

            case MessageType::ATCClearedLanding:
                digi.groundOps.hasLandingClearance = true;
                digi.groundOps.assignedRunway = atc::runwayFromMessage(*msg);
                break;

            case MessageType::ATCHoldShort:
                digi.groundOps.hasTakeoffClearance = false;
                digi.groundOps.phase = GroundOpsPhase::HoldingShort;
                break;

            case MessageType::ATCGoAround:
                digi.groundOps.hasLandingClearance = false;
                digi.groundOps.phase = GroundOpsPhase::Approach;
                break;

            case MessageType::ATCTaxiInstruction: {
                double tx, ty;
                atc::taxiPointFromMessage(*msg, tx, ty);
                digi.groundOps.runwayThresholdX = tx;
                digi.groundOps.runwayThresholdY = ty;
                digi.groundOps.phase = GroundOpsPhase::TaxiToRunway;
                break;
            }

            default:
                break;
        }
    }
}

void RunTaxi(DigiState& digi, const AircraftState& as,
             FcsState& fcsState, double dt) {
    auto& go = digi.groundOps;

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
        digi.throttle = 0.0;
        digi.pStick = 0.0;
        digi.rStick = 0.0;
        digi.wheelBrakes = true;  // hold position with brakes
        return;
    }
    digi.wheelBrakes = false;

    // Steer toward target
    const double desHeading = std::atan2(dy, dx);
    const double headingErr = headingError(desHeading, as.kin.sigma);

    // Use rstick for steering (simplified — no nose-wheel steering model)
    ManeuverPrimitives::SetRstick(headingErr * RTD * 2.0, digi,
                                   FlightControlSystem{}, fcsState);

    // Throttle for taxi speed
    ManeuverPrimitives::MachHold(kTaxiSpeedKts, as.vcas, false,
                                  digi, as, 100.0, 200.0, dt, 100.0);

    // Wings level
    fcsState.maxRoll = 0.0;
    ManeuverPrimitives::SetPstick(0.0, 5.0, CommandType::GCommand, digi, as);
}

void RunTakeoff(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ) {
    (void)dt;  // takeoff uses discrete phase transitions, not dt
    auto& go = digi.groundOps;

    switch (go.phase) {
        case GroundOpsPhase::Idle:
            // Start takeoff sequence
            go.phase = GroundOpsPhase::TakeoffRoll;
            go.takeoffRollStart = simTime;
            go.gearRetracted = false;
            break;

        case GroundOpsPhase::TakeoffRoll: {
            // Full throttle (1.5 = AB if available, 1.0 = MIL otherwise)
            digi.throttle = 1.5;
            // Keep straight on runway heading (use rudder)
            const double headingErr = headingError(go.runwayHeading, as.kin.sigma);
            ManeuverPrimitives::SetRstick(headingErr * RTD * 3.0, digi,
                                           FlightControlSystem{}, fcsState);
            // Nose down (hold on runway)
            ManeuverPrimitives::SetPstick(-2.0, 5.0, CommandType::GCommand, digi, as);
            fcsState.maxRoll = 0.0;

            // Check for rotation speed
            // Estimate rotation speed as 1.1 * stallSpeed (simplified)
            const double rotationSpeed = 1.1 * (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);
            if (as.vcas >= rotationSpeed) {
                go.phase = GroundOpsPhase::Rotation;
            }
            break;
        }

        case GroundOpsPhase::Rotation: {
            // Pull up — rotate to climb attitude
            digi.throttle = 1.5;
            ManeuverPrimitives::SetPstick(3.0, digi.maxGs, CommandType::GCommand, digi, as);
            fcsState.maxRoll = 0.0;

            // Check if airborne (altitude > groundZ + 10ft)
            const double altAGL = -as.kin.z - groundZ;
            if (altAGL > 10.0) {
                go.phase = GroundOpsPhase::AfterTakeoff;
                go.gearRetracted = false;
            }
            break;
        }

        case GroundOpsPhase::AfterTakeoff: {
            // Gear up, climb out
            if (!go.gearRetracted && (-as.kin.z - groundZ) > kClimboutAltFt) {
                go.gearRetracted = true;
            }

            // Climb straight ahead — full throttle for climb
            digi.throttle = 1.5;
            const double targetAlt = groundZ + 1500.0;  // climb to 1500 ft AGL
            ManeuverPrimitives::HeadingAndAltitudeHold(
                go.runwayHeading, targetAlt, digi, as,
                FlightControlSystem{}, fcsState, digi.maxGs);
            break;
        }

        default:
            break;
    }
}

void RunLanding(DigiState& digi, const AircraftState& as,
                FcsState& fcsState, double dt, double simTime, double groundZ) {
    (void)dt;
    (void)simTime;
    auto& go = digi.groundOps;

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
            // FreeFalcon uses TrackPointLanding (mnvers.cpp:33) which calls
            // SimpleTrackElevation + SimpleTrackAzimuth — pure proportional
            // trackers with no integral, no +1G bias, no gamma feedback.
            // This is structurally incapable of the Phugoid oscillation
            // that GammaHold produces on a moving glideslope target.
            //
            // F4Flight previously used HeadingAndAltitudeHold + GammaHold,
            // which caused a Phugoid (descend → overshoot → climb → repeat).
            // Now we use the same TrackPointLanding primitive as FreeFalcon.
            const double dx = go.runwayThresholdX - as.kin.x;
            const double dy = go.runwayThresholdY - as.kin.y;
            const double distToThreshold = std::sqrt(dx * dx + dy * dy);

            // Desired altitude for 3° glideslope (NED: negative up)
            const double desAltAGL = distToThreshold * std::tan(kApproachGlideslope * DTR);
            const double desAltZ = -(groundZ + desAltAGL);  // NED z

            // Approach speed
            const double approachSpeed = kApproachSpeedFraction *
                (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);

            // Set trackpoint for TrackPointLanding
            digi.trackX = go.runwayThresholdX;
            digi.trackY = go.runwayThresholdY;
            digi.trackZ = desAltZ;

            ManeuverPrimitives::TrackPointLanding(approachSpeed, digi, as, dt);

            // Check for flare altitude (FF triggers on altitude alone,
            // not distance — landme.cpp:1036)
            if (altAGL < kFlareAltFt) {
                go.phase = GroundOpsPhase::Flare;
                go.flareStartAlt = altAGL;
            }
            break;
        }

        case GroundOpsPhase::Flare: {
            // FF flare: tiny constant pitch-down + idle throttle
            // (landme.cpp:1123: pStick = -0.01685393258427)
            // The previous code called SetPstick(1.0, GCommand) which
            // produces ZERO pstick due to the sqrt mapping in SetPstick
            // (stickCmd = clamp(1.0, ±maxGs) = 1.0 → sqrt((1-1)/...) = 0).
            digi.throttle = 0.0;
            digi.pStick = -0.02;  // tiny pitch-down to settle onto runway
            fcsState.maxRoll = 0.0;
            // Wings level
            const double rollDeg = as.kin.phi * RTD;
            digi.rStick = -rollDeg * 2.0 * DTR;
            digi.rStick = std::max(-1.0, std::min(1.0, digi.rStick));

            // Check for touchdown — the ground clamp in eom.cpp holds
            // the aircraft at ~5 ft AGL (strut compression), so use 10 ft
            // as the touchdown threshold.
            if (altAGL < 10.0) {
                go.phase = GroundOpsPhase::Touchdown;
                go.touchdownSpeed = as.vcas;
            }
            break;
        }

        case GroundOpsPhase::Touchdown: {
            // Main gear on ground — hold attitude, start deceleration
            digi.throttle = 0.0;
            digi.pStick = 0.0;  // level attitude
            fcsState.maxRoll = 0.0;

            // Transition to rollout
            go.phase = GroundOpsPhase::Rollout;
            break;
        }

        case GroundOpsPhase::Rollout: {
            // Decelerate — brakes + idle.
            // Round-2 structural fix (Rec 7): now that PilotInput.wheelBrakes
            // is plumbed through FlightModel::updateGear → calcMuFric, the
            // rollout can actually decelerate. Previously the brain set
            // throttle=0 but the aircraft kept rolling (no friction increase).
            digi.throttle = 0.0;
            digi.wheelBrakes = (as.vcas > 5.0);  // release brakes below 5 kts (stop)
            // Extend speed brakes for extra drag during rollout.
            digi.speedBrakeCmd = (as.vcas > 30.0) ? 1.0 : 0.0;
            // Keep straight on runway heading
            const double headingErr = headingError(go.runwayHeading, as.kin.sigma);
            ManeuverPrimitives::SetRstick(headingErr * RTD * 3.0, digi,
                                           FlightControlSystem{}, fcsState);
            // Hold attitude level — do NOT push negative G (the aircraft
            // is on the ground; pushing negative G drives it underground
            // because the F4Flight flight model has no ground reaction force).
            digi.pStick = 0.0;
            fcsState.maxRoll = 0.0;

            // Check if stopped or slow enough to vacate
            if (as.vcas < 10.0) {
                go.phase = GroundOpsPhase::VacatingRunway;
            }
            break;
        }

        case GroundOpsPhase::VacatingRunway: {
            // Taxi off runway — for simplicity, just hold position
            digi.throttle = 0.0;
            digi.pStick = 0.0;
            digi.rStick = 0.0;
            // In a full impl, this would taxi to a runway exit node
            break;
        }

        default:
            break;
    }
}

} // namespace digi
} // namespace f4flight
