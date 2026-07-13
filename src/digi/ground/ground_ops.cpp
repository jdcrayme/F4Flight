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
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"
#include "f4flight/steering.h"  // for headingError

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
    // Simple taxi: steer toward target point at taxi speed
    const double dx = digi.groundOps.runwayThresholdX - as.kin.x;
    const double dy = digi.groundOps.runwayThresholdY - as.kin.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 50.0) {
        // Reached target — stop
        digi.throttle = 0.0;
        digi.pStick = 0.0;
        digi.rStick = 0.0;
        return;
    }

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

            // Fly toward runway threshold, descending on 3° glideslope
            const double dx = go.runwayThresholdX - as.kin.x;
            const double dy = go.runwayThresholdY - as.kin.y;
            const double distToThreshold = std::sqrt(dx * dx + dy * dy);

            // Desired altitude for 3° glideslope
            const double desAltAGL = distToThreshold * std::tan(kApproachGlideslope * DTR);
            const double desAlt = groundZ + desAltAGL;

            // Approach speed
            const double approachSpeed = kApproachSpeedFraction *
                (as.aero.stallSpeed > 0 ? as.aero.stallSpeed : 130.0);

            // Steer toward threshold
            const double desHeading = std::atan2(dy, dx);
            ManeuverPrimitives::HeadingAndAltitudeHold(
                desHeading, desAlt, digi, as,
                FlightControlSystem{}, fcsState, digi.maxGs);
            ManeuverPrimitives::MachHold(approachSpeed, as.vcas, true,
                                          digi, as, 100.0, 300.0, dt, 100.0);

            // Check for flare altitude
            if (altAGL < kFlareAltFt && distToThreshold < 1000.0) {
                go.phase = GroundOpsPhase::Flare;
                go.flareStartAlt = altAGL;
            }
            break;
        }

        case GroundOpsPhase::Flare: {
            // Level off — reduce descent rate
            // Target: 1G, wings level, idle throttle
            digi.throttle = 0.0;
            ManeuverPrimitives::SetPstick(1.0, digi.maxGs, CommandType::GCommand, digi, as);
            fcsState.maxRoll = 0.0;

            // Check for touchdown
            if (altAGL < 2.0) {
                go.phase = GroundOpsPhase::Touchdown;
                go.touchdownSpeed = as.vcas;
            }
            break;
        }

        case GroundOpsPhase::Touchdown: {
            // Main gear on ground — hold attitude, start deceleration
            digi.throttle = 0.0;
            ManeuverPrimitives::SetPstick(0.0, 5.0, CommandType::GCommand, digi, as);
            fcsState.maxRoll = 0.0;

            // Transition to rollout
            go.phase = GroundOpsPhase::Rollout;
            break;
        }

        case GroundOpsPhase::Rollout: {
            // Decelerate — brakes + idle
            digi.throttle = 0.0;
            // Keep straight on runway heading
            const double headingErr = headingError(go.runwayHeading, as.kin.sigma);
            ManeuverPrimitives::SetRstick(headingErr * RTD * 3.0, digi,
                                           FlightControlSystem{}, fcsState);
            ManeuverPrimitives::SetPstick(-1.0, 5.0, CommandType::GCommand, digi, as);
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
