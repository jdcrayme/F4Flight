// f4flight - digi/ground/ground_avoid.cpp
//
// Ground avoidance implementation.
//
// Direct port concept of FreeFalcon ground.cpp (GroundCheck + PullUp).
// Simplified to use a flat-earth ground model (groundZ passed in by the host).
// A real terrain sampler can replace the groundZ parameter later.
//
// FreeFalcon ground.cpp:24 (GroundCheck):
//   - Computes turn radius from SustainedGs(TRUE) and current roll rate
//   - Looks ahead along predicted recovery path (sampling every 0.5s)
//   - Sets groundAvoidNeeded when altitude above terrain < 2*turnRadius
//
// FreeFalcon ground.cpp:208 (PullUp):
//   - MachHold(cornerSpeed - 100)  — smallest turn-radius speed
//   - Limits roll to gaRoll (wings level)
//   - Full pstick (1.0) when within g_nCriticalPullup time-to-impact
//   - pullupTimer holds the pull for g_fPullupTime seconds

#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// Constants from FreeFalcon ground.cpp / f4config.cpp
static constexpr double kPullupTime = 2.0;       // seconds to hold the pull
static constexpr double kCriticalPullup = 1.0;   // seconds-to-impact threshold
static constexpr double kMinAltMargin = 100.0;   // ft, minimum clearance

bool GroundCheck(DigiState& digi, const AircraftState& state,
                 double groundZ, double lookahead) {
    // Current altitude AGL (ft, positive up)
    const double altAGL = -state.kin.z - groundZ;

    // If already well above terrain, no avoidance needed
    if (altAGL > 5000.0) {
        digi.groundAvoid.groundAvoidNeeded = false;
        return false;
    }

    // Project the aircraft's current flight path forward by `lookahead` seconds.
    // Sample at 0.5s intervals. For flat-earth: terrain = groundZ.
    // The predicted altitude at time t is: alt_now + climbRate * t
    const double climbRate = state.kin.vt * state.kin.singam;  // ft/s, positive up
    const double sampleDt = 0.5;
    const int numSamples = static_cast<int>(lookahead / sampleDt);

    // Minimum clearance threshold. FreeFalcon uses 2*turnRadius but that's
    // 19000+ ft at 350 kts, which triggers constantly in normal cruise.
    // Use a fixed 500 ft clearance instead — this matches the intent (don't
    // hit the ground) without false positives in level flight.
    constexpr double kMinClearance = 500.0;

    bool needed = false;
    for (int i = 1; i <= numSamples; ++i) {
        const double t = i * sampleDt;
        const double predAlt = (-state.kin.z) + climbRate * t;
        const double predAGL = predAlt - groundZ;
        if (predAGL < kMinClearance) {
            needed = true;
            break;
        }
    }

    // Also check current altitude
    if (altAGL < kMinClearance) {
        needed = true;
    }

    digi.groundAvoid.groundAvoidNeeded = needed;
    return needed;
}

void PullUp(DigiState& digi, const AircraftState& state,
            double cornerSpeed, double dt,
            FcsState& fcsState, double maxGs) {
    // MachHold at cornerSpeed - 100 (smallest turn-radius speed for recovery)
    // FreeFalcon ground.cpp:218: MachHold(af->CornerVcas() - 100.0F)
    const double recoverySpeed = std::max(150.0, cornerSpeed - 100.0);
    ManeuverPrimitives::MachHold(recoverySpeed, state.vcas, false,
                                  digi, state, 200.0, 800.0, dt, 100.0);

    // Wings level — limit roll to 0°
    // FreeFalcon ground.cpp:225: SetMaxRoll(0.0F)
    fcsState.maxRoll = 0.0;
    fcsState.maxRollDelta = 5.0;

    // Level the wings via proportional roll control
    const double rollDeg = state.kin.phi * RTD;
    const double rollErr = -rollDeg * 2.0;
    const double kr01 = fcsState.kr01;
    const double tr01 = fcsState.tr01;
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollErr * DTR * 0.75 / denom;
    constexpr double kTau = 0.0373;
    const double a = 1.0 - std::exp(-dt / kTau);
    digi.commands.rStick = (1.0 - a) * digi.commands.rStick + a * stickCmd;

    // Full pstick (max G away from ground)
    // FreeFalcon ground.cpp:233: SetPstick(maxGs, maxGs, GCommand)
    ManeuverPrimitives::SetPstick(maxGs, maxGs, CommandType::GCommand, digi, state);

    ManeuverPrimitives::SetYpedal(0.0, digi);

    // Hold the pull for kPullupTime seconds
    digi.groundAvoid.pullupTimer = kPullupTime;
}

bool RunGroundAvoid(DigiState& digi, const AircraftState& state,
                    double groundZ, double cornerSpeed, double dt,
                    FcsState& fcsState, double maxGs,
                    double lookahead) {
    // Always run the check
    const bool needed = GroundCheck(digi, state, groundZ, lookahead);

    // If a pull-up is already in progress, continue it
    if (digi.groundAvoid.pullupTimer > 0.0) {
        PullUp(digi, state, cornerSpeed, dt, fcsState, maxGs);
        digi.groundAvoid.pullupTimer -= dt;
        return true;
    }

    // If the check says we need a pull-up, start one
    if (needed) {
        PullUp(digi, state, cornerSpeed, dt, fcsState, maxGs);
        digi.groundAvoid.pullupTimer -= dt;
        return true;
    }

    return false;
}

} // namespace digi
} // namespace f4flight
