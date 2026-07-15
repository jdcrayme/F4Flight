// f4flight - steering.cpp
//
// COMPATIBILITY SHIM — delegates to the digi/ subsystem.
//
// This file now contains only:
//   1. The utility functions (headingError, turnCompensatedG)
//   2. SteeringController::compute() — which maps the old Mode enum to
//      digi::DigiBrain calls and delegates to the brain.
//
// All maneuver logic (SetPstick, GammaHold, MachHold, LevelTurn, etc.) has
// moved to src/digi/maneuvers/maneuver_primitives.cpp. Ground avoidance is
// in src/digi/ground/ground_avoid.cpp. The brain dispatcher is in
// src/digi/digi_brain.cpp.

#include "f4flight/digi/steering.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <cmath>

namespace f4flight {

// ===========================================================================
// Utility functions (kept for backward compatibility)
// ===========================================================================
double headingError(double setpoint, double current) noexcept {
    double err = setpoint - current;
    while (err >  PI) err -= 2.0 * PI;
    while (err < -PI) err += 2.0 * PI;
    return err;
}

double turnCompensatedG(const AircraftState& state) noexcept {
    const double cosphi = std::max(0.1, state.kin.cosphi);
    return 1.0 / cosphi;
}

// ===========================================================================
// SteeringController — thin facade over digi::DigiBrain
// ===========================================================================

SteeringController::SteeringController() {
    // brain_ default-constructs with Veteran skill and Waypoint mode.
}

PilotInput SteeringController::compute(const AircraftState& state, double dt,
                                       double groundZ,
                                       const FlightControlSystem& fcs,
                                       FcsState& fcsState) {
    // Manual mode bypasses the brain entirely
    if (mode_ == Mode::Manual) {
        return manual_;
    }

    // Clear any forced mode so the brain's own resolveMode() can handle
    // threat-based mode switching (MissileDefeat, GunsJink pre-empt
    // navigation). The SteeringController's Mode enum only configures the
    // brain's navigation defaults — it does NOT override threat response.
    //
    // EXCEPTION: when the SteeringController is in Loiter mode, keep the
    // forced Loiter mode so the brain's activeMode() returns Loiter and the
    // Loiter primitive runs through the brain's dispatch (not just the
    // SteeringController override). Without this, the brain resolves to
    // Waypoint and the Loiter override fights the Waypoint heading-hold.
    if (mode_ != Mode::Loiter) {
        brain_.clearForcedMode();
    }

    // Delegate to the brain. The brain runs GroundCheck, resolves the mode
    // (GroundAvoid > MissileDefeat > GunsJink > Waypoint), dispatches to
    // the active mode's maneuver function, and clamps outputs.
    PilotInput out = brain_.compute(state, dt, groundZ, fcs, fcsState);

    // For Loiter mode, override the brain's waypoint logic with a direct
    // call to ManeuverPrimitives::Loiter. This preserves the old behavior
    // where Loiter is a 30° bank orbit, not waypoint following.
    if (mode_ == Mode::Loiter) {
        // Loiter/MachHold take DigiState& (they write pStick/rStick/throttle).
        // Use stateMutable() to avoid the [[deprecated]] non-const state() shim.
        digi::DigiState& s = brain_.stateMutable();
        digi::ManeuverPrimitives::Loiter(s, state, fcs, fcsState, s.config.maxGs);
        digi::ManeuverPrimitives::MachHold(s.config.cornerSpeed, state.vcas, true,
                                            s, state, 200.0, 800.0, dt, 700.0);
        out.pstick = limit(s.commands.pStick, -1.0, 1.0);
        out.rstick = limit(s.commands.rStick, -1.0, 1.0);
        out.ypedal = limit(s.commands.yPedal, -1.0, 1.0);
        out.throttle = limit(s.commands.throttle, 0.0, 1.5);
        out.refueling = false;
    }

    return out;
}

} // namespace f4flight
