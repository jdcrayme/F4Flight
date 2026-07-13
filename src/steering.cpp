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

#include "f4flight/steering.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

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

    // Map the old Mode enum to digi::DigiBrain mode forcing.
    // The brain's own resolveMode() handles priority; we just set the
    // forced mode for the nav modes (HeadingAltitude, Waypoint, Loiter).
    // TerrainFollow, Formation, Combat are not yet implemented in the brain;
    // they fall through to the default (Waypoint) behavior.
    switch (mode_) {
        case Mode::HeadingAltitude:
            brain_.setForcedMode(digi::DigiMode::Waypoint);
            break;
        case Mode::Waypoint:
            brain_.setForcedMode(digi::DigiMode::Waypoint);
            break;
        case Mode::Loiter:
            // Loiter is not a separate DigiMode yet; use Waypoint mode
            // with a single orbit point. For now, delegate to the brain's
            // waypoint logic.
            brain_.setForcedMode(digi::DigiMode::Waypoint);
            break;
        case Mode::TerrainFollow:
        case Mode::Formation:
        case Mode::Combat:
            // Not yet implemented in the brain — fall through to Waypoint
            brain_.setForcedMode(digi::DigiMode::Waypoint);
            break;
        case Mode::Manual:
            return manual_;
    }

    // Delegate to the brain. The brain runs GroundCheck, resolves the mode,
    // dispatches to the active mode's maneuver function, and clamps outputs.
    PilotInput out = brain_.compute(state, dt, groundZ, fcs, fcsState);

    // For Loiter mode, override the brain's waypoint logic with a direct
    // call to ManeuverPrimitives::Loiter. This preserves the old behavior
    // where Loiter is a 30° bank orbit, not waypoint following.
    if (mode_ == Mode::Loiter) {
        digi::ManeuverPrimitives::Loiter(brain_.state(), state, fcs, fcsState,
                                          brain_.state().maxGs);
        digi::ManeuverPrimitives::MachHold(brain_.state().cornerSpeed, state.vcas, true,
                                            brain_.state(), state, 200.0, 800.0, dt, 700.0);
        out.pstick = limit(brain_.state().pStick, -1.0, 1.0);
        out.rstick = limit(brain_.state().rStick, -1.0, 1.0);
        out.ypedal = limit(brain_.state().yPedal, -1.0, 1.0);
        out.throttle = limit(brain_.state().throttle, 0.0, 1.5);
        out.refueling = false;
    }

    return out;
}

} // namespace f4flight
