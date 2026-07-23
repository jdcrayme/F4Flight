// f4flight - steering_utils.h
//
// Core utilities for heading error calculation and turn-compensated G calculation.
// Extracted from the legacy steering shim.

#pragma once

#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/core/constants.h"
#include <cmath>
#include <algorithm>

namespace f4flight {

inline double headingError(double setpoint, double current) noexcept {
    double err = setpoint - current;
    while (err >  PI) err -= 2.0 * PI;
    while (err < -PI) err += 2.0 * PI;
    return err;
}

inline double turnCompensatedG(const AircraftState& state) noexcept {
    const double cosphi = std::max(0.1, state.kin.cosphi);
    return 1.0 / cosphi;
}

} // namespace f4flight
