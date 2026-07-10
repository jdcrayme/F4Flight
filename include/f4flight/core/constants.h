// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/constants.h
//
// Physical constants. Preserved in Imperial units (feet, slugs, Rankine, ft/s)
// to match the original Falcon 4 aerodynamic coefficient tables exactly.
//
// Unless otherwise noted, all quantities are in Imperial units throughout the
// library. Conversions to SI are provided at the bottom of this file.

#pragma once

#include <cmath>

namespace f4flight {

// ---------------------------------------------------------------------------
// Angle conversions
// ---------------------------------------------------------------------------
constexpr double RTD = 57.2957795130823208767;  // radians -> degrees
constexpr double DTR = 0.017453292519943295769; // degrees -> radians

// ---------------------------------------------------------------------------
// Circle constants
// ---------------------------------------------------------------------------
constexpr double PI       = 3.14159265358979323846;
constexpr double HALF_PI  = 1.57079632679489661923;
constexpr double TWO_PI   = 6.28318530717958647692;

// ---------------------------------------------------------------------------
// Gravitational acceleration
// Note: the original Falcon 4 code uses 32.177 ft/s^2 (slightly off from the
// standard 32.17405 ft/s^2). We preserve the legacy value so that the
// coefficient tables produce the same flight feel.
// ---------------------------------------------------------------------------
constexpr double GRAVITY = 32.177; // ft/s^2

// ---------------------------------------------------------------------------
// Sea-level standard atmosphere (Imperial)
// ---------------------------------------------------------------------------
constexpr double RHOASL = 0.0023769; // slugs/ft^3  air density at sea level
constexpr double PASL   = 2116.22;   // lb/ft^2     pressure at sea level
constexpr double AASL   = 1116.44;   // ft/s        speed of sound at sea level
constexpr double AASLK  = 661.48;    // knots       speed of sound at sea level
constexpr double TASL   = 518.7;     // deg R       temperature at sea level

// ---------------------------------------------------------------------------
// Unit conversions (Imperial <-> nautical / SI)
// ---------------------------------------------------------------------------
constexpr double FTPSEC_TO_KNOTS = 0.592474;   // ft/s  -> knots
constexpr double KNOTS_TO_FTPSEC = 1.687836;   // knots -> ft/s
constexpr double FT_TO_METERS    = 0.3048;     // ft    -> m
constexpr double METERS_TO_FT    = 3.28084;    // m     -> ft
constexpr double NM_TO_FT        = 6076.11549; // nm    -> ft
constexpr double LBS_TO_KG       = 0.453592;   // lb    -> kg
constexpr double SLUGS_TO_KG     = 14.5939;    // slugs -> kg

// ---------------------------------------------------------------------------
// Earth model
// ---------------------------------------------------------------------------
constexpr double EARTH_RADIUS_FT = 2.09257e7;  // ft
constexpr double EARTH_RADIUS_NM = 3443.92228; // nm

// ---------------------------------------------------------------------------
// Atmosphere layer breakpoints (standard atmosphere, 1962/1976 US)
// ---------------------------------------------------------------------------
constexpr double TROPO_ALT_FT  = 36089.0;  // ft  tropopause
constexpr double TROPO_ALT2_FT = 65617.0;  // ft  second breakpoint

// ---------------------------------------------------------------------------
// Atmosphere lapse/gradient constants (derived from the formulas used in
// atmos.cpp; exposed so tests can validate them independently)
// ---------------------------------------------------------------------------
constexpr double TROPO_LAPSE      = 6.875e-6;   // 1/ft   temperature lapse rate
constexpr double TROPO_RHO_EXP    = 4.255876;   //        rho exponent (gamma)
constexpr double STRATO_TTHETA    = 0.751865;   //        T/T0 in lower stratosphere
constexpr double STRATO_RHO_BASE  = 0.297076;   //        rho ratio base at tropopause
constexpr double STRATO_RHO_K     = 4.806e-5;   // 1/ft   rho exponential coefficient

} // namespace f4flight
