// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// atmosphere.h
//
// 3-layer standard atmosphere model.
// Port of atmos.cpp.
//
// Units: Imperial (ft, slugs/ft^3, lb/ft^2, ft/s, Rankine).
// Input  : altitude in feet (positive upward)
// Output : density (rho), pressure (pa), temperature ratio (ttheta),
//          density ratio (rsigma), pressure ratio (pdelta), speed of sound,
//          mach number, dynamic pressure (qbar), calibrated airspeed (vcas).

#pragma once

#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

// Output of the atmosphere computation.
struct AtmosphereOutput {
    double rho{0.0};       // slugs/ft^3
    double pa{0.0};        // lb/ft^2
    double ttheta{1.0};    // T/T0
    double rsigma{1.0};    // rho/rho0
    double pdelta{1.0};    // P/P0
    double sound{AASL};    // ft/s
    double mach{0.0};      // dimensionless
    double qbar{0.0};      // lb/ft^2
    double qovt{0.0};      // qbar / vt
    double qsom{0.0};      // q * S / m  (normalised dynamic pressure, ft/s^2 per unit CL)
    double vcas{0.0};      // knots (calibrated airspeed)
};

// Compute the pressure ratio (P/P0) and the temperature/density ratios
// (T/T0, rho/rho0) using the 3-layer standard atmosphere model from
// atmos.cpp.
//
// alt is in FEET and POSITIVE UPWARD.
inline double calcPressureRatio(double alt_ft,
                                double& ttheta,
                                double& rsigma) noexcept {
    if (alt_ft <= TROPO_ALT_FT) {
        // Troposphere
        ttheta = 1.0 - TROPO_LAPSE * alt_ft;
        rsigma = std::pow(ttheta, TROPO_RHO_EXP);
    } else if (alt_ft < TROPO_ALT2_FT) {
        // Lower stratosphere (isothermal)
        ttheta = STRATO_TTHETA;
        rsigma = STRATO_RHO_BASE * std::exp(STRATO_RHO_K * (TROPO_ALT_FT - alt_ft));
    } else {
        // Upper stratosphere
        ttheta = 0.682457 + alt_ft / 945374.0;
        rsigma = std::pow(0.978261 + alt_ft / 659515.0, -35.16319);
    }
    return ttheta * rsigma; // P/P0
}

// Compute Mach number from KCAS using the inverse of the calibrated-airspeed
// formula. For subsonic Mach, the inverse is closed-form; for supersonic
// Mach, a Newton-Raphson iteration on the Rayleigh pitot formula is used.
//
//   kcas  : calibrated airspeed in knots
//   pa    : ambient pressure in lb/ft^2
// Returns Mach (dimensionless). For kcas <= 0 returns 0.
inline double calcMachFromKcas(double kcas, double pa) noexcept {
    if (kcas <= 0.0 || pa <= 0.0) return 0.0;

    // First compute the impact pressure ratio qc/pa from kcas.
    // qc/P0 = (1 + 0.2 M_kcas^2)^3.5 - 1   where M_kcas = kcas / a_sl
    const double M_kcas = kcas / AASLK;
    const double qc = PASL * (std::pow(1.0 + 0.2 * M_kcas * M_kcas, 3.5) - 1.0);
    const double qcpa_val = qc / pa;

    // Try the subsonic inverse first: qcpa = (1 + 0.2 M^2)^3.5 - 1
    // => 1 + qcpa = (1 + 0.2 M^2)^3.5
    // => M^2 = 5 * ((1+qcpa)^(1/3.5) - 1)
    const double M2_sub = 5.0 * (std::pow(1.0 + qcpa_val, 1.0 / 3.5) - 1.0);
    if (M2_sub <= 1.0) {
        // Subsonic solution is valid
        return std::sqrt(std::max(0.0, M2_sub));
    }

    // Supersonic: iterate the Rayleigh formula.
    //   f(u) = 166.921 u^7 / (7 u^2 - 1)^2.5 - (1 + qc/pa)
    //   f'(u) = 7*166.921*u^6*(2 u^2 - 1) / (7 u^2 - 1)^3.5
    double u = std::sqrt(M2_sub); // initial guess from subsonic formula (>1)
    if (u < 1.01) u = 1.01;
    for (int iter = 0; iter < 32; ++iter) {
        const double u2 = u * u;
        const double denom = 7.0 * u2 - 1.0;
        if (denom <= 0.0) { u = 1.05; continue; }
        const double fu  = 166.921 * std::pow(u, 7.0) / std::pow(denom, 2.5) - (1.0 + qcpa_val);
        const double fpu = 7.0 * 166.921 * std::pow(u, 6.0) * (2.0 * u2 - 1.0) / std::pow(denom, 3.5);
        if (std::fabs(fpu) < 1e-12) break;
        const double delta = fu / fpu;
        u -= delta;
        if (std::fabs(fu) < 0.001) break;
        if (u < 1.01) u = 1.01;
        if (u > 5.0)  u = 5.0;
    }
    return u;
}

// Compute KCAS from Mach and ambient pressure.
// Direct port of atmos.cpp's atmosphere() body where vcas is derived.
inline double calcKcasFromMach(double mach, double pa) noexcept {
    if (mach < 0.0) mach = 0.0;
    double qc;
    if (mach <= 1.0) {
        qc = (std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0) * pa;
    } else {
        qc = (166.9 * std::pow(mach, 7.0) / std::pow(7.0 * mach * mach - 1.0, 2.5) - 1.0) * pa;
    }
    const double qpasl1 = qc / PASL + 1.0;
    double vcas = 1479.12 * std::sqrt(std::pow(qpasl1, 0.285714) - 1.0);
    if (qc > 1889.64) {
        // Supersonic CAS correction
        const double oper = qpasl1 * std::pow(7.0 - (AASLK * AASLK) / (vcas * vcas), 2.5);
        vcas = 51.1987 * std::sqrt(oper);
    }
    return vcas;
}

// Compute true airspeed (ft/s) from calibrated airspeed (kts) and altitude.
// Uses calcMachFromKcas to get Mach, then Mach * speed of sound at altitude.
// This is the inverse of the CAS calculation and lets us initialize the
// flight model at the correct TAS for a desired CAS.
inline double calcTasFromKcas(double kcas, double alt_ft) noexcept {
    if (kcas <= 0.0) return 0.0;
    double ttheta, rsigma;
    const double pdelta = calcPressureRatio(alt_ft, ttheta, rsigma);
    const double pa = pdelta * PASL;
    const double sound = std::sqrt(ttheta) * AASL;
    const double mach = calcMachFromKcas(kcas, pa);
    return mach * sound;
}

// Top-level atmosphere update.
//
//   alt_ft     : altitude above sea level, feet, positive upward
//   vt_ftps    : true airspeed, ft/s
//   area_ft2   : wing reference area, ft^2
//   mass_slugs : aircraft mass, slugs
inline AtmosphereOutput computeAtmosphere(double alt_ft,
                                          double vt_ftps,
                                          double area_ft2,
                                          double mass_slugs) noexcept {
    AtmosphereOutput out;

    const double pdelta = calcPressureRatio(alt_ft, out.ttheta, out.rsigma);
    out.pdelta = pdelta;
    out.sound  = std::sqrt(out.ttheta) * AASL;
    out.rho    = out.rsigma * RHOASL;
    out.pa     = pdelta * PASL;

    const double safe_vt = (vt_ftps > 1.0) ? vt_ftps : 1.0;
    out.mach = safe_vt / out.sound;
    out.qbar = 0.5 * out.rho * safe_vt * safe_vt;
    out.qovt = out.qbar / safe_vt;

    // qsom = q * S / m  (normalised so that CL * qsom has units of ft/s^2)
    const double safe_mass = (mass_slugs > 1e-6) ? mass_slugs : 1e-6;
    out.qsom = out.qbar * area_ft2 / safe_mass;

    out.vcas = calcKcasFromMach(out.mach, out.pa);

    return out;
}

} // namespace f4flight
