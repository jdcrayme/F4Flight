// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aerodynamics.cpp
//
// Aerodynamics force model implementation. Port of aero.cpp.

#include "f4flight/aerodynamics.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

Aerodynamics::Aerodynamics(const AeroTable* table,
                           const AircraftGeometry* geom,
                           const AuxAero* aux)
    : table_(table), geom_(geom), aux_(aux) {
    if (table_) {
        cl_ = table_->makeClLookup();
        cd_ = table_->makeCdLookup();
        cy_ = table_->makeCyLookup();
    }
}

void Aerodynamics::update(double alpha_deg,
                          double beta_deg,
                          double mach,
                          double vt_ftps,
                          double qbar,
                          double qsom,
                          double qovt,
                          double altitude_ft,
                          double groundZ_ft,
                          double z_ft,
                          double vcas_kts,
                          double pstick,
                          AeroState& aero) const {
    if (!table_ || !geom_ || !aux_) {
        aero.lift = aero.drag = 0.0;
        aero.xaero = aero.yaero = aero.zaero = 0.0;
        return;
    }

    // --- Trig ---
    const double alp_rad = alpha_deg * DTR;
    const double bet_rad = beta_deg * DTR;
    const double cosalp = std::cos(alp_rad);
    const double sinalp = std::sin(alp_rad);
    const double cosbet = std::cos(bet_rad);
    const double sinbet = std::sin(bet_rad);

    // --- Flap factors ---
    // TEF/LEF are normalised 0..1 (already stored in aero.tefPos / lefPos).
    double tefFactor = aero.tefPos;
    double lefFactor = aero.lefPos;
    // Effective alpha for CL lookup: alpha + tef - lef
    const double tempAlpha = alpha_deg + tefFactor - lefFactor;

    // --- Coefficient lookups ---
    double cl = cl_(mach, tempAlpha) * table_->clFactor;
    double cy = cy_(mach, alpha_deg)  * table_->cyFactor;

    // Drag uses a "reduced alpha" so corners bleed speed less aggressively.
    // dragAlpha = max(|beta|*0.6, tempAlpha)
    double dragAlpha = std::fabs(beta_deg) * 0.6;
    if (dragAlpha < tempAlpha) dragAlpha = tempAlpha;
    double cd = cd_(mach, dragAlpha) * table_->cdFactor;

    // Scale for flaps
    cl *= (1.0 + tefFactor * aux_->CLtefFactor);
    cd *= (1.0 + tefFactor * aux_->CDtefFactor + lefFactor * aux_->CDlefFactor);
    if (aero.dragChutePos > 0.5) {
        cd += aux_->dragChuteCd * aero.dragChutePos;
    }

    // --- Local lift-curve slope (3-point finite difference) ---
    const double cl1 = cl_(mach, tempAlpha - 2.0) * table_->clFactor * (1.0 + tefFactor * aux_->CLtefFactor);
    const double cl2 = cl_(mach, tempAlpha + 2.0) * table_->clFactor * (1.0 + tefFactor * aux_->CLtefFactor);
    const double cd1 = cd_(mach, dragAlpha - 2.0) * table_->cdFactor;
    const double cd2 = cd_(mach, dragAlpha + 2.0) * table_->cdFactor;
    double clalpha = (cl2 - cl1) * 0.25 * (1.0 + tefFactor * aux_->CLtefFactor);
    double cnalpha = ((cl2 - cl1) * cosalp + (cd2 - cd1) * sinalp) * 0.25 *
                     (1.0 + tefFactor * aux_->CLtefFactor);

    // --- Static lift-curve slope (alpha = 0..10 deg) ---
    const double cls0 = cl_(mach, 0.0)  * table_->clFactor * (1.0 + tefFactor * aux_->CLtefFactor);
    const double cls1 = cl_(mach, 10.0) * table_->clFactor * (1.0 + tefFactor * aux_->CLtefFactor);
    double clalph0 = (cls1 - cls0) * 0.1 * (1.0 + tefFactor * aux_->CLtefFactor);
    double clift0 = cls0;

    // --- Ground effect ---
    // Within 0.2*span of the ground: CL *= 1.13. Between 0.2 and 1.0 span:
    // fades linearly back to 1.0.
    const double span = geom_->span_ft;
    const double agl_ft = std::fabs(groundZ_ft - z_ft);
    if (agl_ft < span * 0.2) {
        const double g = 1.13;
        cl *= g; clalpha *= g; cnalpha *= g;
    } else if (agl_ft < span) {
        const double f = 1.13 - ((agl_ft - span * 0.2) / (span * 0.8)) * 0.13;
        cl *= f; clalpha *= f; cnalpha *= f;
    }

    // --- Drag additions (speed brake, gear, stores) ---
    cd += aux_->CDSPDBFactor * aero.dbrake;
    cd += aux_->CDLDGFactor  * aero.gearPos;
    cd += aero.cdStores;

    // --- Force summation ---
    const double lift = cl * qsom;
    const double drag = cd * qsom;

    // Body axes (ft/s^2):
    //   xaero = -D cos(a) + L sin(a)
    //   zaero = -L cos(a) - D sin(a)
    //   yaero =  Cy * qsom * (beta - |beta|*yshape*0.5)
    const double yshape = pstick * pstick * (pstick >= 0 ? 1.0 : -1.0);
    const double yaero = cy * qsom * (beta_deg - std::fabs(beta_deg) * yshape * 0.5);

    const double xaero = -drag * cosalp + lift * sinalp;
    const double zaero = -lift * cosalp - drag * sinalp;

    // Stability axes
    const double xsaero = -drag;
    const double ysaero = yaero;
    const double zsaero = -lift;

    // Wind axes
    const double xwaero =  xsaero * cosbet + ysaero * sinbet;
    const double ywaero = -xsaero * sinbet + ysaero * cosbet;
    const double zwaero =  zsaero;

    // --- Stall model ---
    bool stalled = false;
    double stallSpeed = 0.0;
    if (geom_->area_ft2 > 0.0) {
        // V_stall = 17.16 * sqrt((W/S) / |CL|) where W/S in lb/ft^2
        // We approximate weight via qsom at 1 G: W = qsom * mass * |CL| (for level flight, CL = W/(qS))
        // For a stall indicator, use: V_stall(kts) = 17.16 * sqrt((W/S) / CL_max)
        // We need weight; use mass*GRAVITY * (1/loadingFraction) approximation is wrong; just
        // compute from current conditions.
        const double mass_slugs = (qbar > 0.0 && qsom > 0.0) ? (qbar * geom_->area_ft2 / qsom) : 1.0;
        const double weight_lbs = mass_slugs * GRAVITY;
        const double ws = weight_lbs / geom_->area_ft2;
        if (std::fabs(cl) > 1e-3) {
            stallSpeed = 17.16 * std::sqrt(ws / std::fabs(cl));
        }
        if (vcas_kts < stallSpeed && vt_ftps > 1.0) {
            stalled = true;
        }
    }

    // Store results
    aero.cl = cl;
    aero.cd = cd;
    aero.cy = cy;
    aero.clalpha = clalpha;
    aero.clalph0 = clalph0;
    aero.clift0 = clift0;
    aero.cnalpha = cnalpha;
    aero.lift = lift;
    aero.drag = drag;
    aero.xaero = xaero;
    aero.yaero = yaero;
    aero.zaero = zaero;
    aero.xsaero = xsaero;
    aero.ysaero = ysaero;
    aero.zsaero = zsaero;
    aero.xwaero = xwaero;
    aero.ywaero = ywaero;
    aero.zwaero = zwaero;
    aero.stalled = stalled;
    aero.stallSpeed = stallSpeed;
}

} // namespace f4flight
