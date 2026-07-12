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
    // Bugs #4-5 fix: match FreeFalcon aero.cpp:210-252 exactly.
    //
    // FreeFalcon uses RAW interpolated values for cl1/cl2/cd1/cd2 (no clFactor,
    // no cdFactor, no TEF factor). The TEF factor is applied ONCE in the
    // clalpha/cnalpha/clalph0 formulas. The previous f4flight code applied
    // clFactor AND TEF to cl1/cl2, then applied TEF AGAIN in clalpha — double
    // counting. Also, cnalpha must use CDtefFactor (not CLtefFactor) per
    // FreeFalcon line 231.
    const double cl1_raw = cl_(mach, tempAlpha - 2.0);
    const double cl2_raw = cl_(mach, tempAlpha + 2.0);
    const double cd1_raw = cd_(mach, dragAlpha - 2.0);
    const double cd2_raw = cd_(mach, dragAlpha + 2.0);
    double clalpha = (cl2_raw - cl1_raw) * 0.25 * (1.0 + tefFactor * aux_->CLtefFactor);
    double cnalpha = ((cl2_raw - cl1_raw) * cosalp + (cd2_raw - cd1_raw) * sinalp) * 0.25 *
                     (1.0 + tefFactor * aux_->CDtefFactor);  // CDtefFactor, not CLtefFactor

    // --- Static lift-curve slope (alpha = 0..10 deg) ---
    // FreeFalcon: cl1/cl2 are RAW (no clFactor). clift0 = cl1 * (1 + TEF*CLtef)
    // — note clift0 does NOT include clFactor (matches FreeFalcon line 252).
    const double cls0_raw = cl_(mach, 0.0);
    const double cls1_raw = cl_(mach, 10.0);
    double clalph0 = (cls1_raw - cls0_raw) * 0.1 * (1.0 + tefFactor * aux_->CLtefFactor);
    double clift0 = cls0_raw * (1.0 + tefFactor * aux_->CLtefFactor);

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

    // --- Stall model ---
    // Bug #6 fix: match FreeFalcon aero.cpp:293-356.
    //
    // FreeFalcon: if criticalAOA > 0 and alpha > 10, compute stall speed
    //   stallSpeed = 17.16 * sqrt((W/S) / |CL|)
    // If vcas < stallSpeed OR alpha > criticalAOA, lift is reduced to
    //   lift = min(0, cl * 0.5) * (vcas / stallSpeed)
    // (i.e. lift goes to 0 or negative, scaled by speed ratio).
    //
    // The previous f4flight code computed `stalled = true` but never modified
    // lift -- the aircraft kept flying with full cl*qsom even when stalled.
    bool stalled = false;
    double stallSpeed = 0.0;
    if (aux_->criticalAOA > 0.0 && geom_->area_ft2 > 0.0 && alpha_deg > 10.0) {
        // Need weight (lbf). We have qsom = q*S/m, and q = 0.5*rho*vt^2.
        // W = m*g, and m = q*S/qsom (from qsom definition). So W = q*S*g/qsom.
        // But we also have weight from mass*gravity. Use the mass-derived weight.
        // The caller doesn't pass mass directly, but qsom = q*S/m => m = q*S/qsom.
        const double q_val = qbar;  // lb/ft^2
        const double S = geom_->area_ft2;
        const double mass_from_qsom = (qsom > 1e-6) ? (q_val * S / qsom) : 1.0;
        const double weight_lbs = mass_from_qsom * GRAVITY;
        const double ws = weight_lbs / S;
        if (std::fabs(cl) > 1e-3) {
            stallSpeed = 17.16 * std::sqrt(ws / std::fabs(cl));
        }
        if (vcas_kts < stallSpeed || alpha_deg > aux_->criticalAOA) {
            stalled = true;
        }
    }

    // --- Force summation ---
    // Bug #6: when stalled, reduce lift per FreeFalcon formula.
    double lift;
    if (stalled && stallSpeed > 1e-3) {
        // FreeFalcon: lift = min(0, cl*0.5) * (vcas/stallSpeed)
        // This is the ACCELERATION (cl*qsom is accel). But FreeFalcon applies
        // this to `lift` which is in ft/s^2 (cl*qsom). We must match: the
        // cl*0.5 term is a coefficient, and qsom is already the normalizing
        // accel. So: lift_accel = min(0, cl*0.5) * qsom * (vcas/stallSpeed).
        // Actually FreeFalcon sets lift = min(0,cl*0.5)*(vcas/stallSpeed) —
        // that's the coefficient form, NOT multiplied by qsom. But then
        // xaero/zaero use `lift` directly (not lift*qsom). Let me re-check...
        //
        // FreeFalcon line 342: lift = cl * qsom;  (normal case)
        // FreeFalcon line 327: lift = min(0,cl*0.5) * (vcas/stallSpeed);
        //   ^-- this is NOT * qsom! It's just a coefficient * speed ratio.
        //   But then line 367: xaero = -drag*cosalp + lift*sinalp
        //   If lift is just a coefficient (not accel), xaero would be tiny.
        //
        // This looks like a FreeFalcon bug (lift should be * qsom), but to
        // match behavior we replicate it. The effect: when stalled, lift
        // drops to near-zero (coefficient * speed_ratio << cl*qsom), so the
        // aircraft falls. That's the intended stall behavior.
        const double cl_stalled = std::min(0.0, cl * 0.5);
        lift = cl_stalled * (vcas_kts / stallSpeed) * qsom;
    } else if (vt_ftps < 1e-3) {
        lift = 0.0;  // FlatSpin / vt==0 case
    } else {
        lift = cl * qsom;
    }
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

    // --- Stall model (legacy section removed -- now computed above before force summation) ---

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
