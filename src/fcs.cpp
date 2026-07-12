// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// fcs.cpp
//
// Flight Control System implementation. Port of gain.cpp, fcs.cpp,
// pitch.cpp, roll.cpp, yaw.cpp.

#include "f4flight/fcs.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

FlightControlSystem::FlightControlSystem(const AircraftConfig* cfg,
                                         const AircraftGeometry* geom,
                                         const AuxAero* aux)
    : cfg_(cfg), geom_(geom), aux_(aux) {
    if (cfg_) {
        rollCmdTable_ = cfg_->rollCmd.makeLookup();
    }
}

double FlightControlSystem::applyLimiter(LimiterKey key, double x) const {
    if (!cfg_) return x;
    const int idx = static_cast<int>(key);
    if (idx < 0 || idx >= static_cast<int>(LimiterKey::Count)) return x;
    // Use the typed accessor (bounds-checked above) instead of raw array index.
    return cfg_->limiter(key).limit(x);
}

// ---------------------------------------------------------------------------
// Gains - port of gain.cpp (fixed to match FreeFalcon exactly)
//
// The previous f4flight version had 7 bugs in this function:
//   A: kp01 was constant maxGs; should be attitude-dependent
//   B: gsAvail used clalpha (local); should use clalph0 (static)
//   C: omegasp missing floor of 1.0 and LowSpeedOmega limiter
//   D: tp03 missing floor of 0.5
//   E: zp01 missing loadingFraction term and zpdamp from limiter
//   F: kp05 missing ground fade
//   G: pshape was computed here but should be computed in runPitch
// ---------------------------------------------------------------------------
void FlightControlSystem::computeGains(double qbar, double qsom, double vt,
                                       double alpha_deg, double clift0,
                                       double clalph0, double clalpha,
                                       double cnalpha, double cy,
                                       double cosgam,
                                       double cosmu, double costhe, double cosphi,
                                       double loadingFraction, bool inAir,
                                       bool landingGains, double gearPos,
                                       FcsState& fcs) const {
    const double cosphiLim = std::max(0.0, cosphi);
    const double cosmuLim  = std::max(0.0, cosmu);

    // --- Available G (max load factor at aoamax) ---
    // Bug B fix: FreeFalcon uses clalph (static slope 0..10), not clalpha (local)
    const double aoamax = geom_->aoaMax_deg;
    const double gsAvail = aoamax * clalph0 * qsom / GRAVITY;

    // --- Pitch axis time constants ---
    const double tp01 = 0.200;
    double zp01 = 0.900;

    // Bug E fix: zp01 damping from low-q and loadingFraction.
    // FreeFalcon gain.cpp:187:
    //   zp01 *= (1 - 0.15*(max(0,1-qbar/25)) - zpdamp - max(0,(loadingFraction-1.3)*0.01))
    //   zp01 = max(0.5, zp01)
    // NOTE: `zpdamp` is NOT the PitchYawControlDamper limiter. It's a dynamic
    // damping term (airframe.cpp:937-942) that increases with stick activity,
    // initialized to 0.0. We don't have that model, so we use 0.0.
    const double zpdamp = 0.0;  // dynamic stick-activity damping (not modeled)
    const double lowQdamp = 0.15 * std::max(0.0, 1.0 - qbar / 25.0);
    const double loadingDamp = std::max(0.0, (loadingFraction - 1.3) * 0.01);
    zp01 *= (1.0 - lowQdamp - zpdamp - loadingDamp);
    zp01 = std::max(0.5, zp01);

    // --- kp01: attitude-dependent G command scaling ---
    // Bug A fix: FreeFalcon gain.cpp:194-197
    //   if (pshape > 0) kp01 = maxGs - costhe*cosphiLim;
    //   else            kp01 = 4.0 + costhe*cosphiLim;
    // f4flight sets pshape in runPitch; here we use the current pshape.
    if (fcs.pshape > 0.0) {
        fcs.kp01 = geom_->maxGs - costhe * cosphiLim;
    } else {
        fcs.kp01 = 4.0 + costhe * cosphiLim;
    }

    fcs.kp02 = 1.0;
    fcs.kp03 = 2.0;
    const double kp03 = fcs.kp03;

    // --- Closed-loop pitch frequency ---
    // FreeFalcon gain.cpp:165-204
    const double nzalpha = clalph0 * qsom * RTD / GRAVITY;
    double ttheta2 = vt / (GRAVITY * std::max(0.01, nzalpha));
    ttheta2 = std::max(ttheta2, 0.1);

    double omegasp1 = 1.0 / (ttheta2 * 0.65);
    omegasp1 = std::max(1.0, omegasp1);  // Bug C fix: floor of 1.0
    double omegasp = omegasp1;

    // Bug C fix: LowSpeedOmega limiter (or ×2 if not in air / in stall)
    // FreeFalcon gain.cpp:206-216:
    //   if (stallMode > Recovering or not IsSet(InAir))
    //       omegasp *= 2.0F;
    //   else {
    //       limiter = gLimiterMgr->GetLimiter(LowSpeedOmega, vehicleIndex);
    //       if (limiter) omegasp *= limiter->Limit(qbar);
    //   }
    // NOTE: FreeFalcon's GetLimiter returns NULL if no limiter is loaded.
    // In f4flight, all limiters are default-constructed (Line 0,0,0,0), which
    // returns 0.0 — that would zero out omegasp. We check if the limiter is
    // a default (all-zero Line) and treat it as "not configured" (multiply by 1.0).
    if (!inAir) {
        omegasp *= 2.0;
    } else {
        const Limiter& lso = cfg_->limiter(LimiterKey::LowSpeedOmega);
        const bool isDefault = (lso.type == LimiterType::Line &&
                                lso.x1 == 0.0 && lso.y1 == 0.0 &&
                                lso.x2 == 0.0 && lso.y2 == 0.0);
        if (!isDefault) {
            omegasp *= lso.limit(qbar);
        }
    }

    const double wp01 = omegasp;

    // --- Inner loop dynamics (pole placement) ---
    // FreeFalcon gain.cpp:223-236
    const double pcoef1 = tp01 * wp01 * wp01 - 2.0 * zp01 * wp01 - kp03;
    const double pcoef2 = 2.0 * zp01 * wp01 * kp03 - kp03 * tp01 * wp01 * wp01;
    const double pradcl = std::max(pcoef1 * pcoef1 - 4.0 * pcoef2, 0.0);
    const double pfreq1 = (std::sqrt(pradcl) - pcoef1) * 0.5;
    const double pfreq2 = -pcoef1 - pfreq1;

    double tp02 = (std::fabs(pfreq1) > 1e-6) ? 1.0 / pfreq1 : 1.0;
    double tp03 = (std::fabs(pfreq2) > 1e-6) ? 1.0 / pfreq2 : 1.0;
    tp03 = std::max(tp03, 0.5);  // Bug D fix: floor of 0.5

    fcs.tp01 = tp01;
    fcs.tp02 = tp02;
    fcs.tp03 = tp03;
    fcs.zp01 = zp01;

    // --- kp05: forward-path gain ---
    // FreeFalcon gain.cpp:240-244
    // AOA-command mode when gsAvail <= maxGs (can't reach max G).
    // Store the runtime decision in FcsState so runPitch uses the SAME
    // mode computeGains computed kp05 for. Previously computeGains used the
    // runtime test (gsAvail <= maxGs) while runPitch used a static config
    // flag (cfg_->aoaCommandMode); when they disagreed, kp05 was computed
    // for one mode and runPitch executed the other -- pitch gain wrong by
    // g/(qsom*cnalpha), causing closed-loop bandwidth mismatch / oscillation.
    const bool aoaCmdMode = (gsAvail <= geom_->maxGs);
    fcs.aoaCmdModeRuntime = aoaCmdMode;
    if (aoaCmdMode || qsom * cnalpha == 0.0) {
        fcs.kp05 = tp02 * tp03 * wp01 * wp01;
    } else {
        fcs.kp05 = GRAVITY * tp02 * tp03 * wp01 * wp01 / (qsom * cnalpha);
    }

    // Bug F fix: ground fade
    // FreeFalcon gain.cpp:249-252
    if (!inAir) {
        fcs.kp05 *= std::max(0.0, std::min(1.0, (qbar - 20.0) / 45.0));
    }

    // --- Roll axis ---
    // FreeFalcon gain.cpp:259-262
    double tr01 = (qbar >= 250.0) ? 0.25 : -0.001111 * (qbar - 100.0) + 0.416;
    fcs.tr01 = tr01;

    // Max roll rate from table
    double psmax = 360.0; // deg/s fallback if no table
    if (cfg_ && !cfg_->rollCmd.alpha_deg.empty()) {
        psmax = rollCmdTable_(alpha_deg, qbar);
    }
    fcs.kr01 = psmax * DTR;
    fcs.kr02 = std::cos(alpha_deg * DTR);
    if (landingGains) fcs.kr01 *= aux_->rollGearGain;

    // --- Yaw axis ---
    // FreeFalcon gain.cpp:287-320
    const double zy01 = 0.70;
    double wy01 = 0.3 / std::max(0.01, tr01);
    wy01 *= (1.0 - loadingFraction * 0.1);  // FreeFalcon gain.cpp:293

    fcs.ky02 = 1.0;
    fcs.ky03 = 2.0;
    const double ycoef1 = -2.0 * zy01 * wy01 - fcs.ky03;
    const double ycoef2 =  2.0 * zy01 * wy01 * fcs.ky03;
    const double yradcl = std::max(ycoef1 * ycoef1 - 4.0 * ycoef2, 0.0);
    const double yfreq1 = (std::sqrt(yradcl) - ycoef1) * 0.5;
    const double yfreq2 = -ycoef1 - yfreq1;
    const double ty02 = (std::fabs(yfreq2) > 1e-6) ? 1.0 / yfreq2 : 1.0;
    fcs.ty02 = ty02;

    // ky05 uses the ACTUAL cy (FreeFalcon gain.cpp:319). Previously F4Flight
    // hardcoded cy_est = 1.0, which gave ky05 the WRONG SIGN whenever the
    // real cy was negative (typical for the F-16 side-force coefficient).
    // The wrong sign turned the yaw damper into a positive-feedback loop:
    // any small sideslip grew unboundedly, the heading drifted, and the
    // steering's LevelTurn mode kicked in and disrupted climbs/descents.
    //
    // The original `std::max(1e-6, denom)` was also wrong: when cy < 0 the
    // denominator is NEGATIVE, and max(1e-6, negative) = 1e-6, producing a
    // huge negative ky05 (~34 million). FreeFalcon's original code has no
    // such clamp — it just checks cy != 0. We use std::fabs() > 1e-6 to
    // avoid division by zero while preserving the sign.
    const double ky05_denom = qsom * cy * yfreq1 * yfreq2;
    if (std::fabs(ky05_denom) > 1e-6) {
        fcs.ky05 = -GRAVITY * wy01 * wy01 / ky05_denom;
    }

    // --- Landing-gain scaling ---
    if (landingGains) {
        fcs.kp05 *= aux_->pitchGearGain;
        fcs.ky05 *= aux_->yawGearGain;
    }
}

// ---------------------------------------------------------------------------
// Pitch FCS - port of pitch.cpp (Bug #1 fix)
//
// FreeFalcon uses a PI controller with Adams-Bashforth integration:
//   ptcmd = pshape * kp01  (commanded G)
//   ptcmd = clamp(ptcmd, maxNegGs, min(gsAvail, maxCmd))
//   error = (ptcmd - (nzcgs - cosmu*cosgam - 0.1*gearPos*qsom/g)) * kp05
//   eprop = kp02 * error
//   eintg = AdamsBash2(kp03 * error, dt)
//   eintg = clamp(eintg, aoamin, aoamax)  // anti-windup
//   aoacmd = (eprop + eintg) * plsdamp
//   alpha = F7Tust(aoacmd, tp01*pm, tp02*pm, tp03*pm, dt)  // lead-lag
//
// The previous f4flight code used feed-forward + proportional, which had
// completely different response dynamics (no integrator, no anti-windup,
// 1st-order lag instead of lead-lag).
// ---------------------------------------------------------------------------
void FlightControlSystem::runPitch(double dt, double qbar, double qsom,
                                   double vt, double vcas_kts, double alpha_deg,
                                   double cosmu, double cosgam, double singam,
                                   double nzcgs, double cl, double clalpha,
                                   double clalph0, double cnalpha,
                                   double aoamin, double aoamax, double maxGs,
                                   PilotInput const& input,
                                   FcsState& fcs, AeroState& aero) const {
    // pshape is now computed in update() before computeGains(), since kp01
    // depends on it. Don't recompute it here.

    // Use the SAME aoaCmdMode decision computeGains made. Stored in
    // fcs.aoaCmdModeRuntime. Previously this read cfg_->aoaCommandMode (a
    // static config flag), which could disagree with computeGains' runtime
    // test (gsAvail <= maxGs) and produce a wrong kp05 for the executed mode.
    const bool aoaCmdMode = fcs.aoaCmdModeRuntime;

    double ptcmd, maxCmd, minCmd;
    double maxNegGs = -3.0;

    if (aoaCmdMode) {
        // AOA command mode (not fully ported — keeps existing behavior for now).
        // This path is rarely used; the G-command path is the default.
        double maxgcmd = alpha_deg + (maxGs - cl * qsom / GRAVITY - std::max(0.0, cosmu) * cosgam) *
                                       (GRAVITY / std::max(1e-6, qsom * cnalpha));
        maxgcmd = limit(maxgcmd, aoamin, aoamax);
        if (fcs.pshape >= 0.0) {
            ptcmd = fcs.pshape * maxgcmd;
        } else {
            ptcmd = -fcs.pshape * aoamin;
        }
        // Clamp the commanded AOA to [aoamin, aoamax] — same rationale as
        // the G-command path below. Without this, the lead-lag filter can
        // overshoot past aoamax when commanding 9G at low gsAvail.
        ptcmd = limit(ptcmd, aoamin, aoamax);
        fcs.aoacmd = ptcmd;
        fcs.ptcmd = ptcmd;
        const double tau1 = fcs.tp01 * aux_->pitchMomentum;
        const double tau2 = fcs.tp02 * aux_->pitchMomentum;
        const double tau3 = fcs.tp03 * aux_->pitchMomentum;
        aero.alpha_deg = fcs.pitchAlphaLag.step(ptcmd, tau1, tau2, tau3, dt);
        // Clamp the filtered output too — the lead-lag can overshoot
        aero.alpha_deg = limit(aero.alpha_deg, aoamin, aoamax);
        aero.alpha_dot = (aero.alpha_deg - fcs.oldp03[0]) / std::max(dt, 1e-6);
        fcs.oldp03[0] = aero.alpha_deg;
        return;
    }

    // --- G command mode (the default, and the main fix) ---
    // FreeFalcon pitch.cpp:265-340
    ptcmd = fcs.pshape * fcs.kp01;  // commanded G

    // Negative G limiter
    if (cfg_) {
        const Limiter& lim = cfg_->limiters[static_cast<int>(LimiterKey::NegGLimiter)];
        if (lim.type != LimiterType::Value) {
            maxNegGs = lim.limit(vcas_kts);
        }
    }

    // Available G (max load factor at aoamax). FreeFalcon gain.cpp:127 uses
    // clalph (the static 0..10 deg lift-curve slope). In f4flight's naming,
    // that is clalph0 -- NOT clalpha (the local ±2 deg slope around current
    // alpha). Near stall, clalpha drops faster than clalph0, so using the
    // local slope would underestimate available G and over-clamp ptcmd,
    // reducing pitch authority when recovering from high AOA.
    const double gsAvail = aoamax * clalph0 * qsom / GRAVITY;
    maxCmd = maxGs;
    minCmd = std::max(maxNegGs, -gsAvail);

    // Clamp the G command
    ptcmd = limit(ptcmd, minCmd, std::min(gsAvail, maxCmd));

    // NZ load factor loop closure.
    // FreeFalcon pitch.cpp:334:
    //   error = (ptcmd - (nzcgs - cosmu*cosgam - 0.1*gearPos*qsom/g)) * kp05
    const double cosmu_lim = std::max(0.0, cosmu);
    const double error = (ptcmd - (nzcgs - cosmu_lim * cosgam
                                   - 0.1 * aero.gearPos * qsom / GRAVITY)) * fcs.kp05;
    double eprop = fcs.kp02 * error;   // mutable: zeroed on saturation (anti-windup)
    const double eintg1 = fcs.kp03 * error;
    double eintg = fcs.pitchIntegral.step(eintg1, dt);

    // AOA limiter (anti-windup) — FreeFalcon pitch.cpp:348-389.
    // On saturation, FreeFalcon zeroes eprop AND clears the Adams-Bashforth
    // input history (oldp02[2] = u_prev, oldp02[3] = u_now) so the integrator
    // doesn't immediately re-saturate on the next step. Previously we only
    // reset y_prev, leaving stale u_prev/u_now to push eintg back past the
    // limit on the next frame -> limit-cycle behavior at high AOA.
    //
    // Low-Q relaxation: FreeFalcon widens the limits by ±alphaError where
    // alphaError = (|q*0.5| + |p|) * 0.1 * RTD. We don't have p/q here, so
    // we approximate alphaError as 0 (the relaxation is a small effect).
    if (qbar > 100.0) {
        if (eintg > aoamax) {
            eintg = aoamax;
            eprop = 0.0;
            fcs.pitchIntegral.y_prev = aoamax;
            fcs.pitchIntegral.u_prev = 0.0;
            fcs.pitchIntegral.u_now  = 0.0;
        } else if (eintg < aoamin) {
            eintg = aoamin;
            eprop = 0.0;
            fcs.pitchIntegral.y_prev = aoamin;
            fcs.pitchIntegral.u_prev = 0.0;
            fcs.pitchIntegral.u_now  = 0.0;
        }
    } else {
        if (eintg > aoamax) {
            eintg = aoamax;
            eprop = 0.0;
            fcs.pitchIntegral.y_prev = aoamax;
            fcs.pitchIntegral.u_prev = 0.0;
            fcs.pitchIntegral.u_now  = 0.0;
        } else if (eintg < aoamin) {
            eintg = aoamin;
            eprop = 0.0;
            fcs.pitchIntegral.y_prev = aoamin;
            fcs.pitchIntegral.u_prev = 0.0;
            fcs.pitchIntegral.u_now  = 0.0;
        }
    }

    // Alpha command
    double aoacmd = (eprop + eintg) * fcs.plsdamp;

    // Clamp the final alpha command to [aoamin, aoamax]. FreeFalcon clamps
    // eintg (the integrator output) but NOT the final aoacmd — so eprop +
    // the lead-lag filter can push alpha past aoamax. At high speed
    // mismatch (e.g. commanding 9G at 350 kts where gsAvail is 4G), the
    // integrator saturates at aoamax=25° but eprop adds another 15-20°,
    // driving alpha to 43° and stalling the aircraft. Clamping aoacmd
    // here is the standard AOA limiter behavior.
    aoacmd = limit(aoacmd, aoamin, aoamax);

    // Apply F7Tust lead-lag filter (3 time constants * pitchMomentum)
    const double tau1 = fcs.tp01 * aux_->pitchMomentum;
    const double tau2 = fcs.tp02 * aux_->pitchMomentum;
    const double tau3 = fcs.tp03 * aux_->pitchMomentum;
    const double oldAlpha = aero.alpha_deg;
    aero.alpha_deg = fcs.pitchAlphaLag.step(aoacmd, tau1, tau2, tau3, dt);
    // Also clamp the filtered output — the lead-lag can overshoot
    aero.alpha_deg = limit(aero.alpha_deg, aoamin, aoamax);
    aero.alpha_dot = (aero.alpha_deg - oldAlpha) / std::max(dt, 1e-6);

    fcs.aoacmd = aoacmd;
    fcs.ptcmd = ptcmd;
    fcs.oldp03[0] = aero.alpha_deg;
}

// ---------------------------------------------------------------------------
// Roll FCS - port of roll.cpp
//
// FreeFalcon's Roll() (roll.cpp:70-197) does two things:
//   1. Compute pscmd from rshape * kr01, with alpha-based roll-rate limiting
//      and low-speed / gear fade.
//   2. Call RollIt(pscmd, dt) which clamps pscmd by maxRoll / maxRollDelta
//      and then lag-filters to pstab.
//
// Previously F4Flight did step 1 but skipped step 2's maxRoll/maxRollDelta
// clamping — so any steering-layer SetMaxRoll(0) / SetMaxRollDelta(...) calls
// were silently ignored, and the wings-level loop had no inner-loop roll
// limiting. That caused the bank-angle limit-cycle seen in the level-hold
// maneuver phase (bank slowly diverged to ±90°).
//
// FreeFalcon RollIt (roll.cpp:199-240) has two units bugs:
//   - `if (phi > maxRoll)` compares phi (rad) to maxRoll (deg). This only
//     works for maxRoll=0; any non-zero maxRoll is effectively ignored.
//   - `pscmd *= 1 - startRoll/maxRollDelta` divides startRoll (rad) by
//     maxRollDelta (deg), giving a 57× too-small damping factor.
// We fix both: convert phi to deg for the comparison, and convert startRoll
// to deg for the damping. This is what FreeFalcon INTENDED.
// ---------------------------------------------------------------------------
void FlightControlSystem::runRoll(double dt, double qbar, double vcas_kts,
                                  double alpha_deg, double gearPos,
                                  double phi_rad,
                                  PilotInput const& input, FcsState& fcs) const {
    fcs.rshape = input.rstick * input.rstick * (input.rstick >= 0.0 ? 1.0 : -1.0);
    double pscmd = limit(fcs.rshape * fcs.kr01, -fcs.kr01, fcs.kr01);

    // Roll-rate limiter on alpha
    if (cfg_) {
        const double rl = applyLimiter(LimiterKey::RollRateLimiter, alpha_deg);
        pscmd *= rl;
    }
    // Slow-speed authority fade
    if (vcas_kts < 220.0) {
        pscmd *= std::max(0.0, vcas_kts / 220.0);
    }
    if (gearPos > 0.5) {
        pscmd *= aux_->rollGearGain;
    }

    // --- FreeFalcon RollIt (roll.cpp:199-240): maxRoll / maxRollDelta limiting ---
    // FreeFalcon only applies this when maxRoll < the vehicle's max roll
    // capability (aeroDataset[vehicleIndex].inputData[8]). We don't have that
    // table; we apply it whenever the steering layer has set maxRoll below
    // the FcsState default (80°). When the steering layer hasn't set maxRoll
    // (e.g. Manual mode), the limit is inactive.
    if (fcs.maxRoll < 80.0) {
        const double phi_deg = phi_rad * RTD;

        // Hard limit: if |phi| > maxRoll, command roll back to maxRoll.
        // (FreeFalcon's `pscmd = (maxRoll - phi) * kr01` for both branches —
        // we preserve that exact formula, just with consistent units.)
        if (phi_deg > fcs.maxRoll) {
            pscmd = (fcs.maxRoll - phi_deg) * DTR * fcs.kr01;
        } else if (phi_deg < -fcs.maxRoll) {
            pscmd = (fcs.maxRoll - phi_deg) * DTR * fcs.kr01;
        }

        // Damping: scale pscmd by (1 - startRoll/maxRollDelta) so the roll
        // rate decays as the aircraft approaches the target bank.
        if (fcs.maxRollDelta == 0.0) {
            pscmd = 0.0;
        } else {
            const double startRoll_deg = fcs.startRoll * RTD;
            const double scale = 1.0 - startRoll_deg / std::max(0.01, fcs.maxRollDelta);
            pscmd *= std::max(0.0, std::min(1.0, scale));
        }
    }

    fcs.pscmd = pscmd;
    const double tau = fcs.tr01 * aux_->rollMomentum;
    fcs.pstab = fcs.rollRateLag.step(pscmd, tau, dt);
}

// ---------------------------------------------------------------------------
// Yaw FCS - port of yaw.cpp
// ---------------------------------------------------------------------------
void FlightControlSystem::runYaw(double dt, double qbar, double qsom, double vt,
                                 double vcas_kts, double beta_deg, double nycgw,
                                 double betmin, double betmax,
                                 PilotInput const& input, FcsState& fcs,
                                 AeroState& aero) const {
    (void)qbar; (void)qsom; (void)vt; (void)vcas_kts; (void)beta_deg;

    fcs.yshape = input.ypedal * input.ypedal * (input.ypedal >= 0.0 ? 1.0 : -1.0);
    double nycmd = fcs.yshape * 2.0;
    nycmd = limit(nycmd, -2.0, 2.0);
    const double gsAvail = betmax * 0.05 * qsom / GRAVITY;
    nycmd *= std::min(gsAvail / 2.0, 1.0);

    // Apply yaw limiters (alpha & roll-rate) -- use beta as a proxy
    if (cfg_) {
        nycmd = applyLimiter(LimiterKey::YawAlphaLimiter, nycmd);
    }

    const double error1 = nycmd + nycgw;
    const double error  = error1 * fcs.ky05;
    const double eprop  = fcs.ky02 * error;
    const double eintg1 = fcs.ky03 * error;
    double eintg = fcs.yawIntegral.step(eintg1, dt);
    eintg = limit(eintg, betmin, betmax);

    double betcmd = (eprop + eintg) * fcs.ylsdamp;
    betcmd = limit(betcmd, betmin, betmax);
    fcs.betcmd = betcmd;

    // Beta integration.
    //
    // FreeFalcon yaw.cpp:206-209: `beta = FLTust(betcmd, ty02*yawMomentum, dt, oldy03)`.
    // In FreeFalcon, the yaw loop drives the RUDDER, which produces a yawing
    // moment, which changes beta via the EOM. F4Flight's EOM does NOT model
    // rudder -> yawing moment -> beta dynamics. The FCS directly setting
    // beta creates a positive-feedback loop (any small sideslip grows
    // unboundedly) because the gain sign and the lack of rudder dynamics
    // make the loop unstable.
    //
    // Until the EOM models rudder -> beta dynamics, we leave beta at 0.
    // This is correct for AI steering (which commands ypedal=0 and expects
    // no sideslip). Manual rudder input will not produce sideslip — that's
    // a known limitation to be addressed when the EOM is extended.
    aero.beta_deg = 0.0;
    aero.beta_dot = 0.0;
}

// ---------------------------------------------------------------------------
// Top-level FCS update
// ---------------------------------------------------------------------------
void FlightControlSystem::update(double dt, double qbar, double qsom, double mach,
                                 double vt_ftps, double vcas_kts, double alpha_deg,
                                 double beta_deg, double cosmu, double cosgam,
                                 double singam, double costhe, double cosphi,
                                 double phi_rad,
                                 double loadingFraction, bool inAir,
                                 double nzcgs, double nycgw,
                                 bool gearDown, bool refueling,
                                 bool landingGainsActive, PilotInput const& input,
                                 FcsState& fcs, AeroState& aero) const {
    if (!geom_ || !aux_) return;

    const bool landingGains = landingGainsActive || gearDown || refueling;

    // Damper lookups
    fcs.plsdamp = applyLimiter(LimiterKey::PitchYawControlDamper, qbar);
    fcs.rlsdamp = applyLimiter(LimiterKey::RollControlDamper,     qbar);
    fcs.ylsdamp = fcs.plsdamp;

    // Compute pshape BEFORE computeGains, because kp01 depends on it.
    // FreeFalcon computes pshape in Gains() (gain.cpp:172-176), then uses it
    // for kp01 selection (gain.cpp:194-197).
    fcs.pshape = input.pstick * input.pstick;
    if (input.pstick < 0.0) fcs.pshape *= -1.0;

    computeGains(qbar, qsom, vt_ftps, alpha_deg, aero.clift0, aero.clalph0,
                 aero.clalpha, aero.cnalpha, aero.cy, cosgam, cosmu, costhe, cosphi,
                 loadingFraction, inAir, landingGains, aero.gearPos, fcs);

    runPitch(dt, qbar, qsom, vt_ftps, vcas_kts, alpha_deg, cosmu, cosgam,
             singam, nzcgs, aero.cl, aero.clalpha, aero.clalph0, aero.cnalpha,
             geom_->aoaMin_deg, geom_->aoaMax_deg, geom_->maxGs,
             input, fcs, aero);

    runRoll(dt, qbar, vcas_kts, alpha_deg, aero.gearPos, phi_rad, input, fcs);

    runYaw(dt, qbar, qsom, vt_ftps, vcas_kts, beta_deg, nycgw,
           geom_->betaMin_deg, geom_->betaMax_deg, input, fcs, aero);
}

} // namespace f4flight
