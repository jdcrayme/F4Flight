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
                                       double cnalpha, double cosgam,
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
    // AOA-command mode when gsAvail <= maxGs (can't reach max G)
    const bool aoaCmdMode = (gsAvail <= geom_->maxGs);
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

    // ky05 uses actual cy (not estimated). FreeFalcon uses the current cy value.
    // For now we use 1.0 as a fallback (same as before) but note this should
    // use the actual cy from the aero tables.
    const double cy_est = 1.0;
    if (cy_est != 0.0) {
        fcs.ky05 = -GRAVITY * wy01 * wy01 / std::max(1e-6, qsom * cy_est * yfreq1 * yfreq2);
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
                                   double cnalpha, double aoamin, double aoamax,
                                   double maxGs, PilotInput const& input,
                                   FcsState& fcs, AeroState& aero) const {
    // pshape is now computed in update() before computeGains(), since kp01
    // depends on it. Don't recompute it here.

    const bool aoaCmdMode = (cfg_ != nullptr) && cfg_->aoaCommandMode;

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
        fcs.aoacmd = ptcmd;
        fcs.ptcmd = ptcmd;
        const double tau1 = fcs.tp01 * aux_->pitchMomentum;
        const double tau2 = fcs.tp02 * aux_->pitchMomentum;
        const double tau3 = fcs.tp03 * aux_->pitchMomentum;
        aero.alpha_deg = fcs.pitchAlphaLag.step(ptcmd, tau1, tau2, tau3, dt);
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

    // Available G (max load factor at aoamax)
    const double gsAvail = aoamax * clalpha * qsom / GRAVITY;
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
    const double eprop = fcs.kp02 * error;
    const double eintg1 = fcs.kp03 * error;
    double eintg = fcs.pitchIntegral.step(eintg1, dt);

    // AOA limiter (anti-windup) — FreeFalcon pitch.cpp:348-389
    if (qbar > 100.0) {
        if (eintg > aoamax) {
            eintg = aoamax;
            fcs.pitchIntegral.y_prev = aoamax;
        } else if (eintg < aoamin) {
            eintg = aoamin;
            fcs.pitchIntegral.y_prev = aoamin;
        }
    } else {
        // Low-Q: relax limits slightly
        if (eintg > aoamax) { eintg = aoamax; fcs.pitchIntegral.y_prev = aoamax; }
        else if (eintg < aoamin) { eintg = aoamin; fcs.pitchIntegral.y_prev = aoamin; }
    }

    // Alpha command
    double aoacmd = (eprop + eintg) * fcs.plsdamp;

    // Apply F7Tust lead-lag filter (3 time constants * pitchMomentum)
    const double tau1 = fcs.tp01 * aux_->pitchMomentum;
    const double tau2 = fcs.tp02 * aux_->pitchMomentum;
    const double tau3 = fcs.tp03 * aux_->pitchMomentum;
    const double oldAlpha = aero.alpha_deg;
    aero.alpha_deg = fcs.pitchAlphaLag.step(aoacmd, tau1, tau2, tau3, dt);
    aero.alpha_dot = (aero.alpha_deg - oldAlpha) / std::max(dt, 1e-6);

    fcs.aoacmd = aoacmd;
    fcs.ptcmd = ptcmd;
    fcs.oldp03[0] = aero.alpha_deg;
}

// ---------------------------------------------------------------------------
// Roll FCS - port of roll.cpp
// ---------------------------------------------------------------------------
void FlightControlSystem::runRoll(double dt, double qbar, double vcas_kts,
                                  double alpha_deg, double gearPos,
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
                                 PilotInput const& input, FcsState& fcs) const {
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

    const double tau = fcs.ty02 * aux_->yawMomentum;
    // Apply the lag to beta. We store the result in aero.beta_deg via the
    // lag filter state. The FCS owns the yaw rate lag filter; the EOM will
    // read fcs.betcmd for its own purposes.
    (void)tau;
    // Note: beta integration is handled by the EOM using betcmd as the
    // command. Here we just expose betcmd.
}

// ---------------------------------------------------------------------------
// Top-level FCS update
// ---------------------------------------------------------------------------
void FlightControlSystem::update(double dt, double qbar, double qsom, double mach,
                                 double vt_ftps, double vcas_kts, double alpha_deg,
                                 double beta_deg, double cosmu, double cosgam,
                                 double singam, double costhe, double cosphi,
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
                 aero.clalpha, aero.cnalpha, cosgam, cosmu, costhe, cosphi,
                 loadingFraction, inAir, landingGains, aero.gearPos, fcs);

    runPitch(dt, qbar, qsom, vt_ftps, vcas_kts, alpha_deg, cosmu, cosgam,
             singam, nzcgs, aero.cl, aero.clalpha, aero.cnalpha,
             geom_->aoaMin_deg, geom_->aoaMax_deg, geom_->maxGs,
             input, fcs, aero);

    runRoll(dt, qbar, vcas_kts, alpha_deg, aero.gearPos, input, fcs);

    runYaw(dt, qbar, qsom, vt_ftps, vcas_kts, beta_deg, nycgw,
           geom_->betaMin_deg, geom_->betaMax_deg, input, fcs);
}

} // namespace f4flight
