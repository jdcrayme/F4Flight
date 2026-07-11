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
// Gains - port of gain.cpp
// ---------------------------------------------------------------------------
void FlightControlSystem::computeGains(double qbar, double qsom, double vt,
                                       double alpha_deg, double clift0,
                                       double clalph0, double clalpha,
                                       double cnalpha, double cosgam,
                                       double cosmu, bool landingGains,
                                       double gearPos, FcsState& fcs) const {
    // Stick shaping: quadratic with sign preservation
    auto shape = [](double s) {
        return s * s * (s >= 0.0 ? 1.0 : -1.0);
    };
    fcs.pshape = shape(fcs.pshape); // will be set by caller; placeholder
    (void)fcs;

    // --- Pitch axis ---
    // Time constants (legacy defaults from gain.cpp:235-237)
    double tp01 = 0.200;
    double zp01 = 0.900;

    // Damper reduction at low Q
    const double zpdamp = (qbar < 25.0) ? 0.15 * (1.0 - qbar / 25.0) : 0.0;
    zp01 *= (1.0 - zpdamp);

    // Available G (max load factor at aoamax)
    const double aoamax = geom_->aoaMax_deg;
    const double gsAvail = aoamax * clalpha * qsom / GRAVITY;

    // Pitch-axis closed-loop pole placement
    const double kp03 = 2.0;
    const double ttheta2 = (vt > 1.0) ? vt / (GRAVITY * std::max(0.01, clalph0 * qsom * RTD / GRAVITY)) : 1.0;
    const double omegasp = 1.0 / (ttheta2 * 0.65);
    const double wp01 = omegasp;

    const double pcoef1 = tp01 * wp01 * wp01 - 2.0 * zp01 * wp01 - kp03;
    const double pcoef2 = 2.0 * zp01 * wp01 * kp03 - kp03 * tp01 * wp01 * wp01;
    const double disc = std::sqrt(std::max(0.0, pcoef1 * pcoef1 - 4.0 * pcoef2));
    const double pfreq1 = (disc - pcoef1) * 0.5;
    const double pfreq2 = -pcoef1 - pfreq1;
    const double tp02 = (std::fabs(pfreq1) > 1e-6) ? 1.0 / pfreq1 : 1.0;
    const double tp03 = (std::fabs(pfreq2) > 1e-6) ? 1.0 / pfreq2 : 1.0;

    fcs.tp01 = tp01;
    fcs.tp02 = tp02;
    fcs.tp03 = tp03;
    fcs.zp01 = zp01;
    fcs.kp01 = geom_->maxGs;  // full stick (1.0) commands maxGs G
    fcs.kp02 = 1.0;
    fcs.kp03 = kp03;

    // gsAvail vs maxGs selects AOA-command vs G-command mode
    const bool aoaCmdMode = (gsAvail < geom_->maxGs * 0.99);
    fcs.kp05 = aoaCmdMode
        ? tp02 * tp03 * wp01 * wp01
        : GRAVITY * tp02 * tp03 * wp01 * wp01 / std::max(1e-6, qsom * cnalpha);

    // --- Roll axis ---
    double tr01 = (qbar >= 250.0) ? 0.25 : -0.001111 * (qbar - 100.0) + 0.416;
    if (tr01 < 0.05) tr01 = 0.05;
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
    const double zy01 = 0.70;
    const double wy01 = 0.3 / std::max(0.01, tr01);
    fcs.ky02 = 1.0;
    fcs.ky03 = 2.0;
    const double ycoef1 = -2.0 * zy01 * wy01 - fcs.ky03;
    const double ycoef2 =  2.0 * zy01 * wy01 * fcs.ky03;
    const double ydisc = std::sqrt(std::max(0.0, ycoef1 * ycoef1 - 4.0 * ycoef2));
    const double yfreq1 = (ydisc - ycoef1) * 0.5;
    const double yfreq2 = -ycoef1 - yfreq1;
    const double ty02 = (std::fabs(yfreq2) > 1e-6) ? 1.0 / yfreq2 : 1.0;
    (void)yfreq1; // ty01 not currently exported
    fcs.ty02 = ty02;
    // ky05 has a sign convention: in the legacy code it's negative. We keep
    // the magnitude for use as a coefficient below.
    const double cy_est = 1.0; // Cy derivative not modelled separately; legacy uses qsom*cy
    fcs.ky05 = -GRAVITY * wy01 * wy01 / std::max(1e-6, qsom * cy_est * yfreq1 * yfreq2);

    // Landing-gain scaling
    if (landingGains) {
        fcs.kp05 *= aux_->pitchGearGain;
        fcs.ky05 *= aux_->yawGearGain;
    }
}

// ---------------------------------------------------------------------------
// Pitch FCS - port of pitch.cpp
// ---------------------------------------------------------------------------
void FlightControlSystem::runPitch(double dt, double qbar, double qsom,
                                   double vt, double vcas_kts, double alpha_deg,
                                   double cosmu, double cosgam, double singam,
                                   double nzcgs, double cl, double clalpha,
                                   double cnalpha, double aoamin, double aoamax,
                                   double maxGs, PilotInput const& input,
                                   FcsState& fcs, AeroState& aero) const {
    // Stick shaping
    fcs.pshape = input.pstick * input.pstick * (input.pstick >= 0.0 ? 1.0 : -1.0);

    const bool aoaCmdMode = (cfg_ != nullptr) && cfg_->aoaCommandMode;
    double ptcmd;

    if (aoaCmdMode) {
        // AOA command mode: stick commands alpha directly
        // Compute the alpha that would yield maxGs at current conditions
        double maxgcmd = alpha_deg + (maxGs - cl * qsom / GRAVITY - cosmu * cosgam) *
                                       (GRAVITY / std::max(1e-6, qsom * cnalpha));
        maxgcmd = limit(maxgcmd, aoamin, aoamax);
        if (fcs.pshape >= 0.0) {
            ptcmd = fcs.pshape * maxgcmd;
        } else {
            ptcmd = -fcs.pshape * aoamin;
        }
    } else {
        // G command mode.
        // We use a feed-forward + proportional approach instead of the legacy
        // integral controller (which was prone to integrator windup).
        //
        // The feed-forward alpha for the commanded G is computed from the
        // lift equation: alpha_ff = (targetG * g) / (qsom * clalpha)
        // Then a small proportional correction is added based on the G error.
        ptcmd = fcs.pshape * fcs.kp01;  // commanded G (0..maxGs)
        double maxNegGs = -3.0;
        if (cfg_) {
            const Limiter& lim = cfg_->limiters[static_cast<int>(LimiterKey::NegGLimiter)];
            if (lim.type != LimiterType::Value) {
                maxNegGs = lim.limit(vcas_kts);
            }
        }
        // Clamp the G command
        ptcmd = limit(ptcmd, maxNegGs, maxGs);

        // Feed-forward: alpha needed for the commanded G
        // lift = cl * qsom = targetG * g  =>  cl = targetG * g / qsom
        // alpha = cl / clalpha
        // NOTE: aero.clalpha is per DEGREE (not per radian) in our aero code,
        // so the result is already in degrees.
        double alpha_ff_deg = 0.0;
        if (qsom > 1e-3 && std::fabs(aero.clalpha) > 1e-3) {
            alpha_ff_deg = (ptcmd * GRAVITY) / (qsom * aero.clalpha);
        }

        // Proportional correction on G error
        const double gErr = ptcmd - nzcgs;
        const double alpha_corr_deg = gErr * 0.5;  // 0.5 deg per G of error

        // Total alpha command
        double aoacmd_g = alpha_ff_deg + alpha_corr_deg;
        // Clamp to AOA limits (much tighter than the legacy aoamax for
        // G-command mode — 9° is plenty for 1 G level flight)
        aoacmd_g = limit(aoacmd_g, aoamin, std::min(aoamax, 15.0));

        fcs.aoacmd = aoacmd_g;
        fcs.ptcmd = ptcmd;
        // Apply first-order lag to compute new alpha
        const double tau = fcs.tp01 * aux_->pitchMomentum;
        aero.alpha_deg = fcs.pitchAlphaLag.step(aoacmd_g, tau, dt);
        aero.alpha_dot = (aero.alpha_deg - fcs.oldp03[0]) / std::max(dt, 1e-6);
        fcs.oldp03[0] = aero.alpha_deg;
        return;
    }

    // For AOA command mode, apply a simple lag to alpha
    fcs.ptcmd = ptcmd;
    fcs.aoacmd = ptcmd;
    const double tau = fcs.tp01 * aux_->pitchMomentum;
    aero.alpha_deg = fcs.pitchAlphaLag.step(ptcmd, tau, dt);
    aero.alpha_dot = (aero.alpha_deg - fcs.oldp03[0]) / std::max(dt, 1e-6);
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
                                 double singam, double nzcgs, double nycgw,
                                 bool gearDown, bool refueling,
                                 bool landingGainsActive, PilotInput const& input,
                                 FcsState& fcs, AeroState& aero) const {
    if (!geom_ || !aux_) return;

    const bool landingGains = landingGainsActive || gearDown || refueling;

    // Damper lookups
    fcs.plsdamp = applyLimiter(LimiterKey::PitchYawControlDamper, qbar);
    fcs.rlsdamp = applyLimiter(LimiterKey::RollControlDamper,     qbar);
    fcs.ylsdamp = fcs.plsdamp;

    computeGains(qbar, qsom, vt_ftps, alpha_deg, aero.clift0, aero.clalph0,
                 aero.clalpha, aero.cnalpha, cosgam, cosmu, landingGains,
                 aero.gearPos, fcs);

    runPitch(dt, qbar, qsom, vt_ftps, vcas_kts, alpha_deg, cosmu, cosgam,
             singam, nzcgs, aero.cl, aero.clalpha, aero.cnalpha,
             geom_->aoaMin_deg, geom_->aoaMax_deg, geom_->maxGs,
             input, fcs, aero);

    runRoll(dt, qbar, vcas_kts, alpha_deg, aero.gearPos, input, fcs);

    runYaw(dt, qbar, qsom, vt_ftps, vcas_kts, beta_deg, nycgw,
           geom_->betaMin_deg, geom_->betaMax_deg, input, fcs);
}

} // namespace f4flight
