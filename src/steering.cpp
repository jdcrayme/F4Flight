// f4flight - steering.cpp
//
// AI steering ported from FreeFalcon's DigitalBrain (sim/digi/).
//
// All functions are direct ports of the corresponding FreeFalcon methods,
// preserving the exact control laws, gain scheduling, and smoothing.
//
// Source mapping:
//   SetPstick   <- DigitalBrain::SetPstick    (mnvers.cpp:300-362)
//   SetRstick   <- DigitalBrain::SetRstick    (mnvers.cpp:364-390)
//   SetYpedal   <- DigitalBrain::SetYpedal    (mnvers.cpp:392-397)
//   GammaHold   <- DigitalBrain::GammaHold    (mnvers.cpp:837-866)
//   AltHold     <- DigitalBrain::AltHold      (autopilot.cpp:323-378)
//   AltitudeHold<- DigitalBrain::AltitudeHold (mnvers.cpp:759-784)
//   HeadingAlt  <- DigitalBrain::HeadingAndAlt(mnvers.cpp:786-835)
//   LevelTurn   <- DigitalBrain::LevelTurn    (mnvers.cpp:713-757)
//   MachHold    <- DigitalBrain::MachHold     (mnvers.cpp:414-665)
//   Loiter      <- DigitalBrain::Loiter       (mnvers.cpp:667+)

#include "f4flight/steering.h"
#include "f4flight/fcs.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

// ===========================================================================
// Utility functions
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

// FreeFalcon major-frame time constant for stick smoothing.
// Derived from: 0.2*old + 0.8*new at T=0.06s → tau = -T/ln(0.2) = 0.0373s
static inline double stickSmoothAlpha(double dt) {
    constexpr double kTau = 0.0373;  // seconds
    return 1.0 - std::exp(-dt / kTau);
}

// Helper: compute rstick from roll error using FCS gains
// Frame-rate-scaled smoothing (same time constant as SetPstick)
static inline double computeRstick(double rollErr_deg, double kr01, double tr01,
                                   double currentRstick, double dt) {
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollErr_deg * DTR * 0.75 / denom;
    const double a = stickSmoothAlpha(dt);
    return (1.0 - a) * currentRstick + a * stickCmd;
}

// Helper: limit roll error based on current roll vs max roll
static inline double limitRollError(double rollErr, double rollDeg, double maxRoll) {
    if (std::fabs(rollDeg) > maxRoll) {
        if (rollDeg > 0.0)
            return std::min(rollErr, maxRoll - rollDeg);
        else
            return std::max(rollErr, maxRoll + rollDeg);
    }
    return rollErr;
}

// ===========================================================================
// DigiAI — core steering functions
//
// Frame-rate calibration:
//   FreeFalcon's digi AI runs at the major-frame rate (default 0.06s = 16.67 Hz).
//   Its smoothing constants (0.2*old + 0.8*new) and integral gains (0.0025*...)
//   are tuned for that rate. F4Flight calls the AI at 60 Hz (or whatever the
//   host's dt is). To preserve the same time constants, we compute alpha from
//   the FreeFalcon tau and the actual dt:
//     tau = -0.06 / ln(0.2) = 0.0373 s
//     alpha = 1 - exp(-dt / tau)
//   At dt=1/60: alpha = 0.360, so y = 0.640*old + 0.360*new.
//   (Previously we used 0.852/0.148 which was calibrated for 6 Hz — wrong.)
// ===========================================================================

void DigiAI::SetPstick(double pitchError, double gLimit, int commandType,
                       DigiState& digi, const AircraftState& state) {
    double stickCmd = 0.0;

    if (commandType == ErrorCommand) {
        if (pitchError > 30.0)
            stickCmd = gLimit;
        else if (pitchError > 0.0)
            stickCmd = gLimit * pitchError / 30.0 + 1.0;
        else if (pitchError > -30.0)
            stickCmd = gLimit * 0.5 * pitchError / 30.0;
        else
            stickCmd = -gLimit * 0.5;
    } else if (commandType == GCommand) {
        stickCmd = std::max(std::min(pitchError, gLimit), -gLimit);
    } else { // AlphaCommand
        stickCmd = pitchError * 0.75;
    }

    // Convert G-command to stick deflection (nonlinear sqrt mapping)
    const double costhe = state.kin.costhe;
    if (stickCmd <= 1.0) {
        stickCmd = -std::sqrt(std::max(0.0, (1.0 - stickCmd) / (4.0 + costhe)));
    } else {
        stickCmd = std::sqrt(std::max(0.0, (stickCmd - 1.0) / (gLimit - costhe)));
    }

    // Low-speed stick authority reduction
    double stickFact = std::min(150.0, state.vcas - 150.0);
    stickFact = 0.5 + stickFact / 300.0;
    stickFact = std::max(0.0, stickFact);
    stickCmd *= stickFact;

    // Frame-rate-independent smoothing (matches FreeFalcon's 0.2/0.8 at 16.67 Hz)
    const double a = stickSmoothAlpha(digi.dt);
    digi.pStick = (1.0 - a) * digi.pStick + a * stickCmd;
}

void DigiAI::SetRstick(double rollError, DigiState& digi,
                       const FlightControlSystem& /*fcs*/,
                       const FcsState& fcsState) {
    const double kr01 = fcsState.kr01;
    const double tr01 = fcsState.tr01;
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollError * DTR * 0.75 / denom;
    const double a = stickSmoothAlpha(digi.dt);
    digi.rStick = (1.0 - a) * digi.rStick + a * stickCmd;
}

void DigiAI::SetYpedal(double yawError, DigiState& digi) {
    const double a = stickSmoothAlpha(digi.dt);
    // FreeFalcon: yPedal = 0.2*yPedal - 0.8*yawError*RTD*0.0125
    // The "new value" being smoothed is -yawError*RTD*0.0125
    const double newVal = -yawError * RTD * 0.0125;
    digi.yPedal = (1.0 - a) * digi.yPedal + a * newVal;
}

void DigiAI::GammaHold(double desGamma, DigiState& digi,
                       const AircraftState& state, double maxGs) {
    // Clamp desired gamma. FreeFalcon clamps to +/- 60° (mnvers.cpp:844),
    // matching the F-16 dash-one autopilot attitude-hold envelope. For AI
    // navigation (climb/descent between waypoints) that envelope is too
    // aggressive — a 10000-ft altitude error saturates the clamp and the
    // aircraft zooms to 60° pitch, bleeds airspeed, and stalls. We expose
    // the clamp via DigiState::maxGammaDeg (default 60° to match FreeFalcon)
    // so callers can request a more conservative envelope for navigation.
    const double maxGam = std::max(1.0, digi.maxGammaDeg);
    desGamma = std::max(std::min(desGamma, maxGam), -maxGam);

    // Gamma error (degrees)
    double elevCmd = desGamma - state.kin.gmma * RTD;

    // Scale by speed ratio
    elevCmd *= 0.25 * state.vcas / 350.0;

    // Divide by cos(phi) if within 45° gamma
    if (std::fabs(state.kin.gmma) < (45.0 * DTR))
        elevCmd /= std::max(0.1, state.kin.cosphi);

    // Square with sign preservation
    if (elevCmd > 0.0)
        elevCmd *= elevCmd;
    else
        elevCmd *= -elevCmd;

    // G command = integral + proportional + gravity compensation
    double gammaCmd = digi.gammaHoldIError + elevCmd + (1.0 / std::max(0.1, state.kin.cosphi));

    // Clamp G command to the FreeFalcon envelope [-2, 6.5].
    const double gammaCmdClamped = std::max(std::min(gammaCmd, 6.5), -2.0);

    // Integral with leaky integration.
    //
    // FreeFalcon's GammaHold (mnvers.cpp:857) uses a pure integrator:
    //   gammaHoldIError += 0.0025 * elevCmd  (per 0.06s major frame)
    // This works at FreeFalcon's 16.67 Hz AI rate, but at F4Flight's 60 Hz
    // the integral winds up to ±1.0 during the initial gamma capture (when
    // the squared elevCmd is 15+) and then takes 5+ seconds to unwind,
    // driving a persistent ±5G porpoising oscillation.
    //
    // Fix: make the integrator leaky with a 10-second time constant. This
    // allows the integral to provide trim for sustained climbs (it builds
    // up when elevCmd is sustained) but prevents it from persisting after
    // the gamma error is gone (it decays at 10% per second). The leak rate
    // is chosen to be slow enough that it doesn't affect steady-state trim
    // but fast enough to kill the porpoising within 2-3 oscillation cycles.
    //
    // leak rate = 1 - exp(-dt / 10.0) ≈ dt/10 for small dt
    constexpr double kIntegralTau = 10.0;  // seconds
    const double leakFactor = std::exp(-digi.dt / kIntegralTau);
    digi.gammaHoldIError = digi.gammaHoldIError * leakFactor
                         + 0.0025 * elevCmd * (digi.dt / 0.06);
    digi.gammaHoldIError = std::max(std::min(digi.gammaHoldIError, 1.0), -1.0);

    SetPstick(gammaCmdClamped, maxGs, GCommand, digi, state);
}

void DigiAI::AltHold(double desAlt, DigiState& digi, const AircraftState& state,
                     double maxGs) {
    // FreeFalcon autopilot.cpp:332-333:
    //   alterr = currAlt + self->ZPos();
    //   alterr -= self->ZDelta();
    // ZPos() is negative altitude (world Z-down), ZDelta() is world Z velocity
    // (ft/s, negative when climbing). Subtracting ZDelta provides derivative
    // (lead) damping that prevents the porpoising the F4Flight port previously
    // exhibited. state.kin.zdot has identical sign/magnitude semantics to
    // ZDelta().
    double alterr = (desAlt + state.kin.z) - state.kin.zdot;

    double abs_alterr = std::fabs(alterr);
    double gain;
    if (abs_alterr < 15.0)
        gain = 0.0;
    else if (abs_alterr < 50.0)
        gain = 0.0015;
    else if (abs_alterr < 100.0)
        gain = 0.005;
    else
        gain = 0.015;

    GammaHold(alterr * gain, digi, state, maxGs);
}

bool DigiAI::AltitudeHold(double desAlt, DigiState& digi,
                          const AircraftState& state,
                          const FlightControlSystem& /*fcs*/,
                          FcsState& fcsState, double maxGs) {
    SetYpedal(0.0, digi);

    // Wings level: rstick = -roll * 2 * RTD
    double rollDeg = state.kin.phi * RTD;
    double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

    // FreeFalcon mnvers.cpp:766: SetMaxRoll(0.0F) — tell the FCS RollIt loop
    // to limit phi to 0° (wings level). Without this, the FCS has no inner-
    // loop roll limit and the wings-level proportional controller limit-
    // cycles (bank slowly diverges to ±90°).
    fcsState.maxRoll = 0.0;

    // FreeFalcon mnvers.cpp:769-781:
    //   alterr = desAlt + self->ZPos();
    //   alterr -= self->ZDelta();
    // The -ZDelta term is derivative damping (lead compensation).
    double alterr = (desAlt + state.kin.z) - state.kin.zdot;
    bool retval = (std::fabs(alterr) < 25.0);

    GammaHold(alterr * 0.015, digi, state, maxGs);
    return retval;
}

bool DigiAI::HeadingAndAltitudeHold(double desPsi, double desAlt,
                                    DigiState& digi, const AircraftState& state,
                                    const FlightControlSystem& fcs,
                                    FcsState& fcsState, double maxGs) {
    double psiErr = headingError(desPsi, state.kin.sigma);

    SetYpedal(0.0, digi);

    bool retval = false;
    if (std::fabs(psiErr) < 5.0 * DTR) {
        // Near heading: wings level + altitude hold
        double rollDeg = state.kin.phi * RTD;
        double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        // FreeFalcon mnvers.cpp:805-806:
        //   SetMaxRoll(0.0F);
        //   SetMaxRollDelta(-self->Roll() * 2.0F * RTD);
        // Tell the FCS RollIt loop to limit phi to 0° (wings level), and
        // scale the roll-rate damping by 2x the current roll angle so the
        // aircraft decelerates its roll rate as it approaches level.
        // NOTE: FreeFalcon does NOT reset startRoll here. startRoll
        // accumulates indefinitely (eom.cpp:810). The damping term
        // 1 - startRoll/maxRollDelta would eventually go negative, but
        // FreeFalcon's units mismatch (startRoll in rad, maxRollDelta in
        // deg) keeps it near 1.0. We mirror that behavior — do NOT reset
        // startRoll here.
        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = std::fabs(rollDeg * 2.0);
        if (fcsState.maxRollDelta < 5.0) fcsState.maxRollDelta = 5.0;

        // FreeFalcon mnvers.cpp:809-817:
        //   altErr = desAlt + self->ZPos();
        //   altErr -= self->ZDelta();
        // The -ZDelta term is derivative damping.
        double altErr = (desAlt + state.kin.z) - state.kin.zdot;
        if (std::fabs(altErr) < 25.0) retval = true;
        GammaHold(altErr * 0.015, digi, state, maxGs);
    } else {
        // Far from heading: level turn
        double turnDir = (psiErr > 0.0) ? 1.0 : -1.0;
        DigiAI::LevelTurn(2.0, turnDir, false, digi, state, fcs, fcsState, maxGs);
    }
    return retval;
}

void DigiAI::LevelTurn(double loadFactor, double turnDir, bool newTurn,
                       DigiState& digi, const AircraftState& state,
                       const FlightControlSystem& /*fcs*/,
                       FcsState& fcsState, double maxGs) {
    if (newTurn) {
        digi.gammaHoldIError = 0.0;
        digi.trackMode = 0;
    }

    if (digi.trackMode != 0) {
        // Phase 2: banked turn
        double edroll = std::atan(std::sqrt(std::max(0.0, loadFactor * loadFactor - 1.0)));
        digi.maxRollDelta = edroll * RTD;
        // FreeFalcon mnvers.cpp:729-730: ResetMaxRoll() + SetMaxRollDelta(edroll*RTD).
        // ResetMaxRoll restores maxRoll to the vehicle's default (80° here),
        // which disables the hard phi-limit in RollIt (the turn target is
        // enforced by the roll-error P-controller instead). maxRollDelta is
        // set to the target bank angle so the FCS damps the roll rate as the
        // aircraft approaches edroll.
        fcsState.maxRoll = 80.0;  // "no limit" sentinel
        fcsState.maxRollDelta = edroll * RTD;
        if (newTurn) fcsState.startRoll = 0.0;
        edroll = edroll * turnDir - state.kin.mu;

        double rollErr = edroll * RTD * 2.50;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        if (std::fabs(edroll) < 5.0 * DTR || digi.trackMode == 2) {
            // FreeFalcon mnvers.cpp:737:
            //   alterr = (holdAlt + self->ZPos() - self->ZDelta()) * 0.015F;
            // The -ZDelta term is derivative damping.
            double alterr = ((digi.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
            GammaHold(alterr, digi, state, maxGs);
            digi.trackMode = 2;
        } else {
            SetPstick(0.0, 5.0, GCommand, digi, state);
        }
    } else {
        // Phase 1: level wings
        double rollErr = -state.kin.phi * RTD;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        // FreeFalcon mnvers.cpp:747-748: SetMaxRoll(0) + SetMaxRollDelta(5°).
        // While leveling the wings before the turn, limit phi to 0°.
        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = 5.0;
        digi.maxRoll = 0.0;
        digi.maxRollDelta = 5.0 * RTD;

        double elerr = -state.kin.gmma;
        SetPstick(elerr * RTD, 2.5, ErrorCommand, digi, state);

        if (std::fabs(state.kin.gmma) < 2.0 * DTR &&
            std::fabs(state.kin.phi) < 10.0 * DTR)
            digi.trackMode = 1;
    }

    SetYpedal(0.0, digi);
}

// MachHold — direct port of DigitalBrain::MachHold (mnvers.cpp:414-665),
// reduced to the non-combat path (no targetPtr, no flightLead, no IRCM).
//
// Defaults inlined from FreeFalcon's g_f* / g_b* config (f4config.cpp):
//   g_fFuelBaseProp      = 100.0
//   g_fFuelMultProp      = 0.008
//   g_fFuelTimeStep      = 0.001
//   g_fFuelVtClip        = 5.0
//   g_fFuelVtDotMult     = 5.0
//   g_bFuelLimitBecauseVtDot = true
//   g_fePropFactor       = 40.0   (only used in the curMaxStoreSpeed branch)
//
// burnerDelta defaults to 500.0 (non-combat). Callers can pass 700.0 for
// Wingy/Waypoint mode or 100.0 for combat modes.
//
// Key fixes vs. the previous F4Flight port:
//   1. Uses state.vtDot (ft/s^2, set by EOM) instead of state.netAccel * 60,
//      which was frame-rate-coupled and only correct at 60 Hz.
//   2. Uses g_fFuelVtDotMult = 5.0 (not 0.1) — 50x larger derivative gain.
//   3. Integral gain is now 0.001 * dt per call (frame-rate-independent);
//      previously 0.001 * 6.0 * (1/60) = 0.0001, which is 6x too large at
//      60 Hz and wrong at any other rate.
//   4. Adds g_bFuelLimitBecauseVtDot anti-windup: zeroes the integral when
//      it opposes the proportional error sign.
//   5. Adds the g_fePropFactor subtraction in the curMaxStoreSpeed branch.
bool DigiAI::MachHold(double targetSpeed, double currentSpeed, bool adjustPitch,
                      DigiState& digi, const AircraftState& state,
                      double minVcas, double maxVcas,
                      double dt, double burnerDelta) {
    // --- FreeFalcon defaults (f4config.cpp) ---
    constexpr double kFuelBaseProp      = 100.0;
    constexpr double kFuelMultProp      = 0.008;
    constexpr double kFuelTimeStep      = 0.001;
    constexpr double kFuelVtClip        = 5.0;
    constexpr double kFuelVtDotMult     = 5.0;
    constexpr bool   kLimitBecauseVtDot = true;
    constexpr double kEPropFactor       = 40.0;

    double eProp = targetSpeed - currentSpeed;

    // Clamp target into [minVcas, maxVcas - 20].
    // FreeFalcon also subtracts g_fePropFactor from eProp when clamping to
    // curMaxStoreSpeed (mnvers.cpp:440). We mirror that here.
    if (targetSpeed < minVcas) {
        targetSpeed = minVcas;
        eProp = targetSpeed - currentSpeed;
    }
    if (targetSpeed > maxVcas - 20.0) {
        targetSpeed = maxVcas - 20.0;
        eProp = targetSpeed - currentSpeed - kEPropFactor;
    }

    double thr;
    if (eProp < -100.0) {
        // Idle + speed brakes (we don't command speed brakes here; F4Flight
        // does not have an AI-driven speed brake channel).
        thr = 0.0;
    } else if (eProp < -50.0) {
        // Idle
        thr = 0.0;
    } else {
        if (eProp >= burnerDelta) {
            // Full afterburner
            thr = 1.5;
        } else {
            // Linear throttle + VtDot-aware integral
            double usedVtDot = state.vtDot;
            if (usedVtDot >  kFuelVtClip) usedVtDot =  kFuelVtClip;
            else if (usedVtDot < -kFuelVtClip) usedVtDot = -kFuelVtClip;

            thr = (eProp + kFuelBaseProp) * kFuelMultProp;
            digi.autoThrottle += (eProp - usedVtDot * kFuelVtDotMult)
                                 * kFuelTimeStep * dt;

            // g_bFuelLimitBecauseVtDot anti-windup: don't let the integral
            // oppose the proportional error sign.
            if (kLimitBecauseVtDot) {
                if (eProp > 0.0 && digi.autoThrottle < 0.0)
                    digi.autoThrottle = 0.0;
                else if (eProp < 0.0 && digi.autoThrottle > 0.0)
                    digi.autoThrottle = 0.0;
            }

            digi.autoThrottle = std::max(std::min(digi.autoThrottle, 1.5), -1.5);
            thr += digi.autoThrottle;

            // No afterburner in non-combat modes (mnvers.cpp:521-564). Cap at
            // 0.99 to keep the engine in MIL.
            thr = std::min(thr, 0.99);
        }
    }

    // Pitch-throttle coupling
    if (adjustPitch) {
        thr += std::fabs(digi.pStick) / 15.0;
    }

    thr = std::max(std::min(thr, 1.5), 0.0);

    digi.throttle = thr;

    return std::fabs(eProp) < 0.1 * targetSpeed;
}

void DigiAI::Loiter(DigiState& digi, const AircraftState& state,
                    const FlightControlSystem& /*fcs*/,
                    FcsState& fcsState, double maxGs) {
    double desBank = 30.0 * DTR;
    double rollErr = (desBank - state.kin.phi) * RTD;
    if (rollErr > 180.0) rollErr -= 360.0;
    else if (rollErr < -180.0) rollErr += 360.0;

    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

    // Loiter holds a 30° bank. Let the FCS RollIt loop damp toward 30°.
    fcsState.maxRoll = 30.0;
    fcsState.maxRollDelta = 30.0;

    // FreeFalcon mnvers.cpp:690 (Loiter body): alterr includes -ZDelta.
    //   alterr = (holdAlt + self->ZPos() - self->ZDelta()) * 0.015F;
    double alterr = ((digi.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
    GammaHold(alterr, digi, state, maxGs);
    SetYpedal(0.0, digi);
}

// ===========================================================================
// SteeringController
// ===========================================================================

SteeringController::SteeringController() {
    digi_.reset();
}

PilotInput SteeringController::compute(const AircraftState& state, double dt,
                                       double groundZ,
                                       const FlightControlSystem& fcs,
                                       FcsState& fcsState) {
    PilotInput out;
    (void)groundZ;

    // Store dt so all DigiAI functions can do frame-rate-independent smoothing
    // and integration. FreeFalcon's AI runs at 0.06s (16.67 Hz); our host
    // calls us at whatever dt it uses (typically 1/60 s).
    digi_.dt = dt;

    switch (mode_) {
        case Mode::HeadingAltitude:
            DigiAI::HeadingAndAltitudeHold(digi_.holdPsi, digi_.holdAlt,
                                   digi_, state, fcs, fcsState, digi_.maxGs);
            // burnerDelta=700 matches FreeFalcon WaypointMode (g_fWaypointBurnerDelta).
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0, dt, 700.0);
            break;

        case Mode::Waypoint:
            runWaypoint(state, dt, fcs, fcsState, out);
            break;

        case Mode::Loiter:
            DigiAI::Loiter(digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0, dt, 700.0);
            break;

        case Mode::Manual:
            out = manual_;
            return out;

        default:
            DigiAI::AltitudeHold(digi_.holdAlt, digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0, dt, 700.0);
            break;
    }

    out.pstick = limit(digi_.pStick, -1.0, 1.0);
    out.rstick = limit(digi_.rStick, -1.0, 1.0);
    out.ypedal = limit(digi_.yPedal, -1.0, 1.0);
    out.throttle = limit(digi_.throttle, 0.0, 1.5);
    out.refueling = false;
    return out;
}

void SteeringController::runWaypoint(const AircraftState& state, double dt,
                                     const FlightControlSystem& fcs,
                                     FcsState& fcsState,
                                     PilotInput& /*out*/) {
    if (curWp_ >= wps_.size()) {
        DigiAI::HeadingAndAltitudeHold(digi_.holdPsi, digi_.holdAlt,
                               digi_, state, fcs, fcsState, digi_.maxGs);
        DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                 digi_, state, 200.0, 800.0, dt, 700.0);
        return;
    }

    const Vec3& wp = wps_[curWp_];
    double dx = wp.x - state.kin.x;
    double dy = wp.y - state.kin.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < captureRadius_) {
        ++curWp_;
        if (curWp_ >= wps_.size()) {
            DigiAI::HeadingAndAltitudeHold(digi_.holdPsi, digi_.holdAlt,
                                   digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0, dt, 700.0);
            return;
        }
    }

    double desHeading = std::atan2(dy, dx);
    double desAlt = -wp.z;

    DigiAI::HeadingAndAltitudeHold(desHeading, desAlt,
                           digi_, state, fcs, fcsState, digi_.maxGs);
    DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
             digi_, state, 200.0, 800.0, dt, 700.0);
}

} // namespace f4flight
