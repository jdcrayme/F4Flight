// f4flight - digi/maneuvers/maneuver_primitives.cpp
//
// Implementation of the core maneuver primitives. This is the existing
// steering.cpp logic, re-homed into the digi/ subsystem. The old steering.cpp
// now delegates to these functions for backward compatibility.
//
// All functions are direct ports of the corresponding FreeFalcon methods,
// preserving the exact control laws, gain scheduling, and smoothing.

#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/fcs.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// Utility functions (internal linkage)
// ===========================================================================
namespace {

inline double headingError(double setpoint, double current) noexcept {
    double err = setpoint - current;
    while (err >  PI) err -= 2.0 * PI;
    while (err < -PI) err += 2.0 * PI;
    return err;
}

// FreeFalcon major-frame time constant for stick smoothing.
// Derived from: 0.2*old + 0.8*new at T=0.06s → tau = -T/ln(0.2) = 0.0373s
inline double stickSmoothAlpha(double dt) {
    constexpr double kTau = 0.0373;  // seconds
    return 1.0 - std::exp(-dt / kTau);
}

inline double computeRstick(double rollErr_deg, double kr01, double tr01,
                            double currentRstick, double dt) {
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollErr_deg * DTR * 0.75 / denom;
    const double a = stickSmoothAlpha(dt);
    return (1.0 - a) * currentRstick + a * stickCmd;
}

inline double limitRollError(double rollErr, double rollDeg, double maxRoll) {
    if (std::fabs(rollDeg) > maxRoll) {
        if (rollDeg > 0.0)
            return std::min(rollErr, maxRoll - rollDeg);
        else
            return std::max(rollErr, maxRoll + rollDeg);
    }
    return rollErr;
}

} // anonymous namespace

// ===========================================================================
// ManeuverPrimitives implementation
// ===========================================================================

void ManeuverPrimitives::SetPstick(double pitchError, double gLimit,
                                    CommandType commandType,
                                    DigiState& digi, const AircraftState& state) {
    double stickCmd = 0.0;

    if (commandType == CommandType::ErrorCommand) {
        if (pitchError > 30.0)
            stickCmd = gLimit;
        else if (pitchError > 0.0)
            stickCmd = gLimit * pitchError / 30.0 + 1.0;
        else if (pitchError > -30.0)
            stickCmd = gLimit * 0.5 * pitchError / 30.0;
        else
            stickCmd = -gLimit * 0.5;
    } else if (commandType == CommandType::GCommand) {
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

    // Frame-rate-independent smoothing
    const double a = stickSmoothAlpha(digi.dt);
    digi.pStick = (1.0 - a) * digi.pStick + a * stickCmd;
}

void ManeuverPrimitives::SetRstick(double rollError, DigiState& digi,
                                    const FlightControlSystem& /*fcs*/,
                                    const FcsState& fcsState) {
    const double kr01 = fcsState.kr01;
    const double tr01 = fcsState.tr01;
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollError * DTR * 0.75 / denom;
    const double a = stickSmoothAlpha(digi.dt);
    digi.rStick = (1.0 - a) * digi.rStick + a * stickCmd;
}

void ManeuverPrimitives::SetYpedal(double yawError, DigiState& digi) {
    const double a = stickSmoothAlpha(digi.dt);
    const double newVal = -yawError * RTD * 0.0125;
    digi.yPedal = (1.0 - a) * digi.yPedal + a * newVal;
}

void ManeuverPrimitives::GammaHold(double desGamma, DigiState& digi,
                                    const AircraftState& state, double maxGs) {
    const double maxGam = std::max(1.0, digi.maxGammaDeg);
    desGamma = std::max(std::min(desGamma, maxGam), -maxGam);

    double elevCmd = desGamma - state.kin.gmma * RTD;
    elevCmd *= 0.25 * state.vcas / 350.0;

    if (std::fabs(state.kin.gmma) < (45.0 * DTR))
        elevCmd /= std::max(0.1, state.kin.cosphi);

    if (elevCmd > 0.0)
        elevCmd *= elevCmd;
    else
        elevCmd *= -elevCmd;

    double gammaCmd = digi.gammaHoldIError + elevCmd + (1.0 / std::max(0.1, state.kin.cosphi));
    const double gammaCmdClamped = std::max(std::min(gammaCmd, 6.5), -2.0);

    // Leaky integrator (10s tau) — prevents porpoising at 60 Hz
    constexpr double kIntegralTau = 10.0;
    const double leakFactor = std::exp(-digi.dt / kIntegralTau);
    digi.gammaHoldIError = digi.gammaHoldIError * leakFactor
                         + 0.0025 * elevCmd * (digi.dt / 0.06);
    digi.gammaHoldIError = std::max(std::min(digi.gammaHoldIError, 1.0), -1.0);

    SetPstick(gammaCmdClamped, maxGs, CommandType::GCommand, digi, state);
}

void ManeuverPrimitives::AltHold(double desAlt, DigiState& digi,
                                  const AircraftState& state, double maxGs) {
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

bool ManeuverPrimitives::AltitudeHold(double desAlt, DigiState& digi,
                                       const AircraftState& state,
                                       const FlightControlSystem& /*fcs*/,
                                       FcsState& fcsState, double maxGs) {
    SetYpedal(0.0, digi);

    double rollDeg = state.kin.phi * RTD;
    double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

    fcsState.maxRoll = 0.0;

    double alterr = (desAlt + state.kin.z) - state.kin.zdot;
    bool retval = (std::fabs(alterr) < 25.0);

    GammaHold(alterr * 0.015, digi, state, maxGs);
    return retval;
}

bool ManeuverPrimitives::HeadingAndAltitudeHold(double desPsi, double desAlt,
                                                 DigiState& digi, const AircraftState& state,
                                                 const FlightControlSystem& fcs,
                                                 FcsState& fcsState, double maxGs) {
    double psiErr = headingError(desPsi, state.kin.sigma);

    SetYpedal(0.0, digi);

    bool retval = false;
    if (std::fabs(psiErr) < 5.0 * DTR) {
        double rollDeg = state.kin.phi * RTD;
        double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = std::fabs(rollDeg * 2.0);
        if (fcsState.maxRollDelta < 5.0) fcsState.maxRollDelta = 5.0;

        double altErr = (desAlt + state.kin.z) - state.kin.zdot;
        if (std::fabs(altErr) < 25.0) retval = true;
        GammaHold(altErr * 0.015, digi, state, maxGs);
    } else {
        double turnDir = (psiErr > 0.0) ? 1.0 : -1.0;
        LevelTurn(digi.turnLoadFactor, turnDir, false, digi, state, fcs, fcsState, maxGs);
    }
    return retval;
}

void ManeuverPrimitives::LevelTurn(double loadFactor, double turnDir, bool newTurn,
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
        fcsState.maxRoll = 80.0;  // "no limit" sentinel
        fcsState.maxRollDelta = edroll * RTD;
        if (newTurn) fcsState.startRoll = 0.0;
        edroll = edroll * turnDir - state.kin.mu;

        double rollErr = edroll * RTD * 2.50;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        if (std::fabs(edroll) < 5.0 * DTR || digi.trackMode == 2) {
            double alterr = ((digi.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
            GammaHold(alterr, digi, state, maxGs);
            digi.trackMode = 2;
        } else {
            SetPstick(0.0, 5.0, CommandType::GCommand, digi, state);
        }
    } else {
        // Phase 1: level wings
        double rollErr = -state.kin.phi * RTD;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = 5.0;
        digi.maxRoll = 0.0;
        digi.maxRollDelta = 5.0 * RTD;

        double elerr = -state.kin.gmma;
        SetPstick(elerr * RTD, 2.5, CommandType::ErrorCommand, digi, state);

        if (std::fabs(state.kin.gmma) < 2.0 * DTR &&
            std::fabs(state.kin.phi) < 10.0 * DTR)
            digi.trackMode = 1;
    }

    SetYpedal(0.0, digi);
}

bool ManeuverPrimitives::MachHold(double targetSpeed, double currentSpeed, bool adjustPitch,
                                   DigiState& digi, const AircraftState& state,
                                   double minVcas, double maxVcas,
                                   double dt, double burnerDelta) {
    constexpr double kFuelBaseProp      = 100.0;
    constexpr double kFuelMultProp      = 0.008;
    constexpr double kFuelTimeStep      = 0.001;
    constexpr double kFuelVtClip        = 5.0;
    constexpr double kFuelVtDotMult     = 5.0;
    constexpr bool   kLimitBecauseVtDot = true;
    constexpr double kEPropFactor       = 40.0;

    double eProp = targetSpeed - currentSpeed;

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
        thr = 0.0;
    } else if (eProp < -50.0) {
        thr = 0.0;
    } else {
        if (eProp >= burnerDelta) {
            thr = 1.5;
        } else {
            double usedVtDot = state.vtDot;
            if (usedVtDot >  kFuelVtClip) usedVtDot =  kFuelVtClip;
            else if (usedVtDot < -kFuelVtClip) usedVtDot = -kFuelVtClip;

            thr = (eProp + kFuelBaseProp) * kFuelMultProp;
            digi.autoThrottle += (eProp - usedVtDot * kFuelVtDotMult)
                                 * kFuelTimeStep * dt;

            if (kLimitBecauseVtDot) {
                if (eProp > 0.0 && digi.autoThrottle < 0.0)
                    digi.autoThrottle = 0.0;
                else if (eProp < 0.0 && digi.autoThrottle > 0.0)
                    digi.autoThrottle = 0.0;
            }

            digi.autoThrottle = std::max(std::min(digi.autoThrottle, 1.5), -1.5);
            thr += digi.autoThrottle;
            thr = std::min(thr, 0.99);
        }
    }

    if (adjustPitch) {
        thr += std::fabs(digi.pStick) / 15.0;
    }

    thr = std::max(std::min(thr, 1.5), 0.0);
    digi.throttle = thr;

    return std::fabs(eProp) < 0.1 * targetSpeed;
}

void ManeuverPrimitives::Loiter(DigiState& digi, const AircraftState& state,
                                 const FlightControlSystem& /*fcs*/,
                                 FcsState& fcsState, double maxGs) {
    double desBank = 30.0 * DTR;
    double rollErr = (desBank - state.kin.phi) * RTD;
    if (rollErr > 180.0) rollErr -= 360.0;
    else if (rollErr < -180.0) rollErr += 360.0;

    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick, digi.dt);

    fcsState.maxRoll = 30.0;
    fcsState.maxRollDelta = 30.0;

    double alterr = ((digi.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
    GammaHold(alterr, digi, state, maxGs);
    SetYpedal(0.0, digi);
}

// ===========================================================================
// Combat primitives (Tier 0 — basic implementations)
// ===========================================================================

void ManeuverPrimitives::TrackPoint(double targetX, double targetY, double targetAlt,
                                     DigiState& digi, const AircraftState& state,
                                     const FlightControlSystem& fcs,
                                     FcsState& fcsState, double maxGs) {
    // Compute heading to target point
    const double dx = targetX - state.kin.x;
    const double dy = targetY - state.kin.y;
    const double desHeading = std::atan2(dy, dx);

    HeadingAndAltitudeHold(desHeading, targetAlt, digi, state, fcs, fcsState, maxGs);
}

void ManeuverPrimitives::AutoTrack(double targetX, double targetY, double targetAlt,
                                    double targetVx, double targetVy, double leadTime,
                                    DigiState& digi, const AircraftState& state,
                                    const FlightControlSystem& fcs,
                                    FcsState& fcsState, double maxGs) {
    // Lead the target by its velocity
    const double ledX = targetX + targetVx * leadTime;
    const double ledY = targetY + targetVy * leadTime;
    TrackPoint(ledX, ledY, targetAlt, digi, state, fcs, fcsState, maxGs);
}

void ManeuverPrimitives::VectorTrack(double desHeading, double desAlt, double desSpeed,
                                      DigiState& digi, const AircraftState& state,
                                      const FlightControlSystem& fcs,
                                      FcsState& fcsState, double maxGs, double dt) {
    HeadingAndAltitudeHold(desHeading, desAlt, digi, state, fcs, fcsState, maxGs);
    MachHold(desSpeed, state.vcas, true, digi, state, 200.0, 800.0, dt, 700.0);
}

} // namespace digi
} // namespace f4flight
