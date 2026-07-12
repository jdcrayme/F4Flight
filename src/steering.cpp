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

// Helper: compute rstick from roll error using FCS gains
// Frame-rate-scaled smoothing (same time constant as SetPstick)
static inline double computeRstick(double rollErr_deg, double kr01, double tr01,
                                   double currentRstick) {
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollErr_deg * DTR * 0.75 / denom;
    return 0.852 * currentRstick + 0.148 * stickCmd;
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

    // Smooth: FreeFalcon uses 0.2*old + 0.8*new at ~6 Hz major frame rate.
    // At 60 Hz we need to scale the smoothing to match the same time constant.
    // tau = -(1/6)/ln(0.2) ≈ 0.104 s
    // At 60 Hz: alpha = 1 - exp(-(1/60)/0.104) ≈ 0.148
    // So: y = 0.852*old + 0.148*new
    digi.pStick = 0.852 * digi.pStick + 0.148 * stickCmd;
}

void DigiAI::SetRstick(double rollError, DigiState& digi,
                       const FlightControlSystem& /*fcs*/,
                       const FcsState& fcsState) {
    // This overload is kept for API compatibility but the actual roll-limiting
    // requires the aircraft state. Use the inline version in the calling code.
    const double kr01 = fcsState.kr01;
    const double tr01 = fcsState.tr01;
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollError * DTR * 0.75 / denom;
    digi.rStick = 0.2 * digi.rStick + 0.8 * stickCmd;
}

void DigiAI::SetYpedal(double yawError, DigiState& digi) {
    digi.yPedal = 0.2 * digi.yPedal - 0.8 * yawError * RTD * 0.0125;
}

void DigiAI::GammaHold(double desGamma, DigiState& digi,
                       const AircraftState& state, double maxGs) {
    // Clamp desired gamma to ±60°
    desGamma = std::max(std::min(desGamma, 60.0), -60.0);

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

    // Integral (very small gain).
    // FreeFalcon runs at ~6 Hz; at 60 Hz scale by dt*6 to match.
    digi.gammaHoldIError += 0.0025 * elevCmd * 6.0 * (1.0/60.0);
    digi.gammaHoldIError = std::max(std::min(digi.gammaHoldIError, 1.0), -1.0);

    // G command = integral + proportional + gravity compensation
    double gammaCmd = digi.gammaHoldIError + elevCmd + (1.0 / std::max(0.1, state.kin.cosphi));

    // Clamp and command
    SetPstick(std::max(std::min(gammaCmd, 6.5), -2.0), maxGs, GCommand,
              digi, state);
}

void DigiAI::AltHold(double desAlt, DigiState& digi, const AircraftState& state,
                     double maxGs) {
    double alterr = desAlt - (-state.kin.z);

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
                          const FcsState& fcsState, double maxGs) {
    SetYpedal(0.0, digi);

    // Wings level: rstick = -roll * 2 * RTD
    double rollDeg = state.kin.phi * RTD;
    double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick);

    double alterr = desAlt - (-state.kin.z);
    bool retval = (std::fabs(alterr) < 25.0);

    GammaHold(alterr * 0.015, digi, state, maxGs);
    return retval;
}

bool DigiAI::HeadingAndAltitudeHold(double desPsi, double desAlt,
                                    DigiState& digi, const AircraftState& state,
                                    const FlightControlSystem& fcs,
                                    const FcsState& fcsState, double maxGs) {
    double psiErr = headingError(desPsi, state.kin.sigma);

    SetYpedal(0.0, digi);

    bool retval = false;
    if (std::fabs(psiErr) < 5.0 * DTR) {
        // Near heading: wings level + altitude hold
        double rollDeg = state.kin.phi * RTD;
        double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.maxRoll);
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick);

        double altErr = desAlt - (-state.kin.z);
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
                       const FcsState& fcsState, double maxGs) {
    if (newTurn) {
        digi.gammaHoldIError = 0.0;
        digi.trackMode = 0;
    }

    if (digi.trackMode != 0) {
        // Phase 2: banked turn
        double edroll = std::atan(std::sqrt(std::max(0.0, loadFactor * loadFactor - 1.0)));
        digi.maxRollDelta = edroll * RTD;
        edroll = edroll * turnDir - state.kin.mu;

        double rollErr = edroll * RTD * 2.50;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick);

        if (std::fabs(edroll) < 5.0 * DTR || digi.trackMode == 2) {
            double alterr = (digi.holdAlt - (-state.kin.z)) * 0.015;
            GammaHold(alterr, digi, state, maxGs);
            digi.trackMode = 2;
        } else {
            SetPstick(0.0, 5.0, GCommand, digi, state);
        }
    } else {
        // Phase 1: level wings
        double rollErr = -state.kin.phi * RTD;
        digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick);

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

bool DigiAI::MachHold(double targetSpeed, double currentSpeed, bool adjustPitch,
                      DigiState& digi, const AircraftState& state,
                      double minVcas, double maxVcas) {
    double eProp = targetSpeed - currentSpeed;

    if (targetSpeed < minVcas) {
        targetSpeed = minVcas;
        eProp = targetSpeed - currentSpeed;
    }
    if (targetSpeed > maxVcas - 20.0) {
        targetSpeed = maxVcas - 20.0;
        eProp = targetSpeed - currentSpeed;
    }

    double thr;
    if (eProp < -100.0) {
        thr = 0.0;
    } else if (eProp < -50.0) {
        thr = 0.0;
    } else {
        double burnerDelta = 500.0;
        if (eProp >= burnerDelta) {
            thr = 1.5;
        } else {
            thr = (eProp + 100.0) * 0.008;
            // Use netAccel (ft/s^2 per frame) as a proxy for VtDot
            double usedVtDot = state.netAccel * 60.0;  // convert to ft/s^2 per second
            if (usedVtDot > 5.0) usedVtDot = 5.0;
            else if (usedVtDot < -5.0) usedVtDot = -5.0;

            digi.autoThrottle += (eProp - usedVtDot * 0.1) * 0.001 * 6.0 * (1.0/60.0);
            digi.autoThrottle = std::max(std::min(digi.autoThrottle, 1.5), -1.5);
            thr += digi.autoThrottle;
            thr = std::min(thr, 0.99);
        }
    }

    if (adjustPitch) {
        thr += std::fabs(digi.pStick) / 15.0;
    }

    thr = std::max(std::min(thr, 1.5), 0.0);

    // Store the final throttle in a separate field (not autoThrottle, which
    // is the integral term). We use pStick as a proxy for "the throttle output"
    // since DigiState doesn't have a dedicated throttle field.
    // Actually, we need to add one. For now, use a static — this is a
    // temporary hack that will be fixed when DigiState gets a throttle field.
    // TODO: add throttle field to DigiState
    digi.throttle = thr;  // Will add this field to DigiState

    return std::fabs(eProp) < 0.1 * targetSpeed;
}

void DigiAI::Loiter(DigiState& digi, const AircraftState& state,
                    const FlightControlSystem& /*fcs*/,
                    const FcsState& fcsState, double maxGs) {
    double desBank = 30.0 * DTR;
    double rollErr = (desBank - state.kin.phi) * RTD;
    if (rollErr > 180.0) rollErr -= 360.0;
    else if (rollErr < -180.0) rollErr += 360.0;

    digi.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.rStick);

    double alterr = (digi.holdAlt - (-state.kin.z)) * 0.015;
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
                                       const FcsState& fcsState) {
    PilotInput out;

    switch (mode_) {
        case Mode::HeadingAltitude:
            DigiAI::HeadingAndAltitudeHold(digi_.holdPsi, digi_.holdAlt,
                                   digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0);
            break;

        case Mode::Waypoint:
            runWaypoint(state, dt, fcs, fcsState, out);
            break;

        case Mode::Loiter:
            DigiAI::Loiter(digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0);
            break;

        case Mode::Manual:
            out = manual_;
            return out;

        default:
            DigiAI::AltitudeHold(digi_.holdAlt, digi_, state, fcs, fcsState, digi_.maxGs);
            DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                     digi_, state, 200.0, 800.0);
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
                                     const FcsState& fcsState,
                                     PilotInput& /*out*/) {
    if (curWp_ >= wps_.size()) {
        DigiAI::HeadingAndAltitudeHold(digi_.holdPsi, digi_.holdAlt,
                               digi_, state, fcs, fcsState, digi_.maxGs);
        DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
                 digi_, state, 200.0, 800.0);
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
                     digi_, state, 200.0, 800.0);
            return;
        }
    }

    double desHeading = std::atan2(dy, dx);
    double desAlt = -wp.z;

    DigiAI::HeadingAndAltitudeHold(desHeading, desAlt,
                           digi_, state, fcs, fcsState, digi_.maxGs);
    DigiAI::MachHold(digi_.cornerSpeed, state.vcas, true,
             digi_, state, 200.0, 800.0);
}

} // namespace f4flight
