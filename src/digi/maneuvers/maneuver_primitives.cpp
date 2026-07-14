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

        // NOTE: only set fcsState.maxRoll here (per-frame FCS limit).
        // Setting digi.maxRoll would clobber the host's setMaxBank() config
        // and persist across frames, degrading roll authority in subsequent
        // nav modes. fcsState.maxRoll = 0 alone is sufficient to command
        // wings-level for this frame.
        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = 5.0;

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
// Combat primitives
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

// ---------------------------------------------------------------------------
// AutoTrack — the core offensive BFM primitive.
//
// Port of FreeFalcon mnvers.cpp:211-298. This is the REAL AutoTrack:
//   1. Compute body-frame relative position (rx, ry, rz) via DCM transpose.
//   2. Compute off-boresight angle (ata), lift-vector roll (droll), elevation
//      error (elerr).
//   3. Dispatch three branches:
//      ata < 5°:   fine track — ErrorCommand pitch, wings-level roll
//      ata < 10°:  (BVR only) flip-over — roll opposite + neg G
//      large ata:  roll lift vector onto target + AlphaCommand pull
//
// This is fundamentally different from TrackPoint/HeadingAndAltitudeHold:
//   TrackPoint turns the VELOCITY VECTOR to a heading and holds ALTITUDE.
//   AutoTrack rolls the LIFT VECTOR onto the target and PULLS along body z.
// These are geometrically opposite control laws.
//
// Reads trackX/Y/Z from DigiState (caller sets them before invoking).
// Returns ata (degrees) for the caller's use.
// ---------------------------------------------------------------------------
double ManeuverPrimitives::AutoTrack(DigiState& digi, const AircraftState& state,
                                      FcsState& fcsState, double maxGs) {
    // World-frame relative position
    const double xft = digi.trackX - state.kin.x;
    const double yft = digi.trackY - state.kin.y;
    const double zft = digi.trackZ - state.kin.z;  // NED: z negative up

    // Transform to body frame using DCM transpose (world-to-body).
    // F4Flight's dcm is body-to-world; transpose gives world-to-body.
    // FreeFalcon uses self->dmx directly (which is world-to-body).
    const Matrix3& dcm = state.kin.dcm;
    const double rx = dcm.m[0][0] * xft + dcm.m[1][0] * yft + dcm.m[2][0] * zft;
    const double ry = dcm.m[0][1] * xft + dcm.m[1][1] * yft + dcm.m[2][1] * zft;
    const double rz = dcm.m[0][2] * xft + dcm.m[1][2] * yft + dcm.m[2][2] * zft;

    // Off-boresight angle (radians → degrees)
    const double ata = std::atan2(std::sqrt(ry * ry + rz * rz), rx) * RTD;

    // Lift-vector roll: angle to roll so the lift vector points at the target
    const double droll = std::atan2(ry, -rz);

    // Elevation error: angle above/below the nose
    const double xyRange = std::sqrt(rx * rx + ry * ry);
    const double elerr = std::atan2(-rz, xyRange) * RTD;

    // Azimuth error (for fine-track yaw)
    const double azerr = std::atan2(ry, rx) * RTD;

    if (ata < 5.0) {
        // Fine track — small corrections
        SetPstick(1.5 * elerr, maxGs, CommandType::ErrorCommand, digi, state);
        SetYpedal(azerr / 4.0, digi);
        // Wings level: damp roll
        digi.rStick = -state.kin.phi * RTD * 5.0 * DTR;
        digi.rStick = limit(digi.rStick, -1.0, 1.0);
    } else if (ata < 10.0) {
        // BVR flip-over branch (only meaningful at BVR ranges, but we
        // include it for fidelity). If droll is near ±180°, roll the
        // opposite direction and push negative G instead of rolling
        // all the way around.
        if (droll > 150.0 * DTR && digi.rStick < 0.5) {
            SetRstick(droll * RTD - 180.0, digi, FlightControlSystem{}, fcsState);
            SetPstick(-ata, maxGs, CommandType::ErrorCommand, digi, state);
        } else if (droll < -150.0 * DTR && digi.rStick > -0.5) {
            SetRstick(droll * RTD + 180.0, digi, FlightControlSystem{}, fcsState);
            SetPstick(-ata, maxGs, CommandType::ErrorCommand, digi, state);
        } else {
            SetRstick(droll * RTD, digi, FlightControlSystem{}, fcsState);
            SetPstick(ata * DTR, maxGs, CommandType::ErrorCommand, digi, state);
        }
        SetYpedal(0.0, digi);
    } else {
        // Large ata — roll lift vector onto target + pull
        // AlphaCommand: the pull scales with ata but is limited by droll
        // to avoid over-stressing at high roll angles.
        SetRstick(droll * RTD, digi, FlightControlSystem{}, fcsState);

        const double drollAbs = std::fabs(droll);
        const double alphaCmd = ata * std::min((30.0 * DTR) / std::max(drollAbs, 0.001), 1.0);
        SetPstick(alphaCmd, maxGs, CommandType::AlphaCommand, digi, state);

        SetYpedal(0.0, digi);

        // Set max roll limits so the FCS allows the commanded bank
        fcsState.maxRoll = std::fabs(state.kin.phi + droll) * RTD;
        fcsState.maxRollDelta = std::fabs(droll) * RTD;
    }

    return ata;
}

// ---------------------------------------------------------------------------
// GunsAutoTrack — gun-specific tracking primitive.
//
// Port of FreeFalcon gengage.cpp:362-433. SEPARATE from offensive AutoTrack:
//   - Adds lead for bullet TOF (target velocity * TOF)
//   - Corrects for gravity drop (0.5 * g * TOF²)
//   - Biases rz by 2× for lead
//   - Uses GCommand (vs AutoTrack's AlphaCommand for large ata)
//   - Scales pull by min(25°/|droll|, 1) to limit over-G at high roll
//
// Reads trackX/Y/Z from DigiState. The caller (CoarseGunsTrack) sets
// these to the lead-aim point.
// ---------------------------------------------------------------------------
double ManeuverPrimitives::GunsAutoTrack(DigiState& digi,
                                          const AircraftState& state,
                                          FcsState& fcsState,
                                          double maxGs) {
    // World-frame relative position (already lead-corrected by caller)
    const double xft = digi.trackX - state.kin.x;
    const double yft = digi.trackY - state.kin.y;
    const double zft = digi.trackZ - state.kin.z;

    // Transform to body frame using DCM transpose (world-to-body)
    const Matrix3& dcm = state.kin.dcm;
    const double rx = dcm.m[0][0] * xft + dcm.m[1][0] * yft + dcm.m[2][0] * zft;
    const double ry = dcm.m[0][1] * xft + dcm.m[1][1] * yft + dcm.m[2][1] * zft;
    const double rz = dcm.m[0][2] * xft + dcm.m[1][2] * yft + dcm.m[2][2] * zft;

    // Bias rz by 2× for lead (FF gengage.cpp:414)
    const double rzBiased = -(rz * 2.0);

    // Lift-vector roll
    const double droll = std::atan2(ry, rzBiased);

    // ATA including elevation bias
    const double ata = std::atan2(std::sqrt(ry * ry + rzBiased * rzBiased), rx);

    // Scale pull based on roll error (FF gengage.cpp:423)
    const double pullFact = std::min((25.0 * DTR) / std::max(std::fabs(droll), 0.001), 1.0);

    // GCommand pull (FF gengage.cpp:425)
    SetPstick(ata * RTD * 2.0 * pullFact, maxGs, CommandType::GCommand, digi, state);
    SetRstick(droll * RTD, digi, FlightControlSystem{}, fcsState);
    SetYpedal(0.0, digi);

    // Set max roll limits so the FCS allows the commanded bank
    fcsState.maxRoll = std::fabs(state.kin.phi + droll) * RTD;
    fcsState.maxRollDelta = std::fabs(droll) * RTD;

    return ata;
}

// ---------------------------------------------------------------------------
// SimpleTrackElevation — pure proportional elevation-angle tracker.
//
// Port of FreeFalcon wingmnvers.cpp:333-397. This is the landing pitch law:
//   altErr = -zft / scale
//   if |zft| > 2000: altErr *= 0.5  (soften for large errors)
//   if climbing and vt < 600 kts: altErr *= vt / (600 kts)  (low-airspeed limit)
//   clamp to [-0.5, +0.5]
//
// No integral, no +1G bias, no gamma feedback — structurally incapable of
// the Phugoid oscillation that GammaHold produces.
// ---------------------------------------------------------------------------
double ManeuverPrimitives::SimpleTrackElevation(double zft, double scale,
                                                  const AircraftState& state) {
    if (state.kin.z == 0.0 && zft == 0.0) return 0.0;  // guard

    double altErr = -zft / std::max(scale, 1.0);

    if (std::fabs(zft) > 2000.0)
        altErr *= 0.5;

    // Limit climb based on airspeed (only when climbing, not descending)
    const double vtFtps = state.kin.vt;
    const double climbLimitSpeed = 600.0 * KNOTS_TO_FTPSEC;
    if (-zft > 0.0 && vtFtps < climbLimitSpeed) {
        altErr *= vtFtps / climbLimitSpeed;
    }

    return std::max(-0.5, std::min(0.5, altErr));
}

// ---------------------------------------------------------------------------
// SimpleTrackAzimuth — proportional azimuth tracker.
//
// Port of FreeFalcon wingmnvers.cpp:253-323. Returns roll command [-1, +1].
//   azErr = atan2(ry, rx) / π
//   clamp to [-1, +1]
// ---------------------------------------------------------------------------
double ManeuverPrimitives::SimpleTrackAzimuth(double rx, double ry) {
    double azErr = std::atan2(ry, rx);

    // Normalize to [-1, 1] by dividing by π
    azErr /= PI;

    return std::max(-1.0, std::min(1.0, azErr));
}

// ---------------------------------------------------------------------------
// TrackPointLanding — landing-specific primitive.
//
// Port of FreeFalcon mnvers.cpp:33-103. Combines SimpleTrackAzimuth (roll)
// and SimpleTrackElevation (pitch) with inline throttle logic.
//
// Reads trackX/Y/Z from DigiState. The caller sets:
//   trackX, trackY = runway threshold position
//   trackZ         = desired altitude at current range (glideslope altitude)
//
// NOTE: trackZ is in NED convention (negative up), matching state.kin.z.
// SimpleTrackElevation expects zft = (target_z - current_z), so we pass
// (trackZ - state.kin.z) directly.
// ---------------------------------------------------------------------------
void ManeuverPrimitives::TrackPointLanding(double targetSpeedKts,
                                            DigiState& digi, const AircraftState& state,
                                            double dt) {
    (void)dt;

    // World-frame relative position
    const double xft = digi.trackX - state.kin.x;
    const double yft = digi.trackY - state.kin.y;
    const double zft = digi.trackZ - state.kin.z;

    // Transform to body frame for azimuth
    const Matrix3& dcm = state.kin.dcm;
    const double rx = dcm.m[0][0] * xft + dcm.m[1][0] * yft + dcm.m[2][0] * zft;
    const double ry = dcm.m[0][1] * xft + dcm.m[1][1] * yft + dcm.m[2][1] * zft;

    // Roll: proportional azimuth tracker
    double rCmd = SimpleTrackAzimuth(rx, ry);
    // Clamp roll rate
    rCmd = std::max(-0.6, std::min(0.6, rCmd));
    digi.rStick = rCmd;

    // Pitch: proportional elevation tracker
    // zft = target_z - current_z (NED). If target is above us, zft > 0
    // (less negative), so we need to climb. SimpleTrackElevation takes zft
    // directly and returns altErr = -zft/scale.
    const double distXY = std::sqrt(xft * xft + yft * yft);
    const double elErr = SimpleTrackElevation(zft, distXY, state);
    digi.pStick = std::min(0.2, std::max(elErr, -0.3));

    // Throttle: hold approach speed
    const double targetSpeedFtps = targetSpeedKts * KNOTS_TO_FTPSEC;
    const double eProp = targetSpeedKts - state.vcas;  // kts

    if (eProp >= 150.0) {
        digi.autoThrottle = 1.5;
        digi.throttle = 1.5;  // burner
    } else if (eProp < -100.0) {
        digi.autoThrottle = 0.0;
        digi.throttle = 0.0;  // idle
    } else {
        // Proportional + integral throttle
        digi.autoThrottle += eProp * 0.01 * (dt / 0.06);
        digi.autoThrottle = std::max(0.0, std::min(1.5, digi.autoThrottle));
        digi.throttle = eProp * 0.02 + digi.autoThrottle - state.vtDot * (dt / 0.06) * 0.005;
    }
    digi.throttle = std::max(0.0, std::min(1.5, digi.throttle));
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
