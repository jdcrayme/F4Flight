// f4flight - digi/maneuvers/maneuver_primitives.cpp
//
// Implementation of the core maneuver primitives. This is the existing
// steering.cpp logic, re-homed into the digi/ subsystem. The old steering.cpp
// now delegates to these functions for backward compatibility.
//
// All functions are direct ports of the corresponding FreeFalcon methods,
// preserving the exact control laws, gain scheduling, and smoothing.

#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

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
    const double a = stickSmoothAlpha(digi.nav.dt);
    digi.commands.pStick = (1.0 - a) * digi.commands.pStick + a * stickCmd;
}

void ManeuverPrimitives::SetRstick(double rollError, DigiState& digi,
                                    const FlightControlSystem& /*fcs*/,
                                    const FcsState& fcsState) {
    const double kr01 = fcsState.kr01;
    const double tr01 = fcsState.tr01;
    const double denom = std::max(kr01 * tr01, 0.1);
    const double stickCmd = rollError * DTR * 0.75 / denom;
    const double a = stickSmoothAlpha(digi.nav.dt);
    digi.commands.rStick = (1.0 - a) * digi.commands.rStick + a * stickCmd;
}

void ManeuverPrimitives::SetYpedal(double yawError, DigiState& digi) {
    const double a = stickSmoothAlpha(digi.nav.dt);
    const double newVal = -yawError * RTD * 0.0125;
    digi.commands.yPedal = (1.0 - a) * digi.commands.yPedal + a * newVal;
}

void ManeuverPrimitives::GammaHold(double desGamma, DigiState& digi,
                                    const AircraftState& state, double maxGs) {
    // Phase-aware gain scheduling: read the current flight phase and use
    // the corresponding PhaseGainSet for the integral gain and gamma clamp.
    // This allows landing approach to use a lower gamma clamp (10°) than
    // combat (60°), and formation to use pure P (no integral) to prevent
    // windup during station-keeping.
    const PhaseGainSet gains = PhaseGainSet::forPhase(digi.nav.flightPhase);

    // Clamp the desired gamma to the phase-specific limit.
    const double maxGam = std::max(1.0, std::min(digi.config.maxGammaDeg, gains.gammaClamp));
    desGamma = std::max(std::min(desGamma, maxGam), -maxGam);

    double elevCmd = desGamma - state.kin.gmma * RTD;
    elevCmd *= 0.25 * state.vcas / 350.0;

    if (std::fabs(state.kin.gmma) < (45.0 * DTR))
        elevCmd /= std::max(0.1, state.kin.cosphi);

    if (elevCmd > 0.0)
        elevCmd *= elevCmd;
    else
        elevCmd *= -elevCmd;

    double gammaCmd = digi.nav.gammaHoldIError + elevCmd + (1.0 / std::max(0.1, state.kin.cosphi));
    const double gammaCmdClamped = std::max(std::min(gammaCmd, 6.5), -2.0);

    // Leaky integrator (10s tau) — prevents porpoising at 60 Hz.
    // The integral gain is phase-specific: 0 for Approach/Formation (pure P,
    // prevents windup during glideslope tracking), 0.0025 for Cruise/Combat
    // (steady-state error correction).
    constexpr double kIntegralTau = 10.0;
    const double leakFactor = std::exp(-digi.nav.dt / kIntegralTau);
    digi.nav.gammaHoldIError = digi.nav.gammaHoldIError * leakFactor
                         + gains.integralGain * elevCmd * (digi.nav.dt / 0.06);
    digi.nav.gammaHoldIError = std::max(std::min(digi.nav.gammaHoldIError, 1.0), -1.0);

    SetPstick(gammaCmdClamped, maxGs, CommandType::GCommand, digi, state);
}

void ManeuverPrimitives::PhugoidDamper(DigiState& digi, const AircraftState& state,
                                        double gain) {
    // Pitch rate (q) feedback. q is in rad/s — positive = nose pitching up.
    //
    // If gain < 0, use the phase-aware gain from PhaseGainSet::forPhase().
    // This lets callers write `PhugoidDamper(digi, as)` and get the right
    // damping for the current flight phase without hardcoding a gain.
    double effectiveGain = gain;
    if (gain < 0.0) {
        effectiveGain = PhaseGainSet::forPhase(digi.nav.flightPhase).phugoidGain;
    }
    //
    // The damper subtracts a term proportional to q from the pStick command.
    // If the aircraft is pitching up (q > 0), reduce the nose-up command
    // (oppose the pitch-up). If pitching down (q < 0), add nose-up command.
    // This damps the Phugoid oscillation.
    //
    // The gain is in units of pstick per rad/s of pitch rate. At gain=0.4,
    // a 0.05 rad/s (~3°/s) pitch rate produces 0.02 pstick correction —
    // enough to damp the Phugoid without interfering with the controller's
    // target tracking (the Phugoid's q is small, ~0.02-0.05 rad/s, while
    // maneuvering q is much larger).
    //
    // The damper is a pure derivative term: in steady-state trim q=0, so the
    // damper contributes nothing. It only acts when the aircraft is pitching
    // (i.e. during oscillation or maneuver transients).
    //
    // We also add airspeed scaling: at low speed the elevator has less
    // authority, so the damper needs a larger stick deflection to produce
    // the same pitch acceleration. Scale by (350 / vcas), clamped to [0.5, 2.0].
    const double q = state.kin.q;  // rad/s
    const double speedScale = std::max(0.5, std::min(2.0, 350.0 / std::max(150.0, state.vcas)));
    const double dampingTerm = -effectiveGain * q * speedScale;
    // Clamp the damping term to prevent excessive correction during large
    // maneuvering pitch rates (e.g. pull-up for takeoff rotation). The damper
    // should only act on the small Phugoid oscillation, not fight intentional
    // pitch maneuvers.
    const double clampedDamping = std::max(-0.15, std::min(0.15, dampingTerm));
    digi.commands.pStick = std::max(-1.0, std::min(1.0,
        digi.commands.pStick + clampedDamping));
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
    double rollErr = limitRollError(-rollDeg * 2.0, rollDeg, digi.config.maxRoll);
    digi.commands.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.commands.rStick, digi.nav.dt);

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
    (void)fcs;  // reserved for FCS-aware gain scheduling (FF reads fcs limits)
    double psiErr = headingError(desPsi, state.kin.sigma);

    SetYpedal(0.0, digi);

    bool retval = false;

    // --- Continuous heading-to-roll controller (no LevelTurn) ---
    //
    // The previous code used a hard 5° threshold: below 5° it commanded
    // wings-level (maxRoll=0), above 5° it commanded a full LevelTurn at
    // turnLoadFactor (2G → 60° bank). This created TWO problems:
    //
    //   1. A limit cycle near waypoints — after rolling out, the aircraft
    //      would overshoot the heading, re-enter LevelTurn, overshoot again,
    //      producing ±20° bank chatter for 30-40 seconds.
    //
    //   2. A discontinuity at the threshold — when psiErr drops from 16° to
    //      14°, the commanded bank jumps from 60° (LevelTurn) to 5° (proportional),
    //      causing a violent roll reversal that can reach -88° bank.
    //
    // Fix: use a SINGLE continuous proportional controller for ALL heading
    // errors. The desired bank angle = psiErr × gain, capped at maxRoll.
    // No LevelTurn, no threshold, no discontinuity. The aircraft smoothly
    // banks proportional to heading error and smoothly rolls out as the
    // error approaches zero.
    //
    // Gain of 2.0: 45° heading error → 90° desired bank → clamped to maxRoll
    // (45° for fighters, 25° for heavies). This gives aggressive turns for
    // large errors (waypoint corners) and gentle corrections for small errors.
    constexpr double kHeadingToBankGain = 2.0;

    const double psiErrDeg = psiErr * RTD;
    double desiredBankDeg = psiErrDeg * kHeadingToBankGain;

    // Clamp the desired bank. The clamp limit is the LARGER of:
    //   - digi.config.maxRoll (the brain's navigation bank limit, e.g. 25° for heavy)
    //   - the bank angle implied by turnLoadFactor (e.g. 47.6° for 1.3G)
    //
    // The old LevelTurn code targeted the load-factor-derived bank (47.6° for
    // 1.3G) regardless of digi.config.maxRoll, because digi.config.maxRoll was only used
    // in the wings-level branch's limitRollError. If we clamp to digi.config.maxRoll
    // alone, heavy aircraft (maxRoll=25°) can't bank steeply enough to turn
    // within the waypoint capture radius — the turn radius at 25° bank and
    // 250 kts is ~12 NM, far larger than the 5000 ft capture radius.
    const double loadFactorBankDeg = std::atan(std::sqrt(std::max(0.0,
        digi.config.turnLoadFactor * digi.config.turnLoadFactor - 1.0))) * RTD;
    const double bankClamp = std::max(digi.config.maxRoll, loadFactorBankDeg);
    desiredBankDeg = std::max(-bankClamp, std::min(bankClamp, desiredBankDeg));

    const double rollDeg = state.kin.phi * RTD;
    double rollErr = (desiredBankDeg - rollDeg) * 2.0;
    rollErr = limitRollError(rollErr, rollDeg, digi.config.maxRoll);

    // Roll-rate damping: subtract a term proportional to the roll rate (p).
    // This prevents the limit-cycle oscillation that occurs on fast-rolling
    // aircraft (F-22, MiG-29) where the rstick saturates, the aircraft rolls
    // through the desired bank, and the rstick reverses too late. The damping
    // term reduces the rstick as the aircraft approaches the desired bank,
    // preventing overshoot.
    //
    // The damping gain is based on the CURRENT BANK ANGLE magnitude:
    //   - Large bank (>30°): high gain (0.5) — the aircraft is in a steep
    //     turn (RTB/waypoint case) and needs aggressive damping to prevent
    //     overshooting the desired heading.
    //   - Small bank (<10°): low gain (0.15) — the aircraft is near wings-
    //     level (formation station-keeping case) and needs light damping so
    //     the controller can make precise lateral corrections.
    //   - In between: linearly interpolated.
    //
    // This adaptive gain satisfies both use cases: RTB (steep turns, needs
    // heavy damping) and formation (shallow banks, needs light damping).
    // It doesn't use heading error because formation rejoin has large heading
    // errors but shallow banks (the wingman turns gradually toward the slot).
    const double absBankDeg = std::fabs(rollDeg);
    double dampGain;
    if (absBankDeg > 30.0) {
        dampGain = 0.5;
    } else if (absBankDeg < 10.0) {
        dampGain = 0.15;
    } else {
        // Linear interpolation between 10° (0.15) and 30° (0.5).
        dampGain = 0.15 + (absBankDeg - 10.0) / 20.0 * (0.5 - 0.15);
    }
    const double rollRateDamp = state.kin.p * RTD * dampGain;  // deg
    rollErr -= rollRateDamp;
    digi.commands.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01,
                                digi.commands.rStick, digi.nav.dt);

    // Allow the FCS to roll up to the desired bank angle. Use bankClamp
    // (not desiredBankDeg) so the FCS doesn't fight the turn when the
    // desired bank is less than the load-factor-derived limit.
    fcsState.maxRoll = bankClamp;
    fcsState.maxRollDelta = std::max(5.0, bankClamp * 2.0);

    double altErr = (desAlt + state.kin.z) - state.kin.zdot;
    if (std::fabs(altErr) < 25.0) retval = true;
    GammaHold(altErr * 0.015, digi, state, maxGs);

    return retval;
}

void ManeuverPrimitives::LevelTurn(double loadFactor, double turnDir, bool newTurn,
                                    DigiState& digi, const AircraftState& state,
                                    const FlightControlSystem& /*fcs*/,
                                    FcsState& fcsState, double maxGs) {
    if (newTurn) {
        digi.nav.gammaHoldIError = 0.0;
        digi.nav.trackMode = 0;
    }

    if (digi.nav.trackMode != 0) {
        // Phase 2: banked turn
        double edroll = std::atan(std::sqrt(std::max(0.0, loadFactor * loadFactor - 1.0)));
        // BUG FIX: do NOT clobber digi.config.maxRollDelta (a persistent config field)
        // with the per-frame roll error. The previous line
        //   digi.config.maxRollDelta = edroll * RTD;
        // was a dead write (nothing in the F4Flight codebase reads
        // digi.config.maxRollDelta — only fcsState.maxRollDelta is read by the FCS)
        // but it polluted the config field for any subsequent code that
        // might read it. The correct write is to fcsState.maxRollDelta
        // (per-frame FCS limit), which is already done on the next line.
        fcsState.maxRoll = 80.0;  // "no limit" sentinel
        fcsState.maxRollDelta = edroll * RTD;
        if (newTurn) fcsState.startRoll = 0.0;
        edroll = edroll * turnDir - state.kin.mu;

        double rollErr = edroll * RTD * 2.50;
        digi.commands.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.commands.rStick, digi.nav.dt);

        if (std::fabs(edroll) < 5.0 * DTR || digi.nav.trackMode == 2) {
            double alterr = ((digi.nav.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
            GammaHold(alterr, digi, state, maxGs);
            digi.nav.trackMode = 2;
        } else {
            SetPstick(0.0, 5.0, CommandType::GCommand, digi, state);
        }
    } else {
        // Phase 1: level wings
        double rollErr = -state.kin.phi * RTD;
        digi.commands.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.commands.rStick, digi.nav.dt);

        // NOTE: only set fcsState.maxRoll here (per-frame FCS limit).
        // Setting digi.config.maxRoll would clobber the host's setMaxBank() config
        // and persist across frames, degrading roll authority in subsequent
        // nav modes. fcsState.maxRoll = 0 alone is sufficient to command
        // wings-level for this frame.
        fcsState.maxRoll = 0.0;
        fcsState.maxRollDelta = 5.0;

        double elerr = -state.kin.gmma;
        SetPstick(elerr * RTD, 2.5, CommandType::ErrorCommand, digi, state);

        if (std::fabs(state.kin.gmma) < 2.0 * DTR &&
            std::fabs(state.kin.phi) < 10.0 * DTR)
            digi.nav.trackMode = 1;
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
            digi.nav.autoThrottle += (eProp - usedVtDot * kFuelVtDotMult)
                                 * kFuelTimeStep * dt;

            if (kLimitBecauseVtDot) {
                if (eProp > 0.0 && digi.nav.autoThrottle < 0.0)
                    digi.nav.autoThrottle = 0.0;
                else if (eProp < 0.0 && digi.nav.autoThrottle > 0.0)
                    digi.nav.autoThrottle = 0.0;
            }

            digi.nav.autoThrottle = std::max(std::min(digi.nav.autoThrottle, 1.5), -1.5);
            thr += digi.nav.autoThrottle;
            thr = std::min(thr, 0.99);
        }
    }

    if (adjustPitch) {
        thr += std::fabs(digi.commands.pStick) / 15.0;
    }

    thr = std::max(std::min(thr, 1.5), 0.0);
    digi.commands.throttle = thr;

    return std::fabs(eProp) < 0.1 * targetSpeed;
}

void ManeuverPrimitives::Loiter(DigiState& digi, const AircraftState& state,
                                 const FlightControlSystem& /*fcs*/,
                                 FcsState& fcsState, double maxGs) {
    double desBank = 30.0 * DTR;
    double rollErr = (desBank - state.kin.phi) * RTD;
    if (rollErr > 180.0) rollErr -= 360.0;
    else if (rollErr < -180.0) rollErr += 360.0;

    digi.commands.rStick = computeRstick(rollErr, fcsState.kr01, fcsState.tr01, digi.commands.rStick, digi.nav.dt);

    fcsState.maxRoll = 30.0;
    fcsState.maxRollDelta = 30.0;

    double alterr = ((digi.nav.holdAlt + state.kin.z) - state.kin.zdot) * 0.015;
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
    const double xft = digi.nav.trackX - state.kin.x;
    const double yft = digi.nav.trackY - state.kin.y;
    const double zft = digi.nav.trackZ - state.kin.z;  // NED: z negative up

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
        digi.commands.rStick = -state.kin.phi * RTD * 5.0 * DTR;
        digi.commands.rStick = limit(digi.commands.rStick, -1.0, 1.0);
    } else if (ata < 10.0) {
        // BVR flip-over branch (only meaningful at BVR ranges, but we
        // include it for fidelity). If droll is near ±180°, roll the
        // opposite direction and push negative G instead of rolling
        // all the way around.
        if (droll > 150.0 * DTR && digi.commands.rStick < 0.5) {
            SetRstick(droll * RTD - 180.0, digi, FlightControlSystem{}, fcsState);
            SetPstick(-ata, maxGs, CommandType::ErrorCommand, digi, state);
        } else if (droll < -150.0 * DTR && digi.commands.rStick > -0.5) {
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
    const double xft = digi.nav.trackX - state.kin.x;
    const double yft = digi.nav.trackY - state.kin.y;
    const double zft = digi.nav.trackZ - state.kin.z;

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
    // World-frame relative position
    const double xft = digi.nav.trackX - state.kin.x;
    const double yft = digi.nav.trackY - state.kin.y;
    const double zft = digi.nav.trackZ - state.kin.z;

    // Transform to body frame for azimuth
    const Matrix3& dcm = state.kin.dcm;
    const double rx = dcm.m[0][0] * xft + dcm.m[1][0] * yft + dcm.m[2][0] * zft;
    const double ry = dcm.m[0][1] * xft + dcm.m[1][1] * yft + dcm.m[2][1] * zft;

    // Roll: proportional azimuth tracker
    double rCmd = SimpleTrackAzimuth(rx, ry);
    // Clamp roll rate
    rCmd = std::max(-0.6, std::min(0.6, rCmd));
    digi.commands.rStick = rCmd;

    // Pitch: glideslope tracking with gamma-rate damping.
    //
    // The pure elevation-angle tracker (SimpleTrackElevation) has too little
    // gain at long range — at 3 NM, a 189 ft altitude error produces only
    // 0.7° of pitch command. This lets the aircraft dive well below the
    // glideslope before the pitch responds, creating a steep dive that the
    // flare can't arrest.
    //
    // Fix: add a direct altitude-error term (proportional to the altitude
    // error in feet, not the elevation angle). This gives responsive
    // glideslope tracking at any range. Combined with the gamma-rate
    // damping, this produces a stable 3° descent.
    const double distXY = std::sqrt(xft * xft + yft * yft);
    const double elErr = SimpleTrackElevation(zft, distXY, state);
    const double vt = std::max(state.kin.vt, 100.0);  // avoid /0
    const double desiredZdot = vt * std::sin(3.0 * DTR);  // ~15 ft/s at 170 kts
    const double zdotErr = state.kin.zdot - desiredZdot;  // >0 = descending too fast
    const double dampTerm = 0.5 * (zdotErr / vt);  // positive = pitch up to slow descent

    // Direct altitude-error term: zft is the NED z difference (target - current).
    // zft < 0 = aircraft below glideslope (need pitch up).
    // zft > 0 = aircraft above glideslope (need pitch down).
    // Gain: 0.002 per ft of altitude error. At 100 ft below GS, this adds
    // 0.2 to the pitch command — enough to correct quickly without oscillating.
    const double altErrTerm = -zft * 0.002;

    // Pitch command: elevation error + altitude-error term + gamma-rate damping.
    // The altitude-error term provides the primary glideslope tracking;
    // the elevation error provides fine-tuning near the threshold; the
    // damping term prevents Phugoid oscillation.
    digi.commands.pStick = std::min(0.5, std::max(elErr + altErrTerm + dampTerm, -0.5));

    // Throttle: hold approach speed (error computed in kts — both targetSpeedKts
    // and state.vcas are KCAS, so no unit conversion needed here).
    const double eProp = targetSpeedKts - state.vcas;  // kts

    if (eProp >= 150.0) {
        digi.nav.autoThrottle = 1.5;
        digi.commands.throttle = 1.5;  // burner
    } else if (eProp < -100.0) {
        digi.nav.autoThrottle = 0.0;
        digi.commands.throttle = 0.0;  // idle
    } else {
        // Proportional + integral throttle
        digi.nav.autoThrottle += eProp * 0.01 * (dt / 0.06);
        digi.nav.autoThrottle = std::max(0.0, std::min(1.5, digi.nav.autoThrottle));
        digi.commands.throttle = eProp * 0.02 + digi.nav.autoThrottle - state.vtDot * (dt / 0.06) * 0.005;
    }
    digi.commands.throttle = std::max(0.0, std::min(1.5, digi.commands.throttle));
}

void ManeuverPrimitives::VectorTrack(double desHeading, double desAlt, double desSpeed,
                                      DigiState& digi, const AircraftState& state,
                                      const FlightControlSystem& fcs,
                                      FcsState& fcsState, double maxGs, double dt) {
    HeadingAndAltitudeHold(desHeading, desAlt, digi, state, fcs, fcsState, maxGs);
    MachHold(desSpeed, state.vcas, true, digi, state, 200.0, 800.0, dt, 700.0);
}

// ===========================================================================
// Round-2 structural additions (Rec 10): 4 missing maneuver primitives.
// Direct ports of FreeFalcon mnvers.cpp / randp.cpp / wvrengage.cpp.
// ===========================================================================

// Magic number for PullToCollisionPoint fallback — 2 seconds of lead.
// (FF randp.cpp:470 MAGIC_NUMBER = 0.5; we use 2.0 which is the value
// used in the rest of FF's BFM lead-pursuit code, more appropriate for
// the typical closure rates in F4Flight's BFM scenarios.)
static constexpr double kPullToCollisionMagic = 2.0;
// Altitude-rate deadband (ft/s) — FF ALT_RATE_DEADBAND.
static constexpr double kAltRateDeadband = 1000.0;

void ManeuverPrimitives::PullToCollisionPoint(DigiState& digi,
                                              const DigiEntity& self,
                                              const DigiEntity& target,
                                              const AircraftState& as,
                                              const FlightControlSystem& /*fcs*/,
                                              FcsState& fcsState,
                                              double maxGs, bool firstFrame) {
    // --- Compute the predicted collision point ---
    // Time-to-collision: range / closure_rate. Closure > 0 means closing.
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // tc: time-to-collision (seconds). Use closure rate if positive (closing);
    // otherwise fall back to the magic-number lead.
    double tc = -1.0;
    if (rg.closure > 1.0) {
        tc = rg.range / rg.closure;
    }

    // Predicted target position at collision time
    double newX, newY, newZ;
    if (tc > 0.0) {
        newX = target.x + target.vx * tc;
        newY = target.y + target.vy * tc;
        newZ = target.z + target.vz * tc;
    } else {
        // No closure — lead by the magic number
        newX = target.x + target.vx * kPullToCollisionMagic;
        newY = target.y + target.vy * kPullToCollisionMagic;
        newZ = target.z + target.vz * kPullToCollisionMagic;
    }

    // Altitude-rate deadband (FF randp.cpp:480): if target's vertical rate
    // is within ±deadband, treat as level (don't extrapolate altitude).
    if (target.vz > kAltRateDeadband) {
        // already added target.vz * tc above; subtract the deadband portion
        // (FF adds (vz - deadband) * tc, equivalent to vz*tc - deadband*tc)
        if (tc > 0.0) newZ -= kAltRateDeadband * tc;
        else          newZ -= kAltRateDeadband * kPullToCollisionMagic;
    } else if (target.vz < -kAltRateDeadband) {
        if (tc > 0.0) newZ += kAltRateDeadband * tc;
        else          newZ += kAltRateDeadband * kPullToCollisionMagic;
    }

    // If target is far (>5 NM) and below us, hold our altitude (don't dive).
    // FF randp.cpp:507 — `if (range > 5 NM and target.z < self.z) newZ = self.z`
    if (rg.range > 5.0 * 6076.0 && target.z < self.z) {
        newZ = self.z;
    }

    if (firstFrame) {
        // First frame of the mode — set trackpoint directly.
        digi.nav.trackX = newX;
        digi.nav.trackY = newY;
        digi.nav.trackZ = newZ;
    } else {
        // Subsequent frames — SMOOTH: 0.1*new + 0.9*old.
        // This is the key behavior F4Flight was missing — without it,
        // target jitter propagates straight to AutoTrack pitch/roll.
        digi.nav.trackX = 0.1 * newX + 0.9 * digi.nav.trackX;
        digi.nav.trackY = 0.1 * newY + 0.9 * digi.nav.trackY;
        digi.nav.trackZ = 0.1 * newZ + 0.9 * digi.nav.trackZ;
    }

    // Fly to the (smoothed) trackpoint via AutoTrack.
    AutoTrack(digi, as, fcsState, maxGs);
}

void ManeuverPrimitives::OverBank(DigiState& digi,
                                  const DigiEntity& self,
                                  const DigiEntity& target,
                                  const FlightControlSystem& /*fcs*/,
                                  FcsState& fcsState,
                                  double delta, bool firstFrame) {
    // FF mnvers.cpp:920-965: skip in vertical fights (|pitch| > 45°)
    if (std::fabs(self.pitch) > 45.0 * DTR) {
        return;
    }
    // NOTE: `target` is in the signature for parity with FF (which reads
    // targetData->droll) but F4Flight computes the target roll from
    // self.roll directly. The target entity is intentionally unused.
    (void)target;

    // On first frame, compute the target roll = target.droll ± delta.
    // (FF uses targetData->droll, which is target.roll - self.roll. We
    // approximate by adding delta to self.roll directly — same effect
    // for the small bank angles typical in OverBMode.)
    if (firstFrame) {
        if (self.roll > 0.0) {
            digi.gunsJink.newRoll = self.roll + delta;
        } else {
            digi.gunsJink.newRoll = self.roll - delta;
        }
        // Wrap to [-PI, PI]
        while (digi.gunsJink.newRoll >  PI) digi.gunsJink.newRoll -= 2.0 * PI;
        while (digi.gunsJink.newRoll < -PI) digi.gunsJink.newRoll += 2.0 * PI;
    }

    // Roll error
    double eroll = digi.gunsJink.newRoll - self.roll;
    while (eroll >  PI) eroll -= 2.0 * PI;
    while (eroll < -PI) eroll += 2.0 * PI;

    SetRstick(eroll * RTD, digi, FlightControlSystem{}, fcsState);
}

bool ManeuverPrimitives::RollOutOfPlane(DigiState& digi,
                                         const DigiEntity& self,
                                         const AircraftState& as,
                                         const FlightControlSystem& /*fcs*/,
                                         FcsState& fcsState,
                                         double dt, bool firstFrame) {
    // FF mnvers.cpp:868-918
    if (firstFrame) {
        digi.nav.mnverTime = 1.0;  // 1-second maneuver

        // Roll toward vertical but limit to 30° change (FF uses 30°, was 45°)
        if (self.roll >= 0.0) {
            digi.gunsJink.newRoll = self.roll - 30.0 * DTR;
        } else {
            digi.gunsJink.newRoll = self.roll + 30.0 * DTR;
        }
    }

    // Roll error (shortest direction)
    double eroll = digi.gunsJink.newRoll - self.roll;
    while (eroll >  PI) eroll -= 2.0 * PI;
    while (eroll < -PI) eroll += 2.0 * PI;

    // Max-G pull + roll toward target bank
    SetPstick(digi.config.maxGs, digi.config.maxGs, CommandType::GCommand, digi, as);
    SetRstick(eroll * RTD, digi, FlightControlSystem{}, fcsState);

    // Decrement maneuver timer; return true while still active
    digi.nav.mnverTime -= dt;
    return digi.nav.mnverTime > 0.0;
}

void ManeuverPrimitives::WvrBugOut(DigiState& digi,
                                   const AircraftState& as,
                                   const FlightControlSystem& fcs,
                                   FcsState& fcsState,
                                   double dt) {
    // FF wvrengage.cpp:727-731: hold heading + altitude, accelerate to
    // 2× corner speed. Simplest disengage primitive.
    HeadingAndAltitudeHold(digi.nav.holdPsi, digi.nav.holdAlt,
                            digi, as, fcs, fcsState, digi.config.maxGs);
    MachHold(2.0 * digi.config.cornerSpeed, as.vcas, true,
             digi, as, 200.0, 800.0, dt, 700.0);
}

} // namespace digi
} // namespace f4flight
