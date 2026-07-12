// f4flight - steering.cpp
// AI steering with separated behaviors.

#include "f4flight/steering.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace f4flight {

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
double headingError(double setpoint, double current) noexcept {
    double err = setpoint - current;
    while (err >  PI) err -= 2.0 * PI;
    while (err < -PI) err += 2.0 * PI;
    return err;
}

double turnCompensatedG(const AircraftState& state) noexcept {
    // The G required to maintain level flight in a bank: G = 1/cos(bank).
    // For bank > 90° (inverted), cos(bank) < 0 and 1/cos would be negative —
    // which makes sqrt(tcG/maxGs) in AltitudeHold's level mode produce NaN,
    // cascading into the FCS and EOM. Clamp to always return a positive
    // value so the pitch controller never sees a negative G target. The
    // real fix for overbanking is in the roll controller (maxBank limit);
    // this is a defensive guard.
    double cosBank = std::cos(state.kin.phi);
    if (std::fabs(cosBank) < 0.1) return 1.0;        // near 90°, fall back to 1.0
    return std::fabs(1.0 / cosBank);                  // always positive
}

double computeMaxVVI_fpm(double altErr_ft) noexcept {
    const double VVI_K = 10.0;
    const double VVI_P = 0.7;
    return VVI_K * std::pow(std::fabs(altErr_ft), VVI_P);
}

double protectSpeed(double pitchCmd, const AircraftState& state, double maxGs) noexcept {
    if (state.loads.nzcgs > maxGs - 2.0 && pitchCmd > 0.0) {
        double factor = std::max(0.0, (maxGs - state.loads.nzcgs) / 2.0);
        pitchCmd *= factor;
    }
    return pitchCmd;
}

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------
double PID::update(double error, double dt) noexcept {
    if (dt <= 0.0) return 0.0;
    const double p = gains_.kp * error;
    integ_ += error * dt;
    if (integ_ > gains_.integMax) integ_ = gains_.integMax;
    if (integ_ < gains_.integMin) integ_ = gains_.integMin;
    const double i = gains_.ki * integ_;
    double d = hasPrev_ ? gains_.kd * (error - prevError_) / dt : 0.0;
    prevError_ = error;
    hasPrev_ = true;
    double out = p + i + d;
    if (out > gains_.outputMax) out = gains_.outputMax;
    if (out < gains_.outputMin) out = gains_.outputMin;
    return out;
}

// ===========================================================================
// AltitudeHold — unified vertical behavior
// ===========================================================================
AltitudeHold::AltitudeHold(double targetAlt_ft)
    : targetAlt_(targetAlt_ft) {
}

double AltitudeHold::compute(const AircraftState& state, double dt,
                              SteeringContext& ctx, PilotInput& out) {
    const double currentAlt = -state.kin.z;
    const double altErr = targetAlt_ - currentAlt;
    const double climbRate_fps = -state.kin.zdot;    // positive = climbing
    const double tcG = turnCompensatedG(state);

    // =========================================================================
    // GAMMA-COMMAND ALTITUDE CONTROL (FreeFalcon approach)
    // =========================================================================
    //
    // Instead of separate climb/level/descent modes, a single controller
    // commands a desired flight path angle (gamma) based on altitude error
    // with a lead term that naturally levels off as the aircraft approaches
    // target.
    //
    // Pitch owns altitude; throttle owns speed (via SpeedHold). No coupling.
    //
    // The controller is a direct port of FreeFalcon's DigitalBrain:
    //   AltitudeHold() -> GammaHold() -> SetPstick()
    //
    //   1. altErr = targetAlt - currentAlt
    //   2. leadErr = altErr - climbRate * leadTime   (look ahead)
    //   3. gammaCmd = leadErr * altGain              (desired flight path angle, degrees)
    //   4. gammaErr = gammaCmd - currentGamma
    //   5. elevCmd = gammaErr * speedScaledGain      (proportional, scaled by airspeed)
    //   6. integral += elevCmd * integGain           (slow correction for steady-state)
    //   7. pstick = integral + elevCmd + turnCompG   (total pitch command in G)
    //
    // The lead term (step 2) is the key: as the aircraft climbs, climbRate
    // grows, reducing leadErr, reducing gammaCmd, naturally leveling off
    // before reaching target. No VVI cap or mode switching needed.

    // --- Step 1-2: altitude error with lead term ---
    // The lead time controls how far ahead we look. A longer lead makes the
    // controller start leveling off earlier (smoother but slower capture).
    // FreeFalcon uses ZDelta (climb rate in ft/s) directly, which is
    // equivalent to a 1-second lead. We use 2 seconds for a balance of
    // smoothness and responsiveness.
    const double leadTime_s = 2.0;
    double leadErr = altErr - climbRate_fps * leadTime_s;

    // --- Step 3: desired flight path angle (degrees) ---
    // altGain converts altitude error to a desired gamma. FreeFalcon uses
    // 0.015 (degrees per foot), but that's for an AI that can tolerate large
    // excursions. We use 0.01 for smoother control — at 5000 ft error -> 50 deg
    // gamma (clamped to 30), at 500 ft -> 5 deg, at 50 ft -> 0.5 deg.
    const double altGain = 0.01;
    double gammaCmd = leadErr * altGain;
    // Clamp gamma to a reasonable range (FreeFalcon uses +/- 60 deg)
    gammaCmd = limit(gammaCmd, -30.0, 30.0);

    // --- Step 4: gamma error ---
    // state.kin.gmma is the current flight path angle in radians.
    double gammaErr_deg = gammaCmd - state.kin.gmma * RTD;

    // --- Step 5: elevator command (proportional, speed-scaled) ---
    // FreeFalcon: elevCmd = gammaErr * 0.25 * KIAS / 350
    // The speed scaling makes the gain lower at low speed (where control
    // authority is higher) and higher at high speed (where authority is lower).
    // We reduce the gain from 0.25 to 0.15 for smoother autopilot response.
    double speedScale = 0.15 * state.vcas / 350.0;
    if (speedScale < 0.05) speedScale = 0.05;  // floor to prevent zero gain
    double elevCmd = gammaErr_deg * speedScale;

    // Turn compensation: divide by cos(bank) so level flight in a turn
    // gets the right pitch. Only when bank < 45 deg (cos > 0.7).
    if (std::fabs(state.kin.phi) < 45.0 * DTR) {
        double cosphi = std::cos(state.kin.phi);
        if (cosphi > 0.1) elevCmd /= cosphi;
    }

    // Note: FreeFalcon squares elevCmd for a nonlinear response, but that
    // amplifies oscillation in our smoother autopilot context. We keep it
    // linear for more predictable behavior.

    // --- Step 6: integral term (slow correction for steady-state error) ---
    // FreeFalcon: gammaHoldIError += 0.0025 * elevCmd, clamped to [-1, 1]
    gammaIError_ += 0.0025 * elevCmd * (dt * 60.0);  // scale by frame rate
    gammaIError_ = limit(gammaIError_, -1.0, 1.0);

    // --- Step 7: total pitch command (in G) ---
    // gammaCmd = integral + proportional + turn-compensated level G
    // The 1/cos(bank) term ensures the aircraft maintains 1 G (level) in
    // turns — without it, the aircraft would lose altitude in a turn.
    double gCmd = gammaIError_ + elevCmd + tcG;

    // Clamp to a safe G range (FreeFalcon uses [-2, 6.5])
    gCmd = limit(gCmd, -2.0, std::min(6.5, ctx.maxGs));

    // Convert G command to pstick [-1, +1].
    // The FCS interprets pstick as a G command (via stick shaping).
    // sqrt mapping: pstick = sqrt((gCmd - 1) / (maxGs - 1)) for gCmd > 1,
    //               pstick = -sqrt((1 - gCmd) / 4) for gCmd < 1.
    // This matches FreeFalcon's SetPstick shaping.
    double pstick;
    if (gCmd > 1.0) {
        double denom = ctx.maxGs - 1.0;
        if (denom < 0.1) denom = 0.1;
        pstick = std::sqrt((gCmd - 1.0) / denom);
    } else {
        pstick = -std::sqrt((1.0 - gCmd) / 4.0);
    }

    // Low-speed authority fade: reduce pstick at low airspeed to prevent
    // stalling. FreeFalcon: stickFact = 0.5 + (KIAS - 150) / 300, clamped [0, 1]
    double stickFact = 0.5 + (state.vcas - 150.0) / 300.0;
    stickFact = limit(stickFact, 0.0, 1.0);
    pstick *= stickFact;

    // Smooth the command (low-pass filter) to prevent jerky inputs.
    // FreeFalcon: pStick = 0.2 * pStick + 0.8 * stickCmd
    // We apply this via the ctx.pitchPID's internal state is NOT used here;
    // the gamma controller has its own integral. We just return the command.
    // The SteeringController will apply it directly.

    pstick = protectSpeed(pstick, state, ctx.maxGs);

    // --- Throttle: do NOT set out.throttle ---
    // In the FreeFalcon approach, the throttle is purely speed-based and
    // is handled by the SpeedHold behavior (via the SteeringController's
    // sentinel mechanism). AltitudeHold owns pitch only.
    //
    // (out.throttle is left at the sentinel value set by SteeringController,
    // which tells it to let SpeedHold run.)

    // Reset the level-flight PID (unused in gamma approach, but reset to
    // prevent windup if it was used previously).
    ctx.pitchPID.reset();

    return limit(pstick, -1.0, 1.0);
}

// ===========================================================================
// HeadingHold
// ===========================================================================
double HeadingHold::compute(const AircraftState& state, double dt,
                             SteeringContext& ctx) {
    double err = headingError(heading_, state.kin.psi);
    double maxBank = ctx.maxBank_deg * DTR;
    double desiredBank = limit(err * 2.0, -maxBank, maxBank);
    return limit((desiredBank - state.kin.phi) * 2.0, -1.0, 1.0);
}

// ===========================================================================
// SteerToWaypoint
// ===========================================================================
double SteerToWaypoint::compute(const AircraftState& state, double dt,
                                 SteeringContext& ctx) {
    if (wps_.empty() || curWp_ >= wps_.size()) return 0.0;

    const Vec3& wp = wps_[curWp_];
    double dx = wp.x - state.kin.x;
    double dy = wp.y - state.kin.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < captureRadius_) {
        ++curWp_;
        if (curWp_ >= wps_.size()) return 0.0;
    }

    double desiredHeading = std::atan2(dy, dx);
    double err = headingError(desiredHeading, state.kin.psi);
    double maxBank = ctx.maxBank_deg * DTR;
    double desiredBank = limit(err * 2.0, -maxBank, maxBank);
    return limit((desiredBank - state.kin.phi) * 2.0, -1.0, 1.0);
}

// ===========================================================================
// SpeedHold
// ===========================================================================
double SpeedHold::compute(const AircraftState& state, double dt,
                           SteeringContext& ctx) {
    
    // MachHold-style throttle control (port of FreeFalcon DigitalBrain).
    //
    // FreeFalcon: thr = (eProp + 100) * 0.008 + autoThrottle
    //
    // The +100 bias means at zero speed error, the throttle is 0.8 (near MIL).
    // This is critical — without it, the throttle would be 0 at zero error,
    // and the aircraft would decelerate. The bias provides the cruise power
    // setting; the proportional term corrects for speed error; the integral
    // handles steady-state drag changes.
    //
    // Three-zone response for big errors (bang-bang), linear for small errors.
    double eProp = target_ - state.vcas;  // speed error (kts), + = too slow
    double thr;

    if (eProp >= 150.0) {
        // Big underspeed — full afterburner
        thr = 1.5;
        ctx.throttlePID.reset();
    } else if (eProp < -100.0) {
        // Big overspeed — idle
        thr = 0.0;
        ctx.throttlePID.reset();
    } else {
        // Linear proportional + integral + vtDot damping
        // FreeFalcon: thr = (eProp + 100) * 0.008 + autoThrottle
        //   = 0.8 + eProp * 0.008 + autoThrottle
        //
        // We replicate this with:
        //   - cruise bias (0.5 — roughly MIL for most fighters at altitude)
        //   - proportional: kp * eProp (via throttlePID)
        //   - integral: throttlePID integral (handles steady-state drag)
        //   - vtDot damping: -vtDot * gain (prevents overshoot)
        const double cruiseBias = 0.5;  // baseline throttle at zero error

        double vtDot_kts = state.aero.xwaero * FTPSEC_TO_KNOTS;
        double pidOut = ctx.throttlePID.update(eProp, dt);
        double damping = -vtDot_kts * dt * 0.5;

        thr = cruiseBias + pidOut + damping;
    }

    return limit(thr, 0.0, 1.5);
}

// ===========================================================================
// SteeringController
// ===========================================================================
SteeringController::SteeringController() {
    PIDGains pitchG;
    pitchG.kp = 0.001; pitchG.ki = 0.0001; pitchG.kd = 0.001;
    pitchG.outputMin = -0.4; pitchG.outputMax = 0.4;
    pitchG.integMin = -0.2; pitchG.integMax = 0.2;
    ctx_.pitchPID = PID(pitchG);

    PIDGains rollG;
    rollG.kp = 2.0; rollG.ki = 0.2; rollG.kd = 0.5;
    rollG.outputMin = -1.0; rollG.outputMax = 1.0;
    rollG.integMin = -0.3; rollG.integMax = 0.3;
    ctx_.rollPID = PID(rollG);

    // Throttle PID gains for the MachHold-style speed controller.
    // The PID output is ADDED to a cruise bias (0.5) in SpeedHold::compute,
    // so the PID needs to be able to output negative values (to reduce
    // throttle below the bias when overspeeding). outputMin=-1.0 allows
    // the PID to subtract up to 1.0 from the bias, giving a final throttle
    // range of [0, 1.5] after clamping.
    PIDGains throttleG;
    throttleG.kp = 0.02; throttleG.ki = 0.01; throttleG.kd = 0.005;
    throttleG.outputMin = -1.0; throttleG.outputMax = 1.0;
    throttleG.integMin = -1.0; throttleG.integMax = 1.0;
    ctx_.throttlePID = PID(throttleG);

    PIDGains yawG;
    yawG.kp = 2.0; yawG.ki = 0.0; yawG.kd = 0.5;
    yawG.outputMin = -1.0; yawG.outputMax = 1.0;
    yawG.integMin = -0.2; yawG.integMax = 0.2;
    ctx_.yawPID = PID(yawG);
}

PilotInput SteeringController::compute(const AircraftState& state, double dt, double groundZ) {
    PilotInput out = ctx_.manual;

    // Vertical behavior: computes pstick, may set throttle.
    //
    // We use a sentinel value to detect whether the vertical behavior
    // actually controlled throttle this frame. Comparing to the manual
    // throttle value is unreliable: AltitudeHold in climb mode legitimately
    // sets throttle to 0 when the VVI cap fully reduces it, which would be
    // indistinguishable from "didn't touch it" if we compared against the
    // manual default (also 0). That bug caused SpeedHold to override the
    // climb-mode throttle with its wound-up integral, producing massive
    // altitude overshoots.
    constexpr double THROTTLE_UNSET = -1.0;
    const double manualThrottle = out.throttle;
    out.throttle = THROTTLE_UNSET;

    bool verticalControlsThrottle = false;
    if (vert_) {
        out.pstick = vert_->compute(state, dt, ctx_, out);
        verticalControlsThrottle = (out.throttle != THROTTLE_UNSET);
    }

    if (verticalControlsThrottle) {
        // Climb/descent mode — vertical behavior owns throttle.
    } else if (thrott_) {
        // Level mode (or no vertical behavior) — throttle behavior runs.
        out.throttle = thrott_->compute(state, dt, ctx_);
    } else {
        // No behavior controls throttle — fall back to manual.
        out.throttle = manualThrottle;
    }

    // Horizontal behavior: computes rstick
    if (horiz_) {
        out.rstick = horiz_->compute(state, dt, ctx_);
    }

    // Yaw coordination (always)
    out.ypedal = ctx_.yawPID.update(0.0 - state.aero.beta_deg, dt);

    return out;
}

void SteeringController::reset() noexcept {
    ctx_.pitchPID.reset();
    ctx_.rollPID.reset();
    ctx_.throttlePID.reset();
    ctx_.yawPID.reset();
}

} // namespace f4flight
