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
    double cosBank = std::cos(state.kin.phi);
    return (std::fabs(cosBank) > 0.1) ? (1.0 / cosBank) : 1.0;
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
AltitudeHold::AltitudeHold(
    double targetAlt_ft, double cruiseSpeed_kts,
    double climbSpeed_kts, double climbMach, double climbPower,
    double descentSpeed_kts, double descentMach, double descentPower,
    double levelBand_ft)

    : targetAlt_(targetAlt_ft)
    , cruiseSpeed_(cruiseSpeed_kts)
    , climbSpeed_(climbSpeed_kts > 0 ? climbSpeed_kts : cruiseSpeed_kts)
    , climbMach_(climbMach)
    , climbPower_(climbPower)
    , descentPower_(descentPower)
    , descentSpeed_(descentSpeed_kts > 0 ? descentSpeed_kts : cruiseSpeed_kts)
    , descentMach_(descentMach)
    , levelBand_(levelBand_ft) {}

double AltitudeHold::compute(const AircraftState& state, double dt,
                              SteeringContext& ctx, PilotInput& out) {
    const double currentAlt = -state.kin.z;
    const double altErr = targetAlt_ - currentAlt;
    const double climbRate_fps = -state.kin.zdot;
    const double climbRate_fpm = climbRate_fps * 60.0;
    const double tcG = turnCompensatedG(state);

    // --- Determine mode based on altitude error ---
    const bool isClimb = altErr > levelBand_;
    const bool isDescent = altErr < -levelBand_;

    if (isClimb) {
        // ===== CLIMB: throttle for altitude (VVI-capped), pitch for speed =====
        const double maxVVI_fpm = computeMaxVVI_fpm(altErr);
        const double cruisePower = 0.30;

        // Throttle: reduce as VVI exceeds cap
        double vviExcess = climbRate_fpm - maxVVI_fpm;
        if (vviExcess > 0.0 && maxVVI_fpm > 1.0) {
            double excessFrac = limit(vviExcess / maxVVI_fpm, 0.0, 1.0);
            out.throttle = climbPower_ * (1.0 - excessFrac);
        } else {
            out.throttle = climbPower_;
        }

        // Pitch: hold climb speed schedule
        double targetSpeed = (state.mach < climbMach_) ? climbSpeed_ : state.vcas;
        double speedErr = targetSpeed - state.vcas;

        double gTarget = 1.15;
        if (speedErr > 30.0)
            gTarget -= limit((speedErr - 30.0) * 0.01, 0.0, 0.3);
        if (speedErr < -30.0)
            gTarget += limit(-(speedErr + 30.0) * 0.005, 0.0, 0.2);

        gTarget = limit(gTarget, 0.0, ctx.maxGs);
        double pitchCmd = std::sqrt(gTarget / ctx.maxGs);
        pitchCmd = protectSpeed(pitchCmd, state, ctx.maxGs);

        // Reset level-flight PID to prevent windup
        ctx.pitchPID.reset();

        return limit(pitchCmd, -1.0, 1.0);

    } else if (isDescent) {
        // ===== DESCENT: throttle for cruise speed, pitch for descent speed + VVI =====
        const double maxVVI_fpm = computeMaxVVI_fpm(altErr);
        const double maxVVI_fps = maxVVI_fpm / 60.0;

        // Throttle: chase cruise speed
        out.throttle = ctx.throttlePID.update(cruiseSpeed_ - state.vcas, dt);

        // Pitch: descent speed + VVI cap
        double targetSpeed = (state.vcas > descentSpeed_ && state.mach > descentMach_)
            ? state.vcas : descentSpeed_;
        double speedErr = targetSpeed - state.vcas;

        double gTarget = 0.9;

        // VVI cap: increase G if descending too fast
        if (-climbRate_fps > maxVVI_fps && maxVVI_fps > 0.01) {
            gTarget += limit((-climbRate_fps - maxVVI_fps) * 0.01, 0.0, 0.8);
        }
        if (speedErr < -10.0)
            gTarget += limit(-(speedErr + 10.0) * 0.01, 0.0, 0.5);
        if (speedErr > 10.0)
            gTarget -= limit((speedErr - 10.0) * 0.01, 0.0, 0.2);

        gTarget = limit(gTarget, 0.0, ctx.maxGs);
        double pitchCmd = std::sqrt(gTarget / ctx.maxGs);
        pitchCmd = protectSpeed(pitchCmd, state, ctx.maxGs);

        ctx.pitchPID.reset();

        return limit(pitchCmd, -1.0, 1.0);

    } else {
        // ===== LEVEL: pitch for altitude, throttle for speed =====
        double gErr = tcG - state.loads.nzcgs;
        double gCorrection = limit(gErr * 0.05, -0.2, 0.2);

        double altErrClamped = limit(altErr, -500.0, 500.0);
        double pitchCmd = ctx.pitchPID.update(altErrClamped, dt);
        pitchCmd += limit(-climbRate_fps * 0.002, -0.15, 0.15);
        pitchCmd += std::sqrt(tcG / ctx.maxGs) + gCorrection;
        pitchCmd = protectSpeed(pitchCmd, state, ctx.maxGs);

        // Throttle: let the throttle behavior handle it (or do it here if none)
        // The controller checks if we're in level mode and lets SpeedHold run.

        return limit(pitchCmd, -1.0, 1.0);
    }
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
    return ctx.throttlePID.update(target_ - state.vcas, dt);
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

    PIDGains throttleG;
    throttleG.kp = 0.02; throttleG.ki = 0.03; throttleG.kd = 0.01;
    throttleG.outputMin = 0.0; throttleG.outputMax = 1.5;
    throttleG.integMin = -0.5; throttleG.integMax = 30.0;
    ctx_.throttlePID = PID(throttleG);

    PIDGains yawG;
    yawG.kp = 2.0; yawG.ki = 0.0; yawG.kd = 0.5;
    yawG.outputMin = -1.0; yawG.outputMax = 1.0;
    yawG.integMin = -0.2; yawG.integMax = 0.2;
    ctx_.yawPID = PID(yawG);
}

PilotInput SteeringController::compute(const AircraftState& state, double dt, double groundZ) {
    PilotInput out = ctx_.manual;

    // Vertical behavior: computes pstick, may set throttle
    bool verticalControlsThrottle = false;
    if (vert_) {
        // Save throttle before vertical behavior runs
        double throttleBefore = out.throttle;
        out.pstick = vert_->compute(state, dt, ctx_, out);
        // If the vertical behavior changed the throttle, it's controlling it
        // (climb/descent mode). If not, we let the throttle behavior run.
        verticalControlsThrottle = (out.throttle != throttleBefore);
    }

    // Horizontal behavior: computes rstick
    if (horiz_) {
        out.rstick = horiz_->compute(state, dt, ctx_);
    }

    // Throttle behavior: only runs if vertical behavior didn't control throttle
    if (thrott_ && !verticalControlsThrottle) {
        out.throttle = thrott_->compute(state, dt, ctx_);
    }

    // Yaw coordination (always)
    out.ypedal = ctx_.yawPID.update(0.0 - state.aero.beta_deg, dt);

    return out;
}

} // namespace f4flight
