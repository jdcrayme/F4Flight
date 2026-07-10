// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// steering.cpp
//
// AI steering / autopilot implementation.

#include "f4flight/steering.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace f4flight {

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------
double PID::update(double error, double dt) noexcept {
    if (dt <= 0.0) return 0.0;

    // Proportional
    const double p = gains_.kp * error;

    // Integral (with anti-windup clamping)
    integ_ += error * dt;
    if (integ_ > gains_.integMax) integ_ = gains_.integMax;
    if (integ_ < gains_.integMin) integ_ = gains_.integMin;
    const double i = gains_.ki * integ_;

    // Derivative
    double d = 0.0;
    if (hasPrev_) {
        d = gains_.kd * (error - prevError_) / dt;
    }
    prevError_ = error;
    hasPrev_ = true;

    double out = p + i + d;
    if (out > gains_.outputMax) out = gains_.outputMax;
    if (out < gains_.outputMin) out = gains_.outputMin;
    return out;
}

// ---------------------------------------------------------------------------
// SteeringController
// ---------------------------------------------------------------------------
SteeringController::SteeringController() {
    // PID gains tuned across multiple aircraft types (F-16, A-10, B-52,
    // C-130, Su-27). These are general-purpose values; the controller
    // architecture (VVI cap, G-command mapping, turn compensation) handles
    // aircraft-specific differences.

    PIDGains pitchG;
    pitchG.kp = 0.001;       // altitude error (ft) -> pitch stick
    pitchG.ki = 0.0001;
    pitchG.kd = 0.001;
    pitchG.outputMin = -0.4;
    pitchG.outputMax =  0.4;
    pitchG.integMin  = -0.2;
    pitchG.integMax  =  0.2;
    pitchPID_ = PID(pitchG);

    PIDGains rollG;
    rollG.kp = 2.0;          // heading error (rad) -> roll stick
    rollG.ki = 0.2;
    rollG.kd = 0.5;
    rollG.outputMin = -1.0;
    rollG.outputMax =  1.0;
    rollG.integMin  = -0.3;
    rollG.integMax  =  0.3;
    rollPID_ = PID(rollG);

    // Throttle PID: needs to be aggressive enough to drive the aircraft
    // to the target speed. The integral range is wide (0 to 1.0) so it
    // can accumulate enough throttle to overcome drag at any speed.
    PIDGains throttleG;
    throttleG.kp = 0.02;     // speed error (kts) -> throttle
    throttleG.ki = 0.03;     // moderate integral — too high causes overshoot
    throttleG.kd = 0.01;
    throttleG.outputMin = 0.0;
    throttleG.outputMax = 1.5;
    throttleG.integMin  = -0.5;
    throttleG.integMax  =  30.0;  // wide integral range (ki * integ = 0.03 * 30 = 0.9 max)
    throttlePID_ = PID(throttleG);

    PIDGains yawG;
    yawG.kp = 2.0;
    yawG.ki = 0.0;
    yawG.kd = 0.5;
    yawG.outputMin = -1.0;
    yawG.outputMax =  1.0;
    yawG.integMin  = -0.2;
    yawG.integMax  =  0.2;
    yawPID_ = PID(yawG);

    PIDGains altG;
    altG.kp = 0.005;
    altG.ki = 0.0;
    altG.kd = 0.01;
    altG.outputMin = -1.0;
    altG.outputMax =  1.0;
    altPID_ = PID(altG);

    // Speed PID (used in climb/descent: pitch for speed)
    PIDGains speedG;
    speedG.kp = 0.01;
    speedG.ki = 0.001;
    speedG.kd = 0.005;
    speedG.outputMin = -0.5;
    speedG.outputMax =  0.5;
    speedG.integMin  = -0.3;
    speedG.integMax  =  0.3;
    speedPID_ = PID(speedG);
}

// Compute the heading error (setpoint - current), wrapped to [-pi, +pi].
static double headingError(double setpoint_rad, double current_rad) noexcept {
    double err = setpoint_rad - current_rad;
    while (err >  PI) err -= 2.0 * PI;
    while (err < -PI) err += 2.0 * PI;
    return err;
}

// Distance between two world points (NED, ft).
static double distance2D(Vec3 a, Vec3 b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

[[maybe_unused]] static double distance3D(Vec3 a, Vec3 b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Prevent the pitch command from commanding a speed below the stall speed.
double SteeringController::protectSpeed(double pitchCmd,
                                        const AircraftState& state) noexcept {
    // G protection: if we're already pulling high G, reduce further pitch-up
    // to avoid over-G. The FCS AOA limiter already prevents stall, so we
    // don't need separate stall-speed protection here.
    if (state.loads.nzcgs > 7.0 && pitchCmd > 0.0) {
        const double factor = std::max(0.0, (maxGs_ - state.loads.nzcgs) / 2.0);
        pitchCmd *= factor;
    }

    return pitchCmd;
}

// ---------------------------------------------------------------------------
// Heading + altitude + speed hold with VVI-capped level-off
//
// CLIMB:
//   - Throttle = climbPower, REDUCED by VVI cap (throttle goes down as VVI
//     exceeds the cap, not G). This bleeds excess thrust to control climb
//     rate without causing speed runaway.
//   - Pitch for speed: hold climb speed schedule. G stays at climb setting.
//
// DESCENT:
//   - Throttle chases CRUISE speed (the level-flight target). This keeps the
//     engine spooled for the eventual level-off.
//   - Pitch chases descent speed schedule + VVI cap. As the aircraft
//     approaches target altitude, the VVI cap increases pitch (toward 1.0 G),
//     which slows the descent. As pitch increases, speed drops, and the
//     throttle (chasing cruise speed) adds power to hold speed.
//
// LEVEL FLIGHT:
//   - Pitch for altitude (with turn compensation)
//   - Throttle for speed
//
// VVI cap: maxVVI = K * |altErr|^P  (power law, P < 1)
//   Power < 1 means the cap is lenient far from target (allowing high climb
//   rates) but tight near the target (forcing level-off).
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeHeadingAltitude(const AircraftState& state,
                                                       double dt) {
    PilotInput out = manual_;

    // --- Roll: hold heading ---
    double targetHeading = goal_.hasHeading ? goal_.heading_rad : state.kin.psi;
    const double hErr = headingError(targetHeading, state.kin.psi);
    const double desiredBank_rad = limit(hErr * 2.0,
                                          -maxBank_deg_ * DTR,
                                          maxBank_deg_ * DTR);
    const double bankErr = desiredBank_rad - state.kin.phi;
    out.rstick = limit(bankErr * 2.0, -1.0, 1.0);

    // --- Current state ---
    const double currentAlt_ft = -state.kin.z;
    const double targetAlt_ft = goal_.hasAltitude ? goal_.altitude_ft : currentAlt_ft;
    const double altErr = targetAlt_ft - currentAlt_ft;
    const double levelBand = goal_.levelBand_ft;
    const double climbRate_fps = -state.kin.zdot;  // positive = climbing
    const double climbRate_fpm = climbRate_fps * 60.0;

    // --- Turn-compensated level-flight G ---
    const double bank_rad = state.kin.phi;
    const double cosBank = std::cos(bank_rad);
    const double turnCompG = (std::fabs(cosBank) > 0.1) ? (1.0 / cosBank) : 1.0;

    // --- VVI cap (power law) ---
    const double VVI_K = 10.0;
    const double VVI_P = 0.7;
    const double absAltErr = std::fabs(altErr);
    const double maxVVI_fpm = VVI_K * std::pow(absAltErr, VVI_P);
    const double maxVVI_fps = maxVVI_fpm / 60.0;

    // --- Determine flight phase ---
    const bool isClimb = altErr > levelBand;
    const bool isDescent = altErr < -levelBand;

    // --- Pitch and throttle logic ---

    if (isClimb) {
        // ===== CLIMB =====
        // Throttle: start at climbPower, REDUCE as VVI exceeds the cap.
        // This controls the climb rate by bleeding excess thrust, NOT by
        // reducing G (which would cause speed runaway).
        const double cruisePower = 0.30;
        double vviExcess = climbRate_fpm - maxVVI_fpm;
        if (vviExcess > 0.0 && maxVVI_fpm > 1.0) {
            double excessFrac = limit(vviExcess / maxVVI_fpm, 0.0, 1.0);
            out.throttle = goal_.climbPower * (1.0 - excessFrac)
                         + cruisePower * excessFrac;
        } else {
            out.throttle = goal_.climbPower;
        }

        // Pitch for speed: hold the climb speed schedule.
        // G stays at a moderate climb setting. If too slow, reduce G to
        // dive slightly and accelerate. If too fast, increase G to climb
        // steeper and decelerate.
        double targetSpeed_kts = (state.mach < goal_.climbMach)
            ? goal_.climbVcas_kts
            : state.vcas;
        const double speedErr = targetSpeed_kts - state.vcas;

        double gTarget = 1.15;  // moderate climb G

        // Speed protection: if too slow, reduce G to accelerate
        const double spdDb = 30.0;
        if (speedErr > spdDb) {
            gTarget -= limit((speedErr - spdDb) * 0.01, 0.0, 0.3);
        }
        // If too fast, increase G slightly to bleed speed
        if (speedErr < -spdDb) {
            gTarget += limit(-(speedErr + spdDb) * 0.005, 0.0, 0.2);
        }

        gTarget = limit(gTarget, 0.0, maxGs_);
        double pitchCmd = std::sqrt(gTarget / maxGs_);
        pitchCmd = protectSpeed(pitchCmd, state);
        out.pstick = limit(pitchCmd, -1.0, 1.0);

    } else if (isDescent) {
        // ===== DESCENT =====
        // Throttle: chase CRUISE speed (the level-flight target). This keeps
        // the engine spooled and prevents overspeed during the eventual
        // level-off. The throttle will add power as pitch increases during
        // level-off and speed drops.
        if (goal_.hasSpeed) {
            const double speedErr = goal_.speed_kts - state.vcas;
            out.throttle = throttlePID_.update(speedErr, dt);
        } else {
            out.throttle = goal_.descentPower;
        }

        // Pitch: chase descent speed schedule + VVI cap.
        // Base descent G: 0.9 (slightly nose-down).
        // VVI cap: if descending faster than maxVVI, INCREASE pitch (toward
        // 1.0 G) to level off and kill the descent rate. This naturally
        // slows the aircraft, and the throttle (chasing cruise speed) will
        // add power to hold speed.
        double targetSpeed_kts = (state.vcas > goal_.descentVcas_kts && state.mach > goal_.descentMach)
            ? state.vcas
            : goal_.descentVcas_kts;
        const double speedErr = targetSpeed_kts - state.vcas;

        double gTarget = 0.9;

        // VVI cap: increase pitch (G) if descending too fast
        if (-climbRate_fps > maxVVI_fps && maxVVI_fps > 0.01) {
            double vviExcess_fps = -climbRate_fps - maxVVI_fps;
            gTarget += limit(vviExcess_fps * 0.01, 0.0, 0.8);
        }

        // Speed protection: if too fast, increase G to slow down
        const double spdDb = 10.0;
        if (speedErr < -spdDb) {
            gTarget += limit(-(speedErr + spdDb) * 0.01, 0.0, 0.5);
        }
        // If too slow, reduce G to descend steeper
        if (speedErr > spdDb) {
            gTarget -= limit((speedErr - spdDb) * 0.01, 0.0, 0.2);
        }

        gTarget = limit(gTarget, 0.0, maxGs_);
        double pitchCmd = std::sqrt(gTarget / maxGs_);
        pitchCmd = protectSpeed(pitchCmd, state);
        out.pstick = limit(pitchCmd, -1.0, 1.0);

    } else {
        // ===== LEVEL FLIGHT =====
        // Pitch for altitude, throttle for speed.
        const double gErr = turnCompG - state.loads.nzcgs;
        const double gGain = 0.05;
        const double gCorrection = limit(gErr * gGain, -0.2, 0.2);

        double altErrClamped = limit(altErr, -500.0, 500.0);
        double pitchCmd = pitchPID_.update(altErrClamped, dt);

        // Climb-rate damping
        const double climbErr = 0.0 - climbRate_fps;
        pitchCmd += limit(climbErr * 0.002, -0.15, 0.15);

        // Feed-forward: turn-compensated G
        const double pstick_ff = std::sqrt(turnCompG / maxGs_);
        pitchCmd += pstick_ff + gCorrection;
        pitchCmd = protectSpeed(pitchCmd, state);
        out.pstick = limit(pitchCmd, -1.0, 1.0);

        // Throttle for speed — always use the target speed, don't hold
        // current speed (that prevents the PID from correcting overshoots).
        if (goal_.hasSpeed) {
            const double speedErr = goal_.speed_kts - state.vcas;
            out.throttle = throttlePID_.update(speedErr, dt);
        } else {
            out.throttle = manual_.throttle;
        }
    }

    // --- Yaw: coordinate turns ---
    const double betaErr = 0.0 - state.aero.beta_deg;
    out.ypedal = yawPID_.update(betaErr, dt);

    return out;
}

// ---------------------------------------------------------------------------
// Waypoint following
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeWaypoint(const AircraftState& state,
                                                double dt, double groundZ) {
    // If we have a goal waypoint, use it; otherwise use the current waypoint
    // in the sequence.
    Vec3 target;
    if (goal_.hasWaypoint) {
        target = goal_.waypoint;
    } else if (!waypoints_.empty() && curWp_ < waypoints_.size()) {
        target = waypoints_[curWp_];
        // Check for waypoint capture
        const double dist = distance2D(Vec3{state.kin.x, state.kin.y, state.kin.z}, target);
        if (dist < wpCapture_ft_) {
            ++curWp_;
            if (curWp_ >= waypoints_.size()) curWp_ = 0;  // loop
            // Update target to the new waypoint
            if (curWp_ < waypoints_.size()) target = waypoints_[curWp_];
        }
    } else {
        // No waypoints -- fall back to heading hold
        return computeHeadingAltitude(state, dt);
    }

    // Compute the desired heading to the waypoint
    const double dx = target.x - state.kin.x;
    const double dy = target.y - state.kin.y;
    const double desiredHeading = std::atan2(dy, dx);

    // Compute the desired altitude (use the waypoint's altitude, or hold
    // current if the waypoint is at z=0)
    const double wpAlt_ft = -target.z;
    const double currentAlt_ft = -state.kin.z;

    // Build a synthetic goal for heading/altitude hold
    SteeringGoal g = goal_;
    g.hasHeading = true;
    g.heading_rad = desiredHeading;
    if (wpAlt_ft > 100.0) {
        g.hasAltitude = true;
        g.altitude_ft = wpAlt_ft;
    } else {
        g.hasAltitude = false;
    }

    // Temporarily swap in the synthetic goal and recurse into heading-altitude
    SteeringGoal savedGoal = goal_;
    goal_ = g;
    PilotInput out = computeHeadingAltitude(state, dt);
    goal_ = savedGoal;

    // Add a "approach waypoint" behavior: if we're close to the waypoint,
    // start turning toward the next one. This is implicit in the heading
    // computation since the heading will naturally lead the turn.

    (void)groundZ;
    (void)currentAlt_ft;
    return out;
}

// ---------------------------------------------------------------------------
// Approach (ILS-like)
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeApproach(const AircraftState& state,
                                                double dt) {
    PilotInput out = manual_;

    // Aim at the runway threshold, then fly down the glideslope.
    const Vec3& threshold = goal_.runwayThreshold;
    const double dx = threshold.x - state.kin.x;
    const double dy = threshold.y - state.kin.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    // Localizer: heading to the threshold
    const double desiredHeading = (dist > 100.0)
        ? std::atan2(dy, dx)
        : goal_.runwayHeading_rad;

    const double hErr = headingError(desiredHeading, state.kin.psi);
    const double desiredBank = limit(hErr * 1.5, -25.0 * DTR, 25.0 * DTR);
    out.rstick = limit((desiredBank - state.kin.phi) * 2.0, -1.0, 1.0);

    // Glideslope: 3-degree descent
    const double gsAngle = 3.0 * DTR;
    const double desiredAlt_ft = (dist > 50.0) ? dist * std::tan(gsAngle) : 0.0;
    const double altErr = desiredAlt_ft - (-state.kin.z);
    double pitchCmd = pitchPID_.update(altErr, dt);
    // Add a small feed-forward for the descent
    pitchCmd -= 0.05;  // slight nose-down bias
    out.pstick = limit(pitchCmd, -1.0, 1.0);

    // Speed: approach speed is typically 130-150 kts for an F-16
    if (goal_.hasSpeed) {
        const double speedErr = goal_.speed_kts - state.vcas;
        out.throttle = throttlePID_.update(speedErr, dt);
    } else {
        out.throttle = 0.3;  // default approach power
    }

    // Lower the gear for approach
    out.gearHandle = 1.0;

    return out;
}

// ---------------------------------------------------------------------------
// Formation flying
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeFormation(const AircraftState& state,
                                                 double dt) {
    if (!goal_.hasFormationLead) {
        return computeHeadingAltitude(state, dt);
    }

    // Compute the desired position: lead position + formation offset
    // (offset is in body axes of THIS aircraft, roughly -- we approximate
    // by rotating into the world frame using the current heading).
    const double ch = std::cos(state.kin.psi);
    const double sh = std::sin(state.kin.psi);
    const Vec3 desiredWorld{
        goal_.leadPosition.x + goal_.formationOffset.x * ch - goal_.formationOffset.y * sh,
        goal_.leadPosition.y + goal_.formationOffset.x * sh + goal_.formationOffset.y * ch,
        goal_.leadPosition.z + goal_.formationOffset.z
    };

    // Position error
    const Vec3 posErr{
        desiredWorld.x - state.kin.x,
        desiredWorld.y - state.kin.y,
        desiredWorld.z - state.kin.z
    };

    PilotInput out = manual_;

    // Lateral: turn toward the desired position
    const double desiredHeading = std::atan2(posErr.y, posErr.x);
    const double hErr = headingError(desiredHeading, state.kin.psi);
    const double desiredBank = limit(hErr * 2.0, -45.0 * DTR, 45.0 * DTR);
    out.rstick = limit((desiredBank - state.kin.phi) * 2.0, -1.0, 1.0);

    // Longitudinal: match the lead's speed + close the distance
    const double fwdErr = posErr.x * ch + posErr.y * sh;  // forward component
    const double leadSpeed = goal_.leadVelocity.norm();
    const double desiredSpeed_kts = leadSpeed * FTPSEC_TO_KNOTS + fwdErr * 0.01;
    const double speedErr = desiredSpeed_kts - state.vcas;
    out.throttle = throttlePID_.update(speedErr, dt);

    // Vertical: hold the desired altitude
    const double altErr = -posErr.z;
    double pitchCmd = pitchPID_.update(altErr, dt);
    pitchCmd = protectSpeed(pitchCmd, state);
    out.pstick = limit(pitchCmd, -1.0, 1.0);

    return out;
}

// ---------------------------------------------------------------------------
// Terrain following
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeTerrainFollow(const AircraftState& state,
                                                     double dt, double groundZ) {
    PilotInput out = manual_;

    // Hold heading
    double targetHeading = goal_.hasHeading ? goal_.heading_rad : state.kin.psi;
    const double hErr = headingError(targetHeading, state.kin.psi);
    const double desiredBank = limit(hErr * 2.0, -30.0 * DTR, 30.0 * DTR);
    out.rstick = limit((desiredBank - state.kin.phi) * 2.0, -1.0, 1.0);

    // Hold a set AGL altitude
    const double agl_ft = -state.kin.z - groundZ;
    const double altErr = goal_.radarAltitude_ft - agl_ft;
    double pitchCmd = pitchPID_.update(altErr, dt);
    pitchCmd = protectSpeed(pitchCmd, state);
    out.pstick = limit(pitchCmd, -1.0, 1.0);

    // Hold speed
    if (goal_.hasSpeed) {
        const double speedErr = goal_.speed_kts - state.vcas;
        out.throttle = throttlePID_.update(speedErr, dt);
    } else {
        out.throttle = 0.7;
    }

    return out;
}

// ---------------------------------------------------------------------------
// Combat steering (placeholder -- points at a target)
// ---------------------------------------------------------------------------
PilotInput SteeringController::computeCombat(const AircraftState& state, double dt) {
    // For now, just hold heading and altitude. A real combat-steering module
    // would track a target, compute a lead-pursuit or lag-pursuit curve, and
    // manage weapons envelopes. That's future work.
    return computeHeadingAltitude(state, dt);
}

// ---------------------------------------------------------------------------
// Top-level dispatcher
// ---------------------------------------------------------------------------
PilotInput SteeringController::compute(const AircraftState& state, double dt,
                                        double groundZ) {
    switch (mode_) {
        case SteeringMode::Manual:        return manual_;
        case SteeringMode::Waypoint:      return computeWaypoint(state, dt, groundZ);
        case SteeringMode::HeadingAltitude: return computeHeadingAltitude(state, dt);
        case SteeringMode::Approach:      return computeApproach(state, dt);
        case SteeringMode::Formation:     return computeFormation(state, dt);
        case SteeringMode::TerrainFollow: return computeTerrainFollow(state, dt, groundZ);
        case SteeringMode::Combat:        return computeCombat(state, dt);
    }
    return manual_;
}

} // namespace f4flight
