// f4flight - steering.h
// AI steering with separated horizontal/vertical/throttle behaviors.
//
// Architecture:
//   SteeringController
//     ├── VerticalBehavior*   (controls pstick + throttle when climbing/descending)
//     ├── HorizontalBehavior* (controls rstick)
//     └── ThrottleBehavior*   (controls throttle when in level flight)
//
// Each behavior takes its parameters in its constructor — no shared goal struct.
// Behaviors are self-contained and composable.

#pragma once

#include "f4flight/aircraft_state.h"
#include "f4flight/core/types.h"

#include <memory>
#include <vector>

namespace f4flight {

// ---------------------------------------------------------------------------
// PID controller
// ---------------------------------------------------------------------------
struct PIDGains {
    double kp{1.0}, ki{0.0}, kd{0.0};
    double outputMin{-1.0}, outputMax{1.0};
    double integMin{-1.0}, integMax{1.0};
};

class PID {
public:
    PID() = default;
    explicit PID(const PIDGains& g) : gains_(g) {}
    double update(double error, double dt) noexcept;
    void reset() noexcept { integ_ = 0.0; prevError_ = 0.0; hasPrev_ = false; }
    void setGains(const PIDGains& g) noexcept { gains_ = g; }
    const PIDGains& gains() const noexcept { return gains_; }
private:
    PIDGains gains_;
    double integ_{0.0}, prevError_{0.0};
    bool hasPrev_{false};
};

// ---------------------------------------------------------------------------
// Shared controller state (PID controllers + config)
// ---------------------------------------------------------------------------
struct SteeringContext {
    PilotInput manual;
    double maxBank_deg{30.0};
    double maxGs{9.0};

    PID pitchPID;
    PID rollPID;
    PID throttlePID;
    PID yawPID;
};

// ---------------------------------------------------------------------------
// Abstract base classes
// ---------------------------------------------------------------------------
class VerticalBehavior {
public:
    virtual ~VerticalBehavior() = default;
    // Compute pstick [-1,1]. May also set out.throttle.
    virtual double compute(const AircraftState& state, double dt,
                           SteeringContext& ctx, PilotInput& out) = 0;
    virtual const char* name() const = 0;
};

class HorizontalBehavior {
public:
    virtual ~HorizontalBehavior() = default;
    // Compute rstick [-1,1].
    virtual double compute(const AircraftState& state, double dt,
                           SteeringContext& ctx) = 0;
    virtual const char* name() const = 0;
};

class ThrottleBehavior {
public:
    virtual ~ThrottleBehavior() = default;
    // Compute throttle [0, 1.5].
    virtual double compute(const AircraftState& state, double dt,
                           SteeringContext& ctx) = 0;
    virtual const char* name() const = 0;
};

// ---------------------------------------------------------------------------
// Vertical behaviors
// ---------------------------------------------------------------------------

// AltitudeHold: unified altitude control.
// When far from target: climb/descent mode (throttle for alt, pitch for spd).
// When near target: level mode (pitch for alt, throttle for spd).
// The VVI cap handles level-off automatically — no state machine needed.
class AltitudeHold : public VerticalBehavior {
public:
    AltitudeHold(double targetAlt_ft, double cruiseSpeed_kts,
                 double climbSpeed_kts = 0.0, double climbMach = 0.80, double climbPower = 1.0, 
                 double descentSpeed_kts = 0.0, double descentMach = 0.80, double descentPower = 0.05,
                 double levelBand_ft = 200.0);

    double compute(const AircraftState& state, double dt,
                   SteeringContext& ctx, PilotInput& out) override;
    const char* name() const override { return "AltitudeHold"; }

    void setTargetAltitude(double alt_ft) { targetAlt_ = alt_ft; }
    void setCruiseSpeed(double kts) { cruiseSpeed_ = kts; }

private:
    double targetAlt_;
    double cruiseSpeed_;
    double climbSpeed_;     // 0 = same as cruise
    double climbMach_;
    double climbPower_;
    double descentPower_;
    double descentSpeed_;   // 0 = same as cruise
    double descentMach_;
    double levelBand_;
};

// ---------------------------------------------------------------------------
// Horizontal behaviors
// ---------------------------------------------------------------------------

// HeadingHold: hold a specific heading
class HeadingHold : public HorizontalBehavior {
public:
    explicit HeadingHold(double heading_rad) : heading_(heading_rad) {}
    double compute(const AircraftState& state, double dt,
                   SteeringContext& ctx) override;
    const char* name() const override { return "HeadingHold"; }
    void setHeading(double h) { heading_ = h; }
private:
    double heading_;
};

// SteerToWaypoint: fly toward waypoints, advance on capture
class SteerToWaypoint : public HorizontalBehavior {
public:
    SteerToWaypoint(std::vector<Vec3> wps, double captureRadius_ft)
        : wps_(std::move(wps)), captureRadius_(captureRadius_ft) {}
    double compute(const AircraftState& state, double dt,
                   SteeringContext& ctx) override;
    const char* name() const override { return "SteerToWaypoint"; }
    std::size_t currentWaypoint() const { return curWp_; }
    bool allCaptured() const { return curWp_ >= wps_.size(); }
private:
    std::vector<Vec3> wps_;
    double captureRadius_;
    std::size_t curWp_{0};
};

// ---------------------------------------------------------------------------
// Throttle behaviors
// ---------------------------------------------------------------------------

// SpeedHold: PID throttle to maintain target speed
class SpeedHold : public ThrottleBehavior {
public:
    explicit SpeedHold(double targetSpeed_kts) : target_(targetSpeed_kts) {}
    double compute(const AircraftState& state, double dt,
                   SteeringContext& ctx) override;
    const char* name() const override { return "SpeedHold"; }
    void setTargetSpeed(double kts) { target_ = kts; }
private:
    double target_;
};

// ---------------------------------------------------------------------------
// SteeringController — manages active behaviors and combines outputs
// ---------------------------------------------------------------------------
class SteeringController {
public:
    SteeringController();

    void setManualInput(const PilotInput& in) noexcept { ctx_.manual = in; }

    // Behavior management
    void setVerticalBehavior(std::unique_ptr<VerticalBehavior> b) { vert_ = std::move(b); }
    void setHorizontalBehavior(std::unique_ptr<HorizontalBehavior> b) { horiz_ = std::move(b); }
    void setThrottleBehavior(std::unique_ptr<ThrottleBehavior> b) { thrott_ = std::move(b); }

    // Access active behaviors (for inspection — e.g. checking waypoint index)
    VerticalBehavior* verticalBehavior() { return vert_.get(); }
    HorizontalBehavior* horizontalBehavior() { return horiz_.get(); }
    ThrottleBehavior* throttleBehavior() { return thrott_.get(); }

    // Configuration
    void setMaxBankAngle_deg(double v) noexcept { ctx_.maxBank_deg = v; }
    double maxBankAngle_deg() const noexcept { return ctx_.maxBank_deg; }
    void setMaxGs(double v) noexcept { ctx_.maxGs = v; }
    double maxGs() const noexcept { return ctx_.maxGs; }

    // PID accessors
    PID& pitchPID()    noexcept { return ctx_.pitchPID; }
    PID& rollPID()     noexcept { return ctx_.rollPID; }
    PID& throttlePID() noexcept { return ctx_.throttlePID; }
    PID& yawPID()      noexcept { return ctx_.yawPID; }

    // Active behavior names
    const char* verticalBehaviorName() const { return vert_ ? vert_->name() : "None"; }
    const char* horizontalBehaviorName() const { return horiz_ ? horiz_->name() : "None"; }
    const char* throttleBehaviorName() const { return thrott_ ? thrott_->name() : "None"; }

    // Main compute — combines all three behaviors
    PilotInput compute(const AircraftState& state, double dt, double groundZ);

private:
    SteeringContext ctx_;
    std::unique_ptr<VerticalBehavior> vert_;
    std::unique_ptr<HorizontalBehavior> horiz_;
    std::unique_ptr<ThrottleBehavior> thrott_;
};

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
double headingError(double setpoint, double current) noexcept;
double turnCompensatedG(const AircraftState& state) noexcept;
double computeMaxVVI_fpm(double altErr_ft) noexcept;
double protectSpeed(double pitchCmd, const AircraftState& state, double maxGs) noexcept;

} // namespace f4flight
