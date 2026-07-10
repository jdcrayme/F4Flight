// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// steering.h
//
// AI steering / autopilot module.
//
// This module provides higher-level steering behaviours that translate
// navigational commands (waypoints, headings, altitudes, speeds) into the
// PilotInput struct consumed by FlightModel::update().
//
// The design is layered:
//
//   ┌──────────────────────────────────────────────────────────┐
//   │            SteeringController (top-level)                │
//   │   - holds the active SteeringMode                       │
//   │   - dispatches to the mode-specific compute() function  │
//   ├──────────────────────────────────────────────────────────┤
//   │  WaypointFollower  │  HeadingHold  │  AltitudeHold      │
//   │  SpeedHold         │  Formation    │  CombatSteering    │
//   │   (leaf behaviours, each produces a PilotInput)         │
//   ├──────────────────────────────────────────────────────────┤
//   │              PID controllers                            │
//   │   - PitchPID (altitude -> pitch stick)                  │
//   │   - RollPID  (heading  -> roll stick)                   │
//   │   - ThrottlePID (speed -> throttle)                     │
//   │   - YawPID   (sideslip -> rudder pedal)                 │
//   └──────────────────────────────────────────────────────────┘
//
// Port of the relevant logic from FreeFalcon's sim/digi/ directory
// (autopilot.cpp, waypoint.cpp, wingai.cpp, refuel.cpp, landme.cpp)
// but refactored into a clean, testable, modular design.

#pragma once

#include "f4flight/aircraft_state.h"
#include "f4flight/core/types.h"

#include <vector>

namespace f4flight {

// ---------------------------------------------------------------------------
// Steering goals -- the high-level commands the host program issues.
// Each behaviour reads whichever fields it needs and ignores the rest.
// ---------------------------------------------------------------------------
struct SteeringGoal {
    // Waypoint following
    Vec3   waypoint{0.0, 0.0, 0.0};   // target position (world, ft, NED Z-down)
    bool   hasWaypoint{false};

    // Heading hold (radians, 0 = North, pi/2 = East)
    double heading_rad{0.0};
    bool   hasHeading{false};

    // Altitude hold (feet, positive up)
    double altitude_ft{5000.0};
    bool   hasAltitude{false};

    // Speed hold (knots calibrated airspeed)
    double speed_kts{300.0};
    bool   hasSpeed{false};

    // Climb/descent schedule
    double climbPower{1.0};          // throttle for climbs (1.0 = MIL)
    double descentPower{0.05};       // throttle for descents (0 = idle)
    double climbVcas_kts{350.0};     // target CAS during climb (slower than cruise)
    double climbMach{0.80};          // target Mach during climb (after transition)
    double descentVcas_kts{460.0};   // target CAS during descent (faster than cruise)
    double descentMach{0.80};        // target Mach during descent (after transition)
    double levelBand_ft{200.0};      // altitude band for level flight

    // Approach / landing
    bool   landing{false};
    Vec3   runwayThreshold{0.0, 0.0, 0.0};
    double runwayHeading_rad{0.0};

    // Formation flying
    Vec3   formationOffset{0.0, 0.0, 0.0};
    bool   hasFormationLead{false};
    Vec3   leadPosition{0.0, 0.0, 0.0};
    Vec3   leadVelocity{0.0, 0.0, 0.0};

    // Terrain follow
    double radarAltitude_ft{500.0};
    bool   terrainFollow{false};
};

// ---------------------------------------------------------------------------
// PID controller with anti-windup and output clamping.
// ---------------------------------------------------------------------------
struct PIDGains {
    double kp{1.0};
    double ki{0.0};
    double kd{0.0};
    double outputMin{-1.0};   // output clamp
    double outputMax{ 1.0};
    double integMin{-1.0};    // integrator clamp (anti-windup)
    double integMax{ 1.0};
};

class PID {
public:
    PID() = default;
    explicit PID(const PIDGains& g) : gains_(g) {}

    // Compute the PID output for the given error and dt.
    //   error = setpoint - measurement (positive error -> positive output)
    double update(double error, double dt) noexcept;

    void reset() noexcept { integ_ = 0.0; prevError_ = 0.0; hasPrev_ = false; }
    void setGains(const PIDGains& g) noexcept { gains_ = g; }
    const PIDGains& gains() const noexcept { return gains_; }

private:
    PIDGains gains_;
    double integ_{0.0};
    double prevError_{0.0};
    bool   hasPrev_{false};
};

// ---------------------------------------------------------------------------
// Steering mode enum
// ---------------------------------------------------------------------------
enum class SteeringMode {
    Manual,           // pilot input passes through unchanged
    Waypoint,         // fly to the waypoint, then the next, etc.
    HeadingAltitude,  // hold heading + altitude + speed
    Approach,         // fly an ILS-like approach to the runway
    Formation,        // hold position relative to a lead aircraft
    TerrainFollow,    // follow the terrain at a set AGL
    Combat,           // point at a target (placeholder for future work)
};

// ---------------------------------------------------------------------------
// Steering controller. Holds the PID states and the active mode.
// ---------------------------------------------------------------------------
class SteeringController {
public:
    SteeringController();

    // Set the active steering mode.
    void setMode(SteeringMode mode) noexcept { mode_ = mode; }
    SteeringMode mode() const noexcept { return mode_; }

    // Set the steering goal (the high-level commands).
    void setGoal(const SteeringGoal& goal) noexcept { goal_ = goal; }
    const SteeringGoal& goal() const noexcept { return goal_; }

    // Set the waypoint sequence (for Waypoint mode).
    void setWaypoints(const std::vector<Vec3>& wps) noexcept { waypoints_ = wps; curWp_ = 0; }
    void advanceWaypoint() noexcept { if (curWp_ < waypoints_.size()) ++curWp_; }
    std::size_t currentWaypointIndex() const noexcept { return curWp_; }
    const std::vector<Vec3>& waypoints() const noexcept { return waypoints_; }

    // Set the manual pilot input (used in Manual mode, and as a fallback).
    void setManualInput(const PilotInput& in) noexcept { manual_ = in; }

    // Compute the pilot input for this frame.
    //   state   : current aircraft state (from FlightModel::state())
    //   dt      : frame time step (seconds)
    //   groundZ : terrain altitude at aircraft position (ft)
    PilotInput compute(const AircraftState& state, double dt, double groundZ);

    // Tunable PID accessors. Host programs can tweak these at runtime.
    PID& pitchPID()       noexcept { return pitchPID_; }
    PID& rollPID()        noexcept { return rollPID_; }
    PID& throttlePID()    noexcept { return throttlePID_; }
    PID& yawPID()         noexcept { return yawPID_; }
    PID& altPID()         noexcept { return altPID_; }

    // Configuration
    void   setMaxBankAngle_deg(double v) noexcept { maxBank_deg_ = v; }
    double maxBankAngle_deg()   const noexcept { return maxBank_deg_; }
    void   setMaxPitchAngle_deg(double v) noexcept { maxPitch_deg_ = v; }
    double maxPitchAngle_deg()  const noexcept { return maxPitch_deg_; }
    void   setWaypointCaptureRadius_ft(double v) noexcept { wpCapture_ft_ = v; }
    double waypointCaptureRadius_ft() const noexcept { return wpCapture_ft_; }
    void   setMaxGs(double v) noexcept { maxGs_ = v; }
    double maxGs() const noexcept { return maxGs_; }

private:
    PilotInput computeWaypoint(const AircraftState& state, double dt, double groundZ);
    PilotInput computeHeadingAltitude(const AircraftState& state, double dt);
    PilotInput computeApproach(const AircraftState& state, double dt);
    PilotInput computeFormation(const AircraftState& state, double dt);
    PilotInput computeTerrainFollow(const AircraftState& state, double dt, double groundZ);
    PilotInput computeCombat(const AircraftState& state, double dt);

    // Combine a "pitch command" (from the altitude PID) with a "speed
    // protection" that prevents the aircraft from stalling.
    double protectSpeed(double pitchCmd, const AircraftState& state) noexcept;

    SteeringMode         mode_{SteeringMode::Manual};
    SteeringGoal         goal_;
    PilotInput           manual_;
    std::vector<Vec3>    waypoints_;
    std::size_t          curWp_{0};

    PID pitchPID_;      // altitude error -> pitch stick
    PID rollPID_;       // heading error  -> roll stick
    PID throttlePID_;   // speed error    -> throttle
    PID yawPID_;        // sideslip       -> rudder pedal
    PID altPID_;        // low-level altitude -> pitch (inner loop)
    PID speedPID_;      // speed error -> pitch (climb/descent: pitch for speed)

    double maxBank_deg_{30.0};
    double maxPitch_deg_{20.0};
    double wpCapture_ft_{2000.0};   // waypoint capture radius (ft)
    double maxGs_{9.0};             // aircraft max G (for G-command mapping)

    // Previous-frame values for derivative computation
    double prevAltErr_{0.0};
    bool   hasPrevAltErr_{false};
};

} // namespace f4flight
