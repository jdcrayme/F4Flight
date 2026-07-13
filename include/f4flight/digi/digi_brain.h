// f4flight - digi/digi_brain.h
//
// DigiBrain — the top-level digi AI dispatcher.
//
// This is the modern C++ equivalent of FreeFalcon's DigitalBrain class
// (sim/digi/digimain.cpp). It owns the DigiState, runs the per-frame
// FrameExec() pipeline, and dispatches to per-mode maneuver functions.
//
// Architecture:
//
//   ┌──────────────────────────────────────────────────┐
//   │ FlightModel (pure flight model, no AI dependency) │
//   └────────────────────┬─────────────────────────────┘
//                        │ AircraftState (read)
//                        ▼
//   ┌──────────────────────────────────────────────────┐
//   │ DigiBrain                                         │
//   │   1. GroundCheck (always runs)                   │
//   │   2. ResolveMode (priority stack)                │
//   │   3. Actions (dispatch to active mode)           │
//   │   4. Clamp outputs                               │
//   └────────────────────┬─────────────────────────────┘
//                        │ PilotInput (write)
//                        ▼
//   ┌──────────────────────────────────────────────────┐
//   │ FlightModel.update(dt, input, ...)                │
//   └──────────────────────────────────────────────────┘
//
// The SteeringController (steering.h) is a thin facade over DigiBrain that
// preserves the existing test API. New code should use DigiBrain directly.
//
// Port of FreeFalcon digimain.cpp:566 (DigitalBrain::FrameExec).

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"
#include "f4flight/core/types.h"

#include <vector>

namespace f4flight {
namespace digi {

// DigiBrain — the AI pilot.
//
// Owns DigiState, dispatches to per-mode maneuver functions, produces
// PilotInput. The flight model calls compute() each frame with the current
// AircraftState; the brain returns the PilotInput for that frame.
class DigiBrain {
public:
    DigiBrain();

    // --- Configuration ---
    void setSkill(SkillLevel level) {
        state_.skill = makeSkillParams(level);
    }

    void setCornerSpeed(double kts) { state_.cornerSpeed = kts; }
    void setMaxGs(double g)         { state_.maxGs = g; }
    void setMaxBank(double deg)     { state_.maxRoll = deg; }
    void setMaxGamma(double deg)    { state_.maxGammaDeg = deg; }
    void setTurnG(double lf)        { state_.turnLoadFactor = lf; }

    // --- Waypoint / navigation ---
    void setWaypoints(std::vector<Vec3> wps) { wps_ = std::move(wps); curWp_ = 0; }
    void setCaptureRadius(double r_ft)       { captureRadius_ = r_ft; }
    void setHeading(double rad)              { state_.holdPsi = rad; }
    void setAltitude(double ft)              { state_.holdAlt = ft; }

    // --- Threat entity setters (Tier 1) ---
    // The host sets these each frame from its own entity model.
    // Pass nullptr to clear a threat.
    void setIncomingMissile(const DigiEntity* m) { state_.incomingMissile = m; }
    void setGunsThreat(const DigiEntity* t)      { state_.gunsThreat = t; }

    // --- Own entity (for defensive maneuvers) ---
    // The host must set this each frame so the brain can compute relative
    // geometry to threats. If not set, defensive maneuvers are skipped.
    void setSelfEntity(const DigiEntity* s)      { selfEntity_ = s; }

    // --- Mode override (for testing / scripting) ---
    void setForcedMode(DigiMode m) { forcedMode_ = m; }
    void clearForcedMode()         { forcedMode_ = DigiMode::NoMode; }

    // --- Accessors ---
    DigiMode activeMode() const { return activeMode_; }
    const DigiState& state() const { return state_; }
    DigiState&       state()       { return state_; }
    std::size_t currentWaypoint() const { return curWp_; }
    bool allWaypointsCaptured() const { return curWp_ >= wps_.size(); }

    // --- Main compute ---
    PilotInput compute(const AircraftState& as, double dt, double groundZ,
                       const FlightControlSystem& fcs, FcsState& fcsState);

    // Reset all state (call between independent test phases).
    void reset() noexcept {
        state_.reset();
        curWp_ = 0;
        activeMode_ = DigiMode::Waypoint;
        forcedMode_ = DigiMode::NoMode;
    }

private:
    DigiState state_;
    std::vector<Vec3> wps_;
    std::size_t curWp_{0};
    double captureRadius_{5000.0};  // ft
    const DigiEntity* selfEntity_{nullptr};

    DigiMode activeMode_{DigiMode::Waypoint};
    DigiMode forcedMode_{DigiMode::NoMode};

    // --- Per-mode actions ---
    void runWaypoint(const AircraftState& as, double dt,
                     const FlightControlSystem& fcs, FcsState& fcsState);
    void runGroundAvoid(const AircraftState& as, double dt,
                        FcsState& fcsState);
    void runMissileDefeat(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState);
    void runGunsJink(const AircraftState& as, double dt,
                     const FlightControlSystem& fcs, FcsState& fcsState);
    void runWVREngage(const AircraftState& as, double dt,
                      const FlightControlSystem& fcs, FcsState& fcsState);

    // Resolve the active mode based on priority + threats.
    void resolveMode(const AircraftState& as, double groundZ, double dt);
};

} // namespace digi
} // namespace f4flight
