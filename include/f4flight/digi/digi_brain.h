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
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"
#include "f4flight/core/types.h"

#include <vector>
#include <optional>

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

    // --- Threat entity setters (Tier 1) — DEPRECATED ---
    // These are retained for backward compatibility but should not be used
    // when SensorFusion is enabled. The brain will auto-detect threats via
    // the SensorPicture. If both are set, the injected pointers take priority
    // (for testing).
    void setIncomingMissile(const DigiEntity* m) { state_.incomingMissile = m; }
    void setGunsThreat(const DigiEntity* t)      { state_.gunsThreat = t; }

    // --- Target entity setter (Tier 2 — offensive) ---
    // The host sets this each frame to the current target (bandit).
    // Pass nullptr to clear (brain falls back to navigation or auto-targeting).
    void setTarget(const DigiEntity* t) { target_ = t; }

    // --- Sensor system (Phase 3) ---
    // Provide the truth state each frame. The brain runs SensorFusion to
    // build a SensorPicture, then uses it for autonomous threat/target
    // detection. If no truth is provided, the brain falls back to injected
    // threats (backward compatibility).
    void setTruth(const TruthState* truth) { truth_ = truth; }

    // Access the sensor fusion (for configuration / testing)
    SensorFusion& sensorFusion() { return sensorFusion_; }
    const SensorFusion& sensorFusion() const { return sensorFusion_; }

    // Get the current sensor picture (read-only)
    const SensorPicture& sensorPicture() const { return sensorFusion_.picture(); }

    // --- Ground ops setters (Phase 1-2) ---
    // Set the aircraft's entity ID (for ATC addressing) and register its
    // mailbox on the message bus.
    void setSelfId(EntityId id) { state_.selfId = id; }
    void setMessageBus(MessageBus* bus) { bus_ = bus; }
    Mailbox& mailbox() { return state_.mailbox; }

    // --- Ground ops control ---
    // Command the brain to initiate a takeoff sequence. The brain will
    // request clearance from ATC (via message bus) and execute the takeoff
    // when cleared.
    void startTakeoff(RunwayId rwy, double rwyHeading,
                       double rwyThresholdX, double rwyThresholdY, double rwyAlt);

    // Command the brain to initiate a landing sequence.
    void startLanding(RunwayId rwy, double rwyHeading,
                       double rwyThresholdX, double rwyThresholdY, double rwyAlt);

    // --- Own entity (for defensive maneuvers) ---
    // The host may set this each frame from its own entity model.
    // If NOT set (nullptr), the brain will auto-sync from AircraftState in
    // compute() every frame. This makes the brain self-contained for testing.
    void setSelfEntity(const DigiEntity* s) { selfEntity_ = s; selfEntityExplicit_ = (s != nullptr); }
    void clearSelfEntity() { selfEntity_ = nullptr; selfEntityExplicit_ = false; }

    // Build a DigiEntity from the current AircraftState. Called automatically
    // in compute() if no selfEntity_ is set by the host. Also exposed publicly
    // so hosts can use it to populate their own DigiEntity if desired.
    static DigiEntity buildSelfEntity(const AircraftState& as);

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
    DigiEntity selfEntityAuto_;  // auto-synced from AircraftState when selfEntity_ is null
    bool selfEntityExplicit_{false};  // true if host called setSelfEntity()
    const DigiEntity* target_{nullptr};  // offensive target (Tier 2)
    MessageBus* bus_{nullptr};  // message bus for ATC/flight comms
    double simTime_{0.0};  // current sim time (seconds)

    // Sensor system (Phase 3)
    SensorFusion sensorFusion_;
    const TruthState* truth_{nullptr};

    // Auto-built entities from SensorPicture (for threat/target detection)
    std::optional<DigiEntity> missileEntityAuto_;
    std::optional<DigiEntity> gunsEntityAuto_;
    std::optional<DigiEntity> targetEntityAuto_;

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
    void runTakeoff(const AircraftState& as, double dt,
                    FcsState& fcsState, double groundZ);
    void runLanding(const AircraftState& as, double dt,
                    FcsState& fcsState, double groundZ);

    // Resolve the active mode based on priority + threats.
    void resolveMode(const AircraftState& as, double groundZ, double dt);
};

} // namespace digi
} // namespace f4flight
