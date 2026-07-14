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
//
// ============================================================================
// Host-facing API (refactored 2026-07-14)
// ============================================================================
//
// The API is split into four clear categories:
//
//   1. configure(DigiConfig)     — set once at init (skill, G-limits, etc.)
//   2. setWaypoints / setHeading — navigation setup (rarely changes)
//   3. setFrameInputs(FrameInputs) — per-frame world state (truth, threats)
//   4. commandXxx(...)           — asynchronous commands (takeoff, landing)
//
// Output: compute() returns PilotInput each frame.
// State:  state() returns const DigiState& (read-only).
//         stateMutable() returns DigiState& (TESTING ONLY).
//
// Old set* methods are retained as [[deprecated]] shims so existing code
// keeps compiling. They delegate to the new internal storage. Migrate
// callers at your convenience; the shims will be removed in a future
// release.

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/flight/core/types.h"

#include <vector>
#include <optional>

namespace f4flight {
namespace digi {

// ===========================================================================
// DigiConfig — configuration set once at init (persists across frames).
//
// Use DigiBrain::configure(DigiConfig) to apply. Fields map 1:1 to DigiState
// fields that are "configuration" (not per-frame inputs).
// ===========================================================================
struct DigiConfig {
    SkillLevel skillLevel    {SkillLevel::Veteran};
    double     cornerSpeedKts{330.0};   // digi.cornerSpeed
    double     maxGs         {9.0};      // digi.maxGs
    double     maxBankDeg    {30.0};     // digi.maxRoll
    double     maxGammaDeg   {60.0};     // digi.maxGammaDeg
    double     turnLoadFactor{2.0};      // digi.turnLoadFactor
};

// ===========================================================================
// FrameInputs — per-frame world state the host provides before compute().
//
// Production code: populate `truth` and let SensorFusion detect threats
// and targets autonomously.
//
// Testing code: populate `injectedMissile` / `injectedGunsThreat` /
// `injectedTarget` to bypass SensorFusion and directly set the threat/
// target the brain will react to. These are read with priority over
// SensorPicture in resolveMode().
// ===========================================================================
struct FrameInputs {
    // SensorFusion input (production path). Nullptr = no sensor fusion.
    const TruthState* truth              {nullptr};

    // Own aircraft entity. If null, the brain auto-builds from AircraftState
    // in compute(). Set this if the host has a richer entity model.
    const DigiEntity* selfEntity         {nullptr};

    // Offensive target (bandit). Nullptr = no injected target; brain uses
    // SensorPicture.bestTarget if truth is provided.
    const DigiEntity* injectedTarget     {nullptr};

    // --- Testing-only injection (bypasses SensorFusion) ---
    // Use these for unit tests that need deterministic threat geometry.
    // Production code should populate `truth` instead.
    const DigiEntity* injectedMissile    {nullptr};
    const DigiEntity* injectedGunsThreat {nullptr};

    // --- Round-2 structural addition (Rec 9): A/G target injection ---
    // The host can inject a ground target for the future GroundMnvr mode.
    // When non-null, the brain's groundTarget pointer is set; the (still
    // unported) GroundAttackMode will read it.
    const DigiEntity* injectedGroundTarget {nullptr};
};

// ===========================================================================
// DigiBrain — the AI pilot.
//
// Owns DigiState, dispatches to per-mode maneuver functions, produces
// PilotInput. The flight model calls compute() each frame with the current
// AircraftState; the brain returns the PilotInput for that frame.
// ===========================================================================
class DigiBrain {
public:
    DigiBrain();

    // =======================================================================
    // 1. Configuration (set once at init, persists)
    // =======================================================================

    /// Apply configuration. Updates the underlying DigiState fields
    /// (skill, cornerSpeed, maxGs, maxRoll, maxGammaDeg, turnLoadFactor).
    void configure(const DigiConfig& cfg);

    /// Read the current configuration (reconstructed from DigiState).
    DigiConfig config() const;

    // =======================================================================
    // 2. Navigation setup (rarely changes)
    // =======================================================================

    void setWaypoints(std::vector<Vec3> wps) {
        wps_ = std::move(wps);
        curWp_ = 0;
    }
    void setCaptureRadius(double r_ft) { captureRadius_ = r_ft; }

    /// Set the held heading (radians). Read by Waypoint mode when no
    /// waypoints remain, and by HeadingAltitude mode.
    void setHeading(double rad) { state_.holdPsi = rad; }

    /// Set the held altitude (ft, positive up). Read by Waypoint mode.
    void setAltitude(double ft) { state_.holdAlt = ft; }

    // =======================================================================
    // 3. Per-frame inputs (host provides each frame before compute())
    // =======================================================================

    /// Set the per-frame world inputs (truth, self entity, threats, target).
    /// Call this every frame before compute(). Values persist until the next
    /// call (so you can skip a frame and the brain reuses the last inputs).
    ///
    /// API contract: if `injectedMissile` / `injectedGunsThreat` /
    /// `injectedTarget` are null AND `truth` is null, any previously
    /// committed threat/target pointer in `state_` is cleared. This prevents
    /// a use-after-free when the host's injected entity goes out of scope
    /// (e.g. a test phase that injects a local DigiEntity then ends — the
    /// next phase must not inherit a dangling pointer). When `truth` is
    /// non-null, SensorFusion owns threat/target tracking and stale pointers
    /// are refreshed in `resolveMode()`.
    void setFrameInputs(const FrameInputs& inputs) {
        const bool hostGaveUpOnThreats = (inputs.truth == nullptr);
        if (hostGaveUpOnThreats) {
            if (!inputs.injectedMissile) {
                state_.incomingMissile = nullptr;
                state_.incomingMissileId = kInvalidEntityId;
                missileEntityAuto_.reset();
            }
            if (!inputs.injectedGunsThreat) {
                state_.gunsThreat = nullptr;
                gunsEntityAuto_.reset();
            }
            if (!inputs.injectedTarget) {
                wvrTarget_ = nullptr;
                targetEntityAuto_.reset();
            }
            // Round-2 fix (Rec 9): also clear stale A/G target pointer.
            if (!inputs.injectedGroundTarget) {
                state_.groundTarget = nullptr;
                state_.groundTargetId = kInvalidEntityId;
            }
        }
        // Commit injected ground target immediately (the future GroundMnvr
        // mode will read state_.groundTarget; the host's injection is the
        // production path).
        if (inputs.injectedGroundTarget) {
            state_.groundTarget = inputs.injectedGroundTarget;
        }
        frameInputs_ = inputs;
    }

    /// Read back the current frame inputs.
    const FrameInputs& frameInputs() const { return frameInputs_; }

    // =======================================================================
    // 4. Commands (asynchronous; take effect on next compute())
    // =======================================================================

    /// Command the brain to initiate a takeoff sequence. The brain will
    /// request clearance from ATC (via message bus) and execute the takeoff
    /// when cleared.
    void commandTakeoff(RunwayId rwy, double rwyHeading,
                        double rwyThresholdX, double rwyThresholdY,
                        double rwyAlt);

    /// Command the brain to initiate a landing sequence.
    void commandLanding(RunwayId rwy, double rwyHeading,
                        double rwyThresholdX, double rwyThresholdY,
                        double rwyAlt);

    /// Clear the offensive target (brain falls back to auto-targeting or
    /// waypoint navigation).
    void clearTarget() { frameInputs_.injectedTarget = nullptr; }

    // =======================================================================
    // 5. Mode override (testing / scripting only)
    // =======================================================================

    void forceMode(DigiMode m) { forcedMode_ = m; }
    void clearForcedMode()     { forcedMode_ = DigiMode::NoMode; }

    // =======================================================================
    // 6. Communication (set once at init)
    // =======================================================================

    void setSelfId(EntityId id) { state_.selfId = id; }
    void setMessageBus(MessageBus* bus) { bus_ = bus; }
    Mailbox& mailbox() { return state_.mailbox; }

    // =======================================================================
    // 6b. Weapon system (set once at init)
    // =======================================================================

    /// Provide the StoresManagementSystem. The brain reads it to select
    /// weapons and check firing envelopes. If not set, the brain defaults
    /// to a gun-only loadout (for GunsEngage).
    void setSMS(const StoresManagementSystem* sms) { sms_ = sms; }
    const StoresManagementSystem* sms() const { return sms_; }

    // =======================================================================
    // 7. Sensor system
    // =======================================================================

    SensorFusion&       sensorFusion()       { return sensorFusion_; }
    const SensorFusion& sensorFusion() const { return sensorFusion_; }
    const SensorPicture& sensorPicture() const { return sensorFusion_.picture(); }

    // =======================================================================
    // 8. Output
    // =======================================================================

    /// Compute the PilotInput for this frame.
    /// Reads from frameInputs_ (set via setFrameInputs) and the AircraftState.
    PilotInput compute(const AircraftState& as, double dt, double groundZ,
                       const FlightControlSystem& fcs, FcsState& fcsState);

    // =======================================================================
    // 9. Read-only state access (const only)
    // =======================================================================

    DigiMode activeMode() const { return activeMode_; }
    const DigiState& state() const { return state_; }
    std::size_t currentWaypoint() const { return curWp_; }
    bool allWaypointsCaptured() const { return curWp_ >= wps_.size(); }
    const std::vector<Vec3>& waypoints() const { return wps_; }

    /// TESTING ONLY — mutable state access. Production code should never
    /// write to DigiState directly; use configure() / setFrameInputs() /
    /// commandXxx() instead.
    DigiState& stateMutable() { return state_; }

    // =======================================================================
    // 10. Static helpers
    // =======================================================================

    /// Build a DigiEntity from the current AircraftState. Called automatically
    /// in compute() if frameInputs_.selfEntity is null. Also exposed publicly
    /// so hosts can use it to populate their own DigiEntity if desired.
    static DigiEntity buildSelfEntity(const AircraftState& as);

    // =======================================================================
    // 11. Reset
    // =======================================================================

    /// Reset all internal state (call between independent test phases).
    /// Clears frame inputs, auto-tracked entities, and threat pointers.
    void reset() noexcept {
        state_.reset();
        // DigiState::reset() intentionally doesn't clear threat pointers
        // (host-managed), but the brain's frameInputs_ injection path means
        // we need to clear them here so a stale injected threat doesn't
        // persist across reset().
        state_.incomingMissile = nullptr;
        state_.gunsThreat = nullptr;
        curWp_ = 0;
        activeMode_ = DigiMode::Waypoint;
        forcedMode_ = DigiMode::NoMode;
        frameInputs_ = FrameInputs{};
        missileEntityAuto_.reset();
        gunsEntityAuto_.reset();
        targetEntityAuto_.reset();
        selfEntityExplicit_ = false;
        wvrTarget_ = nullptr;
    }

    // =======================================================================
    // DEPRECATED backward-compat shims
    // -------------------------------
    // Kept so existing code (tests, SteeringController, host programs)
    // continues to compile. Each shim delegates to the new internal
    // storage (frameInputs_ / state_). Migrate callers to the new API
    // above; these will be removed in a future release.
    // =======================================================================

    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setSkill(SkillLevel level) {
        state_.skill = makeSkillParams(level);
    }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setCornerSpeed(double kts) { state_.cornerSpeed = kts; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxGs(double g) { state_.maxGs = g; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxBank(double deg) { state_.maxRoll = deg; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxGamma(double deg) { state_.maxGammaDeg = deg; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setTurnG(double lf) { state_.turnLoadFactor = lf; }

    /// @deprecated Use setFrameInputs(FrameInputs) instead.
    [[deprecated("Use setFrameInputs(FrameInputs)")]]
    void setTruth(const TruthState* truth) { frameInputs_.truth = truth; }

    /// @deprecated Use setFrameInputs(FrameInputs) instead.
    [[deprecated("Use setFrameInputs(FrameInputs)")]]
    void setSelfEntity(const DigiEntity* s) {
        frameInputs_.selfEntity = s;
        selfEntityExplicit_ = (s != nullptr);
    }
    /// @deprecated Use setFrameInputs(FrameInputs) with selfEntity=nullptr.
    [[deprecated("Use setFrameInputs(FrameInputs) with selfEntity=nullptr")]]
    void clearSelfEntity() {
        frameInputs_.selfEntity = nullptr;
        selfEntityExplicit_ = false;
    }

    /// @deprecated Use setFrameInputs(FrameInputs) with injectedMissile.
    [[deprecated("Use setFrameInputs(FrameInputs).injectedMissile")]]
    void setIncomingMissile(const DigiEntity* m) {
        frameInputs_.injectedMissile = m;
        // Also commit to state_ so runMissileDefeat sees it immediately.
        state_.incomingMissile = m;
        if (m) {
            // Reset per-missile state on injection.
            state_.missileDefeatTtgo = -1.0;
            state_.incomingMissileEvadeTimer = 0.0;
        }
    }
    /// @deprecated Use setFrameInputs(FrameInputs) with injectedGunsThreat.
    [[deprecated("Use setFrameInputs(FrameInputs).injectedGunsThreat")]]
    void setGunsThreat(const DigiEntity* t) {
        frameInputs_.injectedGunsThreat = t;
        state_.gunsThreat = t;
    }

    /// @deprecated Use setFrameInputs(FrameInputs) with injectedTarget.
    [[deprecated("Use setFrameInputs(FrameInputs).injectedTarget")]]
    void setTarget(const DigiEntity* t) {
        frameInputs_.injectedTarget = t;
    }

    /// @deprecated Use commandTakeoff(...) instead.
    [[deprecated("Use commandTakeoff(...)")]]
    void startTakeoff(RunwayId rwy, double rwyHeading,
                      double rwyThresholdX, double rwyThresholdY,
                      double rwyAlt) {
        commandTakeoff(rwy, rwyHeading, rwyThresholdX, rwyThresholdY, rwyAlt);
    }
    /// @deprecated Use commandLanding(...) instead.
    [[deprecated("Use commandLanding(...)")]]
    void startLanding(RunwayId rwy, double rwyHeading,
                      double rwyThresholdX, double rwyThresholdY,
                      double rwyAlt) {
        commandLanding(rwy, rwyHeading, rwyThresholdX, rwyThresholdY, rwyAlt);
    }

    /// @deprecated Use forceMode(...) instead.
    [[deprecated("Use forceMode(...)")]]
    void setForcedMode(DigiMode m) { forcedMode_ = m; }

    /// @deprecated Use stateMutable() for write access or state() for read.
    [[deprecated("Use state() (const) or stateMutable() (testing)")]]
    DigiState& state() { return state_; }

private:
    // --- Internal state ---
    DigiState state_;
    std::vector<Vec3> wps_;
    std::size_t curWp_{0};
    double captureRadius_{5000.0};  // ft

    // Per-frame inputs (set via setFrameInputs, or via deprecated shims)
    FrameInputs frameInputs_;

    // Legacy self-entity tracking (used by deprecated setSelfEntity shim)
    bool selfEntityExplicit_{false};

    // Auto-built self entity (when frameInputs_.selfEntity is null)
    DigiEntity selfEntityAuto_;

    // Resolved WVR target for the current frame (set by resolveMode, read
    // by runWVREngage). Points to either frameInputs_.injectedTarget or
    // targetEntityAuto_.
    const DigiEntity* wvrTarget_{nullptr};

    MessageBus* bus_{nullptr};       // message bus for ATC/flight comms
    const StoresManagementSystem* sms_{nullptr};  // weapon stores (optional)
    double simTime_{0.0};            // current sim time (seconds)

    // Sensor system (Phase 3)
    SensorFusion sensorFusion_;

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
    void runCollisionAvoid(const AircraftState& as, double dt,
                           const FlightControlSystem& fcs, FcsState& fcsState);
    void runWVREngage(const AircraftState& as, double dt,
                      const FlightControlSystem& fcs, FcsState& fcsState);
    void runGunsEngage(const AircraftState& as, double dt,
                       const FlightControlSystem& fcs, FcsState& fcsState);
    void runMissileEngage(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState);
    void runBVREngage(const AircraftState& as, double dt,
                      const FlightControlSystem& fcs, FcsState& fcsState);
    void runMerge(const AircraftState& as, double dt,
                  const FlightControlSystem& fcs, FcsState& fcsState);
    void runAccel(const AircraftState& as, double dt,
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
