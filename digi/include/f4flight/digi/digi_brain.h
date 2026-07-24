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

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"

#include <vector>
#include <optional>
#include <memory>

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
    double     cornerSpeedKts{330.0};   // digi.config.cornerSpeed
    double     maxGs         {9.0};      // digi.config.maxGs
    double     maxBankDeg    {30.0};     // digi.config.maxRoll
    double     maxGammaDeg   {60.0};     // digi.config.maxGammaDeg
    double     turnLoadFactor{2.0};      // digi.config.turnLoadFactor
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

    // --- Task 15-a: A/G attack profile selection ---
    // Selects which delivery geometry runGroundAttack() executes when the
    // injectedGroundTarget is non-null. Defaults to DiveBomb for backward
    // compatibility (existing scenarios that don't set it keep the Task 11
    // dive-bomb profile). Committed to state_.ag.agProfile by
    // setFrameInputs().
    //
    // Set this to LevelDelivery or TossBomb to exercise the new profiles.
    AgAttackProfile injectedAgProfile {AgAttackProfile::DiveBomb};

    // --- Wingman formation following ---
    // The flight lead entity. When non-null AND formation.isWing is true,
    // the brain enters Wingy mode and calls AiFollowLead to fly to the
    // wingman's formation slot relative to this lead.
    const DigiEntity* injectedLead {nullptr};

    // --- Air-to-air refueling (AAR) ---
    // The tanker entity. When non-null, the brain can enter Refueling mode
    // to fly to the tanker's boom position and hold formation for refueling.
    // The tanker must be a friendly aircraft flying straight and level.
    // The brain tracks the refueling state (approach → contact → disconnect)
    // in state_.refuel.
    const DigiEntity* injectedTanker {nullptr};

    // --- Secondary threat injection (HandleThreat port) ---
    // A "secondary threat" is an aircraft that has spiked us, fired a missile
    // at us, or been called out by a wingman/RWR. Distinct from
    // injectedMissile (the actual incoming missile) and injectedGunsThreat
    // (a guns threat). When non-null, the brain engages the threat via
    // HandleThreat() / RollAndPull for up to 10 s before re-evaluating.
    const DigiEntity* injectedThreat {nullptr};

    // --- Fuel state (FuelCheck port) ---
    // The host provides fuel state each frame so FuelCheck() can transition
    // fuel.phase. Set bingoFuelLbs/jokerFuelLbs/fumesFuelLbs once at mission
    // start; fuelLbs is updated every frame from AircraftState.fuel.fuel_lbs.
    double fuelLbs       {0.0};
    double bingoFuelLbs  {0.0};
    double jokerFuelLbs  {0.0};
    double fumesFuelLbs  {0.0};
    bool   winchester    {false};  // true = out of A/A weapons (host sets)

    // --- Damage state (SeparateCheck port — Round 6) ---
    // Airframe strength fraction (1.0 = pristine, 0.0 = destroyed).
    // Set by the host each frame from the damage model. When pctStrength
    // drops below 0.50, SeparateCheck enters RTB/Bugout mode.
    double pctStrength  {1.0};

    // --- Friendly airbase list (AirbaseCheck port — Round 6) ---
    // The host provides a list of friendly airbases so AirbaseCheck can
    // auto-pick the nearest one when fuel goes bingo/fumes. Each entry
    // is a divert candidate. The list is read-only; the brain doesn't
    // modify it. Nullptr/0 = no airbases available (brain falls back to
    // waypoint-based RTB).
    //
    // The airbase positions are in world frame (ft, NED). The brain
    // computes distance to each and picks the nearest.
    //
    // DESIGN NOTE: we use a raw pointer + count (not std::vector) because
    // FrameInputs is a plain aggregate copied by value each frame — a
    // vector would force a heap allocation per frame. The host owns the
    // array and ensures it outlives the compute() call.
    struct AirbaseInfo {
        double x{0.0}, y{0.0}, z{0.0};        // world position (ft, NED)
        double runwayHeading{0.0};             // runway heading (rad)
        EntityId id{kInvalidEntityId};          // for addressing (ATC clearance)
    };
    const AirbaseInfo* airbases{nullptr};
    std::size_t        airbaseCount{0};
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
        if (flightPlan_) {
            flightPlan_->clear();
            for (const auto& wp : wps_) {
                flightPlan_->pushTask(MissionTask{TaskType::Navigate, wp, state_.config.cornerSpeed, -wp.z, kInvalidEntityId, 0.0});
            }
        }
    }
    void setCaptureRadius(double r_ft) { captureRadius_ = r_ft; }
    double captureRadius() const { return captureRadius_; }

    /// Set the held heading (radians). Read by Waypoint mode when no
    /// waypoints remain, and by HeadingAltitude mode.
    void setHeading(double rad) { state_.nav.holdPsi = rad; }

    /// Set the held altitude (ft, positive up). Read by Waypoint mode.
    void setAltitude(double ft) { state_.nav.holdAlt = ft; }

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
                state_.missileDefeat.incomingMissile = nullptr;
                state_.missileDefeat.incomingMissileId = kInvalidEntityId;
                missileEntityAuto_.reset();
            }
            if (!inputs.injectedGunsThreat) {
                state_.gunsJink.gunsThreat = nullptr;
                gunsEntityAuto_.reset();
            }
            if (!inputs.injectedTarget) {
                wvrTarget_ = nullptr;
                targetEntityAuto_.reset();
            }
            // Round-2 fix (Rec 9): also clear stale A/G target pointer.
            if (!inputs.injectedGroundTarget) {
                state_.ag.groundTarget = nullptr;
                state_.ag.groundTargetId = kInvalidEntityId;
            }
            // HandleThreat: clear stale secondary threat pointer. The host
            // owns the entity lifetime; if it doesn't re-inject, we must not
            // hold a dangling pointer across frames.
            if (!inputs.injectedThreat) {
                state_.threat.threatPtr = nullptr;
                state_.threat.threatTimer = 0.0;
            }
        }
        // Commit injected ground target immediately (the future GroundMnvr
        // mode will read state_.ag.groundTarget; the host's injection is the
        // production path).
        if (inputs.injectedGroundTarget) {
            // Task 15-a: when a NEW ground target is injected (different
            // pointer from the previous one), reset the attack phase counter
            // so the new attack starts from approach (phase 0). Without this,
            // a previous scenario phase's egress phase (phase 3) leaks into
            // the next phase and immediately clears the new target because
            // the egress-complete range check passes at the start distance.
            if (state_.ag.groundTarget != inputs.injectedGroundTarget) {
                state_.ag.agApproach = 0;
            }
            state_.ag.groundTarget = inputs.injectedGroundTarget;
        }
        // Task 15-a: commit the A/G attack profile. The host selects the
        // delivery geometry via FrameInputs.injectedAgProfile; we mirror it
        // into state_.ag.agProfile so runGroundAttack() can dispatch on it.
        // Defaults to DiveBomb when the host doesn't set it (backward compat).
        state_.ag.agProfile = inputs.injectedAgProfile;
        // Commit injected secondary threat. HandleThreat will read threatPtr
        // each frame and re-evaluate every 10 s.
        if (inputs.injectedThreat) {
            // If this is a new threat (different pointer), arm the 10 s
            // re-eval timer so HandleThreat runs at least one full window
            // before dropping it.
            if (state_.threat.threatPtr != inputs.injectedThreat) {
                state_.threat.threatTimer = 0.0;  // 0 → re-eval next frame, then arm
            }
            state_.threat.threatPtr = inputs.injectedThreat;
        }
        // Commit fuel state. The host provides these each frame; FuelCheck()
        // (called from resolveMode) transitions state_.fuel.phase based on them.
        state_.fuel.fuelLbs       = inputs.fuelLbs;
        state_.fuel.bingoFuelLbs  = inputs.bingoFuelLbs;
        state_.fuel.jokerFuelLbs  = inputs.jokerFuelLbs;
        state_.fuel.fumesFuelLbs  = inputs.fumesFuelLbs;
        state_.fuel.winchester    = inputs.winchester;
        // Commit damage state (Round 6: SeparateCheck reads pctStrength).
        state_.damage.pctStrength = inputs.pctStrength;
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

    void setSelfId(EntityId id) { state_.comm.selfId = id; }
    void setMessageBus(MessageBus* bus) { bus_ = bus; }
    Mailbox& mailbox() { return state_.comm.mailbox; }

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

    DigiMode activeMode() const { return curMode_; }
    const DigiState& state() const { return state_; }
    std::size_t currentWaypoint() const {
        if (flightPlan_ && !flightPlan_->tasks().empty()) {
            return flightPlan_->currentTaskIndex();
        }
        return curWp_;
    }
    bool allWaypointsCaptured() const {
        if (flightPlan_ && !flightPlan_->tasks().empty()) {
            return flightPlan_->isComplete();
        }
        return curWp_ >= wps_.size();
    }
    const std::vector<Vec3>& waypoints() const { return wps_; }

    /// The resolved offensive target for THIS frame (injected or auto-tracked
    /// from SensorFusion). Non-null when the brain has a target to engage.
    /// Used by the test framework / visualization to draw the target in the
    /// report even when the target was detected autonomously (not injected).
    const DigiEntity* resolvedTarget() const { return wvrTarget_; }

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
        if (rootNode_) rootNode_->reset();
        if (flightPlan_) flightPlan_->clear();
        // DigiState::reset() intentionally doesn't clear threat pointers
        // (host-managed), but the brain's frameInputs_ injection path means
        // we need to clear them here so a stale injected threat doesn't
        // persist across reset().
        state_.missileDefeat.incomingMissile = nullptr;
        state_.gunsJink.gunsThreat = nullptr;
        state_.threat.threatPtr = nullptr;
        state_.threat.threatTimer = 0.0;
        curWp_ = 0;
        startPosInitialized_ = false;
        curMode_ = DigiMode::Waypoint;
        nextMode_ = DigiMode::NoMode;
        lastMode_ = DigiMode::Waypoint;
        forcedMode_ = DigiMode::NoMode;
        frameInputs_ = FrameInputs{};
        missileEntityAuto_.reset();
        gunsEntityAuto_.reset();
        targetEntityAuto_.reset();
        selfEntityExplicit_ = false;
        wvrTarget_ = nullptr;
        lastInjectedMissilePtr_ = nullptr;  // so next injection is detected as new
        // Clear the SensorFusion picture so stale contacts (bestTarget,
        // incomingMissile, gunsThreat) from a previous scenario don't leak
        // into the next one and trigger spurious mode entries. Without this,
        // a guns scenario that populates bestTarget would cause the next
        // scenario (e.g. digi_rtb) to enter WVREngage instead of RTB.
        sensorFusion_.reset();
        // Clear the SMS pointer — the host owns the SMS object and may
        // destroy it between scenarios. Without this, a stale pointer
        // causes a use-after-free when the next scenario's compute() calls
        // sms_->hasWeaponClass().
        sms_ = nullptr;
    }

    /// Reset per-phase navigation state (call between PHASES within a scenario).
    ///
    /// Unlike reset() (which clears everything including mode and config),
    /// this only clears the navigation integrators and stick commands that
    /// accumulate during a phase and would leak into the next phase:
    ///   - gammaHoldIError (GammaHold integrator)
    ///   - autoThrottle (MachHold integrator)
    ///   - pStick, rStick, yPedal, throttle (smoothed stick commands)
    ///   - speedBrakeCmd, tefCmd, lefCmd
    ///
    /// This is needed because the scenario framework reuses the same brain
    /// across all phases in a scenario. Without this, a Takeoff phase's
    /// wound-up GammaHold integrator and full-throttle stick command carry
    /// over to the Landing phase, causing the aircraft to pitch the wrong
    /// way initially and excite the Phugoid.
    ///
    /// The mode, config, waypoints, and threat state are PRESERVED — only
    /// the transient control state is cleared. Each phase's Init() should
    /// still set up whatever it needs, but it no longer has to manually
    /// clear the previous phase's integrators.
    void resetPhaseState() noexcept {
        if (flightPlan_) flightPlan_->clear();
        if (rootNode_) rootNode_->reset();
        wps_.clear();
        curWp_ = 0;
        startPosInitialized_ = false;
        state_.nav.gammaHoldIError = 0.0;
        state_.nav.autoThrottle = 0.0;
        state_.commands.pStick = 0.0;
        state_.commands.rStick = 0.0;
        state_.commands.yPedal = 0.0;
        state_.commands.throttle = 0.5;
        state_.commands.speedBrakeCmd = -1.0;
        state_.commands.tefCmd = 0.0;
        state_.commands.lefCmd = 0.0;
        state_.commands.wheelBrakes = false;
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
        state_.config.skill = makeSkillParams(level);
    }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setCornerSpeed(double kts) { state_.config.cornerSpeed = kts; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxGs(double g) { state_.config.maxGs = g; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxBank(double deg) { state_.config.maxRoll = deg; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setMaxGamma(double deg) { state_.config.maxGammaDeg = deg; }
    /// @deprecated Use configure(DigiConfig) instead.
    [[deprecated("Use configure(DigiConfig)")]]
    void setTurnG(double lf) { state_.config.turnLoadFactor = lf; }

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
        state_.missileDefeat.incomingMissile = m;
        if (m) {
            // Reset per-missile state on injection.
            state_.missileDefeat.missileDefeatTtgo = -1.0;
            state_.missileDefeat.incomingMissileEvadeTimer = 0.0;
        }
    }
    /// @deprecated Use setFrameInputs(FrameInputs) with injectedGunsThreat.
    [[deprecated("Use setFrameInputs(FrameInputs).injectedGunsThreat")]]
    void setGunsThreat(const DigiEntity* t) {
        frameInputs_.injectedGunsThreat = t;
        state_.gunsJink.gunsThreat = t;
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

    // =======================================================================
    // 10. Behavior Tree and FlightPlan getters/setters
    // =======================================================================
    std::shared_ptr<FlightPlan> flightPlan() const { return flightPlan_; }
    void setFlightPlan(std::shared_ptr<FlightPlan> fp) { flightPlan_ = fp; }

    const Blackboard& blackboard() const { return blackboard_; }
    Blackboard& blackboardMutable() { return blackboard_; }

    const BehaviorNodePtr& rootNode() const { return rootNode_; }

    const std::optional<DigiEntity>& targetEntityAuto() const { return targetEntityAuto_; }
    void setTargetEntityAuto(const DigiEntity& e) { targetEntityAuto_ = e; }
    void clearTargetEntityAuto() { targetEntityAuto_.reset(); }

    const std::optional<DigiEntity>& missileEntityAuto() const { return missileEntityAuto_; }
    void setMissileEntityAuto(const DigiEntity& e) { missileEntityAuto_ = e; }
    void clearMissileEntityAuto() { missileEntityAuto_.reset(); }

    const std::optional<DigiEntity>& gunsEntityAuto() const { return gunsEntityAuto_; }
    void setGunsEntityAuto(const DigiEntity& e) { gunsEntityAuto_ = e; }
    void clearGunsEntityAuto() { gunsEntityAuto_.reset(); }

    MessageBus* bus() { return bus_; }

    const DigiEntity* wvrTarget() const { return wvrTarget_; }
    void setWvrTarget(const DigiEntity* tgt) { wvrTarget_ = tgt; }

    void setCurMode(DigiMode m) { curMode_ = m; }

    DigiMode forcedMode() const { return forcedMode_; }

    void runLegacyMode(const AircraftState& as, double dt,
                        const FlightControlSystem& fcs, FcsState& fcsState,
                        double groundZ, const DigiEntity* selfEntity);

    void buildBehaviorTree();

    friend class ForcedModeNode;
    friend class GroundAvoidNode;
    friend class CollisionAvoidNode;
    friend class MissileDefeatNode;
    friend class GunsJinkNode;
    friend class TakeoffNode;
    friend class LandingNode;
    friend class FollowOrdersNode;
    friend class RTBNode;
    friend class CombatNode;
    friend class WingyNode;
    friend class RefuelNode;
    friend class GroundMnvrNode;
    friend class WaypointFollowNode;
    friend class DefaultFallbackNode;

private:
    // --- Internal state ---
    DigiState state_;
    std::vector<Vec3> wps_;
    std::size_t curWp_{0};
    double captureRadius_{5000.0};  // ft
    bool startPosInitialized_{false};
    Vec3 startPos_{0.0, 0.0, 0.0};

    Blackboard blackboard_;
    BehaviorNodePtr rootNode_;
    std::shared_ptr<FlightPlan> flightPlan_ {std::make_shared<FlightPlan>()};

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

    // Last injected missile pointer — used to detect when the host injects a
    // NEW missile (different pointer) so per-missile state can be reset.
    // Without this, the brain can't tell if the same pointer means "same
    // missile, refresh" or "new missile at the same address."
    const DigiEntity* lastInjectedMissilePtr_{nullptr};

    // --- Priority-stack mode arbitration (port of FF dlogic.cpp:729-790) ---
    // curMode_  : the resolved mode for THIS frame (set by resolveModeConflicts)
    // nextMode_ : the mode being queued for next frame (set by addMode)
    // lastMode_ : the mode from last frame (for newTurn detection)
    //
    // activeMode() accessor returns curMode_ for backward compatibility.
    DigiMode curMode_{DigiMode::Waypoint};    // resolved mode this frame
    DigiMode nextMode_{DigiMode::NoMode};     // mode being queued for next frame
    DigiMode lastMode_{DigiMode::Waypoint};   // mode from last frame (newTurn detection)

    // AddMode — queue a mode for next frame, respecting priority + interlocks.
    // Port of FreeFalcon dlogic.cpp:729-762.
    // Rules:
    //   1. Standard priority: only accept if newMode < nextMode (smaller = higher priority)
    //   2. BugoutMode is sticky: can't be bumped except by MissileDefeat
    //   3. LandingMode can't be bumped by WVR engagements once set
    //   4. Don't alternate between Landing and WVR each frame
    void addMode(DigiMode newMode);

    // Resolve queued mode: copy nextMode -> curMode, reset nextMode.
    // Port of FreeFalcon dlogic.cpp:764-790.
    void resolveModeConflicts();

    DigiMode forcedMode_{DigiMode::NoMode};

    // --- Per-mode actions ---
    // NOTE: there is intentionally no runGroundAvoid(). Ground avoidance is
    // NOT a dispatched mode — it runs as a concurrent overlay in compute()
    // (RunGroundAvoid sets `pullingUp`, and the per-mode switch is skipped
    // entirely while pullingUp is true). The `case DigiMode::GroundAvoid`
    // in the switch is therefore a documented no-op, reachable only via
    // forceMode(GroundAvoid) when there is no terrain danger (in which case
    // doing nothing is correct).
    void runWaypoint(const AircraftState& as, double dt,
                     const FlightControlSystem& fcs, FcsState& fcsState);
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

    // Wingman formation following. Called when Wingy mode is active.
    // Uses AiFollowLead to fly to the wingman's formation slot relative
    // to the injected lead entity.
    void runWingy(const AircraftState& as, double dt,
                  const FlightControlSystem& fcs, FcsState& fcsState);

    // RTB: navigate toward the divert airbase (set by host via
    // FrameInputs.fuel.* or directly via stateMutable()). Falls back to
    // waypoint nav if no divert airbase is set.
    void runRTB(const AircraftState& as, double dt,
                const FlightControlSystem& fcs, FcsState& fcsState);

    // GroundAttack: simplified dive-bomb attack profile. Flies toward the
    // ground target, dives to the release altitude, releases the weapon,
    // pulls out, and egresses. Port of FreeFalcon's GroundAttackMode
    // (gndattck.cpp:96-4900), vastly simplified.
    //
    // Task 15-a: dispatches on state_.ag.agProfile (DiveBomb / LevelDelivery /
    // TossBomb) to one of the three per-profile helper methods below. The host
    // selects the profile via FrameInputs.injectedAgProfile. Defaults to
    // DiveBomb for backward compatibility.
    void runGroundAttack(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState);

    // Refueling: air-to-air refueling. Flies to the tanker's boom position,
    // holds contact for fuel transfer, then disconnects. Port of FreeFalcon's
    // AiRefuel (refuel.cpp:33-200), vastly simplified.
    void runRefueling(const AircraftState& as, double dt,
                       const FlightControlSystem& fcs, FcsState& fcsState);

    // DiveBomb profile: the original Task 11 4-phase dive-bomb attack
    // (approach → dive → pullout → egress). Reuses state_.ag.agApproach as
    // the phase counter (0..3).
    void runDiveBombAttack(const DigiEntity* target,
                            const AircraftState& as, double dt,
                            const FlightControlSystem& fcs, FcsState& fcsState);

    // LevelDelivery profile: Task 15-a level bombing against soft/area
    // targets. 4-phase (approach → level → release → egress). Approaches at
    // 8000 ft / 400 kts, descends to 500 ft AGL within 2 NM, releases when
    // directly over the target, climbs back to 8000 ft on egress.
    void runLevelDeliveryAttack(const DigiEntity* target,
                                 const AircraftState& as, double dt,
                                 const FlightControlSystem& fcs, FcsState& fcsState);

    // TossBomb profile: Task 15-a loft (toss) bombing for standoff delivery.
    // 4-phase (approach → pull-up → release → egress). Approaches at 500 ft
    // AGL / 450 kts, pulls up into a 4G climb within 3 NM, releases at the
    // apex (45° pitch attitude or 3000 ft AGL), continues climbing to
    // 10000 ft before leveling off.
    void runTossBombAttack(const DigiEntity* target,
                            const AircraftState& as, double dt,
                            const FlightControlSystem& fcs, FcsState& fcsState);

    // FollowOrders: execute the wingman's current tactical maneuver
    // (BreakLeft/Right, ClearSix, Posthole, Chainsaw). Dispatches to
    // AiPerformManeuver. If no maneuver is active, falls back to Wingy
    // (formation following).
    void runFollowOrders(const AircraftState& as, double dt,
                         const FlightControlSystem& fcs, FcsState& fcsState);

    // Resolve the active mode based on priority + threats.
    void resolveMode(const AircraftState& as, double groundZ, double dt);
};

} // namespace digi
} // namespace f4flight
