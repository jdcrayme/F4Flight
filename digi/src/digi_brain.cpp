// f4flight - digi/digi_brain.cpp
//
// DigiBrain — the top-level digi AI dispatcher.
//
// Port of FreeFalcon digimain.cpp:566 (DigitalBrain::FrameExec).
//
// FrameExec pipeline:
//   1. GroundCheck (always runs — pre-empts everything)
//   2. ResolveMode (priority stack)
//   3. Actions (dispatch to active mode's maneuver function)
//   4. Clamp outputs
//
// Tier 0-1 status:
//   - GroundAvoid:  IMPLEMENTED (ground_avoid.cpp)
//   - MissileDefeat: IMPLEMENTED (missile_defeat.cpp)
//   - GunsJink:     IMPLEMENTED (guns_jink.cpp)
//   - Waypoint:     IMPLEMENTED (delegates to ManeuverPrimitives)
//   - WVREngage:    IMPLEMENTED (roll_and_pull.cpp)

#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/steering.h"  // for headingError
#include "f4flight/digi/behavior_tree/brain_bt_nodes.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/decision/flight_lead.h"  // FlightLeadDecisions
#include "f4flight/flight/core/airspeed_conversions.h"  // casFromTasFps
#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/digi/defensive/collision_avoid.h"
#include "f4flight/digi/defensive/handle_threat.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/offensive/missile_engage.h"
#include "f4flight/digi/offensive/bvr_engage.h"
#include "f4flight/digi/offensive/merge.h"
#include "f4flight/digi/wingman/wingman_ai.h"
#include "f4flight/digi/wingman/wingman_maneuvers.h"
#include "f4flight/digi/decision/decision_routines.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/flight/core/airspeed_conversions.h"  // cas_kts (typed machHoldCas)
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// Constructor
// ===========================================================================
DigiBrain::DigiBrain() {
    state_.reset();
    state_.config.skill = makeSkillParams(SkillLevel::Veteran);
    buildBehaviorTree();
}

// ===========================================================================
// Configuration
// ===========================================================================
void DigiBrain::configure(const DigiConfig& cfg) {
    state_.config.skill          = makeSkillParams(cfg.skillLevel);
    state_.config.cornerSpeed    = cfg.cornerSpeedKts;
    state_.config.maxGs          = cfg.maxGs;
    state_.config.maxRoll        = cfg.maxBankDeg;
    state_.config.maxGammaDeg    = cfg.maxGammaDeg;
    state_.config.turnLoadFactor = cfg.turnLoadFactor;
}

DigiConfig DigiBrain::config() const {
    DigiConfig cfg;
    cfg.skillLevel     = state_.config.skill.level;
    cfg.cornerSpeedKts = state_.config.cornerSpeed;
    cfg.maxGs          = state_.config.maxGs;
    cfg.maxBankDeg     = state_.config.maxRoll;
    cfg.maxGammaDeg    = state_.config.maxGammaDeg;
    cfg.turnLoadFactor = state_.config.turnLoadFactor;
    return cfg;
}

// ===========================================================================
// buildSelfEntity
// ===========================================================================
DigiEntity DigiBrain::buildSelfEntity(const AircraftState& as) {
    DigiEntity e;
    e.x = as.kin.x;
    e.y = as.kin.y;
    e.z = as.kin.z;
    e.vx = as.kin.xdot;
    e.vy = as.kin.ydot;
    e.vz = as.kin.zdot;
    e.yaw = as.kin.sigma;    // velocity-vector heading
    e.pitch = as.kin.gmma;   // flight path angle
    e.roll = as.kin.phi;     // body roll
    e.speed = as.kin.vt;     // true airspeed (ft/s)
    e.dcm = as.kin.dcm;      // body-to-world DCM for full 3D geometry
    return e;
}

// ===========================================================================
// Commands (asynchronous)
// ===========================================================================
void DigiBrain::commandTakeoff(RunwayId rwy, double rwyHeading,
                                double rwyThresholdX, double rwyThresholdY,
                                double rwyAlt) {
    auto& go = state_.ag.groundOps;
    go.assignedRunway = rwy;
    go.runwayHeading = rwyHeading;
    go.runwayThresholdX = rwyThresholdX;
    go.runwayThresholdY = rwyThresholdY;
    go.runwayAltitude = rwyAlt;
    go.phase = GroundOpsPhase::TakeoffRoll;
    go.gearRetracted = false;
    go.hasTakeoffClearance = true;  // simplified: auto-clear (ATC can deny via message)
}

void DigiBrain::commandLanding(RunwayId rwy, double rwyHeading,
                                double rwyThresholdX, double rwyThresholdY,
                                double rwyAlt) {
    auto& go = state_.ag.groundOps;
    go.assignedRunway = rwy;
    go.runwayHeading = rwyHeading;
    go.runwayThresholdX = rwyThresholdX;
    go.runwayThresholdY = rwyThresholdY;
    go.runwayAltitude = rwyAlt;
    go.phase = GroundOpsPhase::Approach;
    go.gearDeployed = false;
    go.hasLandingClearance = true;
}

// ===========================================================================
// compute — main per-frame entry point
// ===========================================================================
PilotInput DigiBrain::compute(const AircraftState& as, double dt, double groundZ,
                               const FlightControlSystem& fcs, FcsState& fcsState) {
    PilotInput out;
    state_.nav.dt = dt;
    simTime_ += dt;

    // --- Clear fire flags at frame start (FF digimain.cpp:599) ---
    state_.weapon.gunFireFlag = false;
    state_.weapon.mslFireFlag = false;

    // --- Resolve self entity for this frame ---
    // Use host-injected selfEntity if provided; otherwise auto-build from
    // AircraftState. selfEntityAuto_ is a member so its address is stable
    // across the frame.
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) {
        selfEntityAuto_ = buildSelfEntity(as);
        selfEntity = &selfEntityAuto_;
    }

    // --- Apply injected threats/target from frameInputs_ ---
    // Host-injected values (via setFrameInputs) take priority over
    // SensorPicture and over any stale state_ pointers from last frame.
    if (frameInputs_.injectedMissile) {
        // Detect missile identity change by pointer value. The host provides
        // a raw const DigiEntity* — if the pointer differs from last frame,
        // it's a new missile and we must reset per-missile state.
        //
        // CONTRACT FIX: previously the SteeringController's setIncomingMissile
        // bypassed setFrameInputs and wrote directly to stateMutable() to
        // force this reset. Now the brain handles it here, so the
        // SteeringController can be a pure facade.
        const bool is_new_missile = (frameInputs_.injectedMissile != lastInjectedMissilePtr_);
        state_.missileDefeat.incomingMissile = frameInputs_.injectedMissile;
        if (is_new_missile) {
            state_.missileDefeat.missileDefeatTtgo = -1.0;
            state_.missileDefeat.incomingMissileEvadeTimer = 0.0;
            // Clear the auto-tracked missile entity (if any) so the brain
            // doesn't confuse injected with auto-tracked.
            missileEntityAuto_.reset();
        }
        lastInjectedMissilePtr_ = frameInputs_.injectedMissile;
    } else {
        // No injected missile this frame — clear the last-pointer so the
        // next injection is detected as new.
        lastInjectedMissilePtr_ = nullptr;
    }

    if (frameInputs_.injectedGunsThreat) {
        state_.gunsJink.gunsThreat = frameInputs_.injectedGunsThreat;
    }

    // --- Run sensor fusion if truth state is provided ---
    // This builds a SensorPicture the brain uses for autonomous detection.
    // If injected threats are also set, resolveMode() gives them priority.
    const TruthState* truth = frameInputs_.truth;
    if (truth && selfEntity) {
        sensorFusion_.update(*selfEntity, *truth, state_.config.skill, dt);

        // Round 7 (P1): Apply GCI + NCTR to the sensor picture.
        // GCI: skill-gated detection beyond sensor range (veteran/ace only).
        // NCTR: radar-based type identification at close range.
        SensorPicture& pic = sensorFusion_.picture();
        ApplyGCI(state_, pic, *selfEntity);
        ApplyNCTR(state_, pic, *selfEntity,
                  /*hasNCTR=*/true,          // host would configure this
                  /*maxNctrRangeFt=*/60.0 * 6076.0);  // FF default: 60 NM
    }

    // Round 7 (P1): chooseRadarMode — translate radModeSelect → radarMode.
    // Throttled internally; called every frame but only re-evaluates every
    // (4 + (4 - skill)) seconds.
    ChooseRadarMode(state_, simTime_, /*hasTWS=*/true);  // host would configure TWS

    // Process incoming messages (ATC clearances, flight commands)
    ProcessATCMessages(state_, state_.comm.mailbox);

    // --- 1. Ground avoidance ---
    // FreeFalcon explicitly disables GroundCheck during LandingMode AND
    // TakeoffMode (dlogic.cpp:49-52: the ATC state machine owns terrain
    // logic for both). TakeoffRoll/Rotation/AfterTakeoff operate on the
    // ground or in the early climbout where the aircraft is below the 500 ft
    // kMinClearance threshold — GroundCheck would trigger a PullUp every
    // frame, fighting the takeoff roll with max-G pitch commands and
    // producing the erratic pitch/yaw seen at takeoff start.
    //
    // We check state_.ag.groundOps.phase (set by commandTakeoff/commandLanding)
    // rather than curMode_ (which hasn't been resolved yet this frame).
    const bool isGroundOps = (state_.ag.groundOps.phase == GroundOpsPhase::Parking ||
                              state_.ag.groundOps.phase == GroundOpsPhase::RequestTaxi ||
                              state_.ag.groundOps.phase == GroundOpsPhase::TaxiToRunway ||
                              state_.ag.groundOps.phase == GroundOpsPhase::HoldingShort ||
                              state_.ag.groundOps.phase == GroundOpsPhase::LiningUp ||
                              state_.ag.groundOps.phase == GroundOpsPhase::TakeoffRoll ||
                              state_.ag.groundOps.phase == GroundOpsPhase::Rotation ||
                              state_.ag.groundOps.phase == GroundOpsPhase::AfterTakeoff ||
                              state_.ag.groundOps.phase == GroundOpsPhase::Approach ||
                              state_.ag.groundOps.phase == GroundOpsPhase::Flare ||
                              state_.ag.groundOps.phase == GroundOpsPhase::Touchdown ||
                              state_.ag.groundOps.phase == GroundOpsPhase::Rollout ||
                              state_.ag.groundOps.phase == GroundOpsPhase::VacatingRunway);

    bool pullingUp = false;
    if (isGroundOps) {
        // Ground ops owns terrain logic — suppress ground avoid.
        state_.groundAvoid.groundAvoidNeeded = false;
        state_.groundAvoid.pullupTimer = 0.0;
    } else {
        pullingUp = RunGroundAvoid(state_, as, groundZ,
                                   state_.config.cornerSpeed, dt,
                                   fcsState, state_.config.maxGs);
    }

    // --- 2. Resolve mode ---
    // Pass the resolved selfEntity so resolveMode doesn't need to re-resolve.
    resolveMode(as, groundZ, dt);

    // --- 3. Actions ---
    if (!pullingUp) {
        // HandleThreat overlay (port of FF digimain.cpp Actions() at 635+).
        // FF: "if (threatPtr && curMode != MissileDefeatMode) HandleThreat();"
        // HandleThreat returns true when it engaged the threat this frame —
        // in that case we skip the normal per-mode switch (the threat IS
        // the action for this frame). MissileDefeat pre-empts HandleThreat
        // because an incoming missile is more urgent than a secondary threat.
        const bool threatHandled =
            (curMode_ != DigiMode::MissileDefeat) &&
            HandleThreat(state_, *selfEntity, as, fcs, fcsState, dt);

        if (!threatHandled) {
            // Populate the Blackboard
            blackboard_.as = &as;
            blackboard_.dt = dt;
            blackboard_.groundZ = groundZ;
            blackboard_.fcs = &fcs;
            blackboard_.fcsState = &fcsState;
            blackboard_.simTime = simTime_;
            blackboard_.state = &state_;
            blackboard_.self = selfEntity;
            blackboard_.target = wvrTarget_;
            blackboard_.flightPlan = flightPlan_;
            blackboard_.brain = this;

            // Execute the Behavior Tree root!
            if (rootNode_) {
                rootNode_->tick(blackboard_);
            } else {
                runWaypoint(as, dt, fcs, fcsState);
            }
        }  // end if (!threatHandled)
    }

    // --- 4. Clamp outputs + map fire flags ---
    out.pstick = limit(state_.commands.pStick, -1.0, 1.0);
    out.rstick = limit(state_.commands.rStick, -1.0, 1.0);
    out.ypedal = limit(state_.commands.yPedal, -1.0, 1.0);
    out.throttle = limit(state_.commands.throttle, 0.0, 1.5);
    // Round-2 structural fix (Rec 7): map digi brake / speed-brake / gear
    // commands to PilotInput. Previously the brain wrote only pStick/
    // rStick/yPedal/throttle — PilotInput.wheelBrakes / speedBrake /
    // gearHandle were dead fields. Now RunLanding::Rollout can actually
    // command wheel brakes, and a future A/G dive-bomb mode can command
    // speed brakes.
    out.wheelBrakes  = state_.commands.wheelBrakes;
    out.parkingBrake = state_.commands.parkingBrake;
    out.speedBrake   = state_.commands.speedBrakeCmd;
    out.gearHandle   = state_.commands.gearHandleCmd;
    out.tefCmd       = state_.commands.tefCmd;
    out.lefCmd       = state_.commands.lefCmd;
    out.refueling = false;
    // Map digi fire flags to PilotInput (host reads these to fire weapons)
    out.fireGun        = state_.weapon.gunFireFlag;
    out.releaseConsent = state_.weapon.mslFireFlag;
    out.weaponStation  = state_.weapon.fireStation;
    return out;
}

// ===========================================================================
// resolveMode — priority-stack mode arbitration
//
// Port of FreeFalcon dlogic.cpp RunDecisionRoutines (lines 729-790).
//
// Architecture:
//   1. If forcedMode_ set (testing override), curMode_ = forcedMode_, return.
//   2. Otherwise, reset nextMode_ and walk the priority-ordered check
//      sequence, calling addMode() for each match. addMode() enforces
//      the FreeFalcon interlock rules (Bugout sticky, Landing↔WVR
//      interlock, standard < priority).
//   3. addMode(Waypoint) is always queued last as the lowest-priority
//      fallback (matching FF RunDecisionRoutines ending).
//   4. resolveModeConflicts() copies nextMode_ → curMode_ and snapshots
//      lastMode_ for next frame's newTurn detection.
//
// Behavior preservation:
//   The check order is the SAME as the previous flat if-else chain. Because
//   addMode() only accepts a new mode if it has higher priority (smaller
//   value) than the currently-queued nextMode_, and the checks are ordered
//   highest-priority-first, the FIRST matching check still wins — exactly
//   as the old early-return chain behaved.
//
//   The BVR-vs-WVR overlap region (8..35 NM) is preserved by structuring the
//   offensive block as `if (BvrEngageCheck) ... else if (range < maxAAWpnRange)`
//   so when BVR matches, the WVR sub-branch is skipped entirely (matching
//   the old early return). Without this, addMode(WVREngage) would override
//   addMode(BVREngage) since WVREngage has higher priority.
// ===========================================================================
void DigiBrain::resolveMode(const AircraftState& /*as*/, double /*groundZ*/,
                             double dt) {
    // If a mode is forced (testing), use it directly — bypass the priority
    // stack and interlocks. (FF: forcedMode is a debug/scripting override.)
    if (forcedMode_ != DigiMode::NoMode) {
        curMode_ = forcedMode_;
        return;
    }

    // Reset the queue for this frame's resolution pass.
    nextMode_ = DigiMode::NoMode;

    // --- Resolve self entity ---
    // (Same logic as compute() — kept inline to avoid passing it through.)
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) {
        selfEntity = &selfEntityAuto_;  // built in compute() this frame
    }

    const SensorPicture& pic = sensorFusion_.picture();
    const bool sensorFusionActive = (frameInputs_.truth != nullptr);

    // ===================================================================
    // --- Incoming missile ---
    // ===================================================================
    // Resolution order:
    //   a. If the host injected a missile (frameInputs_.injectedMissile),
    //      use it directly. The host manages its lifetime.
    //   b. Otherwise, if SensorFusion is active, auto-track the missile
    //      from pic.incomingMissile (sticky-track by entityId).
    //   c. Otherwise, fall back to whatever state_.missileDefeat.incomingMissile points
    //      to (set by deprecated shim or prior frame).
    //
    // We COMMIT the pointer to state_.missileDefeat.incomingMissile so that
    // runMissileDefeat sees it, and MissileDefeatCheck's clearing
    // side-effect persists.

    // Step a: host-injected missile
    if (frameInputs_.injectedMissile) {
        state_.missileDefeat.incomingMissile = frameInputs_.injectedMissile;
        // Clear auto-track so we don't fight the injection.
        missileEntityAuto_.reset();
        // Don't update incomingMissileId for injected missiles — the host
        // manages identity. (If the host wants per-missile state reset,
        // they should call reset() or use the deprecated shim which does it.)
    }
    // Step b: auto-track from SensorFusion
    else {
        const bool autoTracking =
            missileEntityAuto_.has_value() &&
            state_.missileDefeat.incomingMissile == &(*missileEntityAuto_);

        // Clear auto-tracked missile if SensorFusion no longer sees one.
        if (sensorFusionActive && autoTracking && !pic.incomingMissile) {
            state_.missileDefeat.incomingMissile = nullptr;
            state_.missileDefeat.incomingMissileId = kInvalidEntityId;
            missileEntityAuto_.reset();
        }
        // SensorFusion sees a missile we're not tracking (or a different one).
        else if (sensorFusionActive && pic.incomingMissile &&
                 (!autoTracking ||
                  pic.incomingMissile->entityId != state_.missileDefeat.incomingMissileId)) {
            // BUG FIX: use the toDigiEntity helper instead of inline
            // field-by-field copy. The previous code copied 9 fields and
            // hardcoded seekerType=Radar, missing pitch/roll/isFiring.
            missileEntityAuto_ = toDigiEntity(*pic.incomingMissile);
            state_.missileDefeat.incomingMissile = &(*missileEntityAuto_);

            // Reset per-missile state on identity change.
            const EntityId newId = pic.incomingMissile->entityId;
            if (newId != state_.missileDefeat.incomingMissileId) {
                state_.missileDefeat.incomingMissileId = newId;
                state_.missileDefeat.missileDefeatTtgo = -1.0;
                state_.missileDefeat.incomingMissileEvadeTimer = 0.0;
            }
        }
        // Same missile — refresh position/velocity.
        else if (sensorFusionActive && autoTracking && pic.incomingMissile &&
                 pic.incomingMissile->entityId == state_.missileDefeat.incomingMissileId) {
            // BUG FIX: use the helper here too, so pitch/roll/isFiring stay
            // current (the previous code only refreshed 8 fields).
            missileEntityAuto_ = toDigiEntity(*pic.incomingMissile);
        }
    }

    if (selfEntity && state_.missileDefeat.incomingMissile) {
        if (MissileDefeatCheck(state_, *selfEntity, dt)) {
            addMode(DigiMode::MissileDefeat);
        }
    }

    // ===================================================================
    // --- Guns threat ---
    // ===================================================================
    if (frameInputs_.injectedGunsThreat) {
        state_.gunsJink.gunsThreat = frameInputs_.injectedGunsThreat;
        gunsEntityAuto_.reset();
    } else {
        // Clear auto-tracked guns threat if SensorFusion no longer sees one.
        if (sensorFusionActive && gunsEntityAuto_.has_value() &&
            state_.gunsJink.gunsThreat == &(*gunsEntityAuto_) &&
            !pic.gunsThreat) {
            state_.gunsJink.gunsThreat = nullptr;
            gunsEntityAuto_.reset();
        }

        if (!state_.gunsJink.gunsThreat && pic.gunsThreat) {
            // BUG FIX: use toDigiEntity helper instead of inline copy that
            // hardcoded isFiring=true and missed pitch/roll/seekerType.
            gunsEntityAuto_ = toDigiEntity(*pic.gunsThreat);
            state_.gunsJink.gunsThreat = &(*gunsEntityAuto_);
        }
    }

    if (selfEntity && state_.gunsJink.gunsThreat) {
        if (GunsJinkCheck(state_, *selfEntity)) {
            addMode(DigiMode::GunsJink);
        }
    }

    // ===================================================================
    // --- Collision avoidance ---
    // ===================================================================
    // FF dlogic.cpp:CollisionCheck runs in RunDecisionRoutines after
    // SeparateCheck + AirbaseCheck, before the offensive checks. It
    // extrapolates the current target's velocity vector and enters
    // CollisionAvoid mode if a mid-air is predicted.
    // We check the resolved WVR target (injected or auto-tracked).
    const DigiEntity* collTarget = frameInputs_.injectedTarget;
    if (!collTarget) collTarget = wvrTarget_;
    if (selfEntity && collTarget && !collTarget->isDead) {
        if (CollisionCheck(state_, *selfEntity, *collTarget)) {
            addMode(DigiMode::CollisionAvoid);
        }
    }

    // ===================================================================
    // --- Ground ops (takeoff/landing) ---
    // ===================================================================
    const auto gp = state_.ag.groundOps.phase;
    if (gp == GroundOpsPhase::TakeoffRoll || gp == GroundOpsPhase::Rotation ||
        gp == GroundOpsPhase::AfterTakeoff || gp == GroundOpsPhase::LiningUp ||
        gp == GroundOpsPhase::TaxiToRunway || gp == GroundOpsPhase::HoldingShort ||
        gp == GroundOpsPhase::Parking || gp == GroundOpsPhase::RequestTaxi) {
        addMode(DigiMode::Takeoff);
    }
    if (gp == GroundOpsPhase::Approach || gp == GroundOpsPhase::Flare ||
        gp == GroundOpsPhase::Touchdown || gp == GroundOpsPhase::Rollout ||
        gp == GroundOpsPhase::VacatingRunway) {
        addMode(DigiMode::Landing);
    }

    // ===================================================================
    // --- Fuel state → RTB (port of FF separate.cpp FuelCheck + AirbaseCheck) ---
    // ===================================================================
    // Without this, AI fly until flameout. We transition fuel.phase each
    // frame based on fuelLbs vs the bingo/joker/fumes thresholds, then queue
    // RTB when fuel <= bingo (or when winchester AND we're not actively
    // engaging).
    //
    // RTB priority is between Landing (sticky) and MissileEngage — i.e. RTB
    // pre-empts offensive modes (we're going home, not fighting) but NOT
    // defensive modes (still have to defeat missiles / jink guns / avoid
    // terrain) and NOT landing (the landing state machine owns the approach).
    //
    // addMode() priority resolution means: if MissileDefeat/GunsJink/Collision
    // is already queued, RTB loses (correct — survive first, RTB second). If
    // no defensive mode is queued, RTB pre-empts Waypoint/WVR/BVR.
    {
        // Transition fuel state machine.
        if (state_.fuel.fumesFuelLbs > 0.0 && state_.fuel.fuelLbs <= state_.fuel.fumesFuelLbs) {
            state_.fuel.phase = DigiFuelState::Phase::Fumes;
            state_.damage.saidFumes = true;
        } else if (state_.fuel.bingoFuelLbs > 0.0 && state_.fuel.fuelLbs <= state_.fuel.bingoFuelLbs) {
            state_.fuel.phase = DigiFuelState::Phase::Bingo;
            state_.damage.saidBingo = true;
            // Radio call: "Bingo" (once-only)
            {
                const uint32_t bit = 1u << static_cast<uint32_t>(RadioCallType::Bingo);
                if (!(state_.comm.callsMade & bit)) {
                    if (makeRadioCall(state_.comm.radioCalls, RadioCallType::Bingo,
                                       simTime_, state_.comm.selfId)) {
                        state_.comm.callsMade |= bit;
                    }
                }
            }
        } else if (state_.fuel.jokerFuelLbs > 0.0 && state_.fuel.fuelLbs <= state_.fuel.jokerFuelLbs) {
            state_.fuel.phase = DigiFuelState::Phase::Joker;
            // Radio call: "Joker" (once-only)
            {
                const uint32_t bit = 1u << static_cast<uint32_t>(RadioCallType::Joker);
                if (!(state_.comm.callsMade & bit)) {
                    if (makeRadioCall(state_.comm.radioCalls, RadioCallType::Joker,
                                       simTime_, state_.comm.selfId)) {
                        state_.comm.callsMade |= bit;
                    }
                }
            }
        } else {
            state_.fuel.phase = DigiFuelState::Phase::Normal;
        }

        const bool fuelCritical =
            (state_.fuel.phase == DigiFuelState::Phase::Bingo ||
             state_.fuel.phase == DigiFuelState::Phase::Fumes ||
             state_.fuel.phase == DigiFuelState::Phase::Flameout);
        // Winchester: out of A/A weapons. RTB if we're not actively
        // defending (no missile / guns threat) — there's nothing to fight
        // with anyway.
        const bool winchesterRTB =
            state_.fuel.winchester &&
            !state_.missileDefeat.incomingMissile &&
            !state_.gunsJink.gunsThreat;

        if (fuelCritical || winchesterRTB) {
            // Round 6: AirbaseCheck auto-picks the nearest friendly airbase
            // and transitions RTB → Landing when within range. If the host
            // hasn't provided an airbase list, AirbaseCheck returns None
            // and we fall back to plain RTB (waypoint-based).
            if (selfEntity) {
                const AirbaseAction abAction = AirbaseCheck(state_, *selfEntity,
                                                             frameInputs_, simTime_);
                if (abAction == AirbaseAction::Landing) {
                    addMode(DigiMode::Landing);
                } else if (abAction == AirbaseAction::RTB || fuelCritical || winchesterRTB) {
                    addMode(DigiMode::RTB);
                }
            } else {
                addMode(DigiMode::RTB);
            }
        }
    }

    // ===================================================================
    // --- SeparateCheck + Bugout (Round 6: port of FF separate.cpp) ---
    // ===================================================================
    // Disengage logic: damage abort, deep-six bugout, lateral separation.
    // Runs AFTER fuel check (so RTB from fuel pre-empts Separate) and
    // BEFORE offensive checks (so disengage pre-empts engage).
    //
    // Separate (14) and Bugout (21) are lower priority than RTB (19) in the
    // enum, but we queue them conditionally — only when SeparateCheck
    // recommends them AND fuel isn't already driving RTB. addMode() priority
    // resolution handles the rest.
    {
        const DigiEntity* sepTarget = wvrTarget_;
        if (!sepTarget) sepTarget = frameInputs_.injectedTarget;
        if (selfEntity) {
            const SeparateAction sepAction = SeparateCheck(state_, *selfEntity,
                                                             sepTarget, dt);
            switch (sepAction) {
                case SeparateAction::RTB:
                    // Damage abort → RTB (may already be queued by fuel check;
                    // addMode priority resolution handles the dedup).
                    addMode(DigiMode::RTB);
                    break;
                case SeparateAction::Separate:
                    addMode(DigiMode::Separate);
                    break;
                case SeparateAction::Bugout:
                    addMode(DigiMode::Bugout);
                    break;
                case SeparateAction::None:
                    break;
            }
        }
    }

    // ===================================================================
    // --- FlightLeadDecisions (flight-lead tactical decisions) ---
    // ===================================================================
    // If this aircraft is a flight lead (not a wingman), make tactical
    // decisions: engage/disengage, target prioritization, formation
    // management. This sets state that the rest of resolveMode() uses.
    // Port of FreeFalcon's flitlead.cpp + dlogic.cpp lead-specific parts.
    {
        const DigiEntity* leadTarget = wvrTarget_;
        if (!leadTarget) leadTarget = frameInputs_.injectedTarget;
        if (selfEntity) {
            FlightLeadDecisions(state_, *selfEntity, leadTarget,
                                sensorFusion_.picture(), dt);
        }
    }

    // ===================================================================
    // --- CommandFlight (Round 6: flight-lead issues orders to wingmen) ---
    // ===================================================================
    // If this aircraft is a flight lead (not a wingman), issue engage/rejoin
    // orders to its wingmen via the MessageBus. This is a side-effecting
    // call (it publishes messages); it doesn't queue any mode for this brain.
    // Throttled to one order per 5 seconds (matches FF).
    {
        const DigiEntity* cmdTarget = wvrTarget_;
        if (!cmdTarget) cmdTarget = frameInputs_.injectedTarget;
        CommandFlight(state_, cmdTarget, bus_, state_.comm.selfId, simTime_);
    }

    // ===================================================================
    // --- WVR target (offensive) ---
    // ===================================================================
    // Priority: injected target > SensorPicture bestTarget > DoTargeting
    const DigiEntity* tgt = frameInputs_.injectedTarget;
    if (tgt && tgt->isDead) tgt = nullptr;

    // Clear auto-tracked target if SensorFusion no longer sees one.
    if (sensorFusionActive && targetEntityAuto_.has_value() &&
        tgt == &(*targetEntityAuto_) &&
        !pic.bestTarget) {
        tgt = nullptr;
        targetEntityAuto_.reset();
    }

    if (!tgt && pic.bestTarget) {
        // BUG FIX: use toDigiEntity helper for consistency with the missile
        // and guns paths. The previous inline copy missed seekerType and
        // isFiring.
        targetEntityAuto_ = toDigiEntity(*pic.bestTarget);
        tgt = &(*targetEntityAuto_);
    }

    // Round 7 (P1): DoTargeting — autonomous target selection.
    // If no injected target AND no SensorPicture bestTarget, try DoTargeting.
    // This scans the SensorPicture for the highest-threat aircraft contact
    // and returns it as the target. Without this, the brain can't find
    // targets autonomously — it relies on the host injecting them.
    if (!tgt && sensorFusionActive && selfEntity) {
        const DigiEntity* autoTarget = DoTargeting(state_, pic, *selfEntity);
        if (autoTarget) {
            tgt = autoTarget;
        }
    }

    if (selfEntity && tgt && !tgt->isDead) {
        const double dx = tgt->x - selfEntity->x;
        const double dy = tgt->y - selfEntity->y;
        const double dz = tgt->z - selfEntity->z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        // --- Resolve max A/A weapon range ---
        // FF mengage.cpp:318: maxAAWpnRange starts at 6000 (gun) or 0, then
        // is extended by each missile's RMax. We default to the gun range
        // (6000 ft) if the host hasn't set up an SMS; if missiles are
        // available, use the AIM-120 RMax (35 NM) as the BVR gate.
        // TODO: read from SMS when host provides one
        const double maxAAWpnRangeFt = state_.weapon.maxAAWpnRange > 0
            ? state_.weapon.maxAAWpnRange
            : 35.0 * 6076.0;  // default: AIM-120 RMax (35 NM)

        // --- BVR engagement (8 NM .. engageRange) ---
        // FF bvrengage.cpp:46-216: enter BvrEngage when the target is beyond
        // the RAP distance (8 NM) and within engageRange = max(maxAAWpnRange
        // × 1.3, 45 NM). This is checked BEFORE the `range < maxAAWpnRangeFt`
        // guard below so BVR can engage out to 45 NM even when the host's
        // maxAAWpnRange is only 35 NM — previously the inline `range > 8 NM`
        // check was nested inside `range < maxAAWpnRangeFt`, which silently
        // capped BVR at 35 NM and left BvrEngageCheck() as dead code.
        // Inside 8 NM, BvrEngageCheck returns false and the WVR-family checks
        // (Missile/Merge/Guns/WVR) below handle the engagement.
        //
        // The if/else-if structure here is CRITICAL: when BVR matches, the
        // WVR sub-branch is skipped entirely. Without this, addMode(WVREngage)
        // (priority 10) would override addMode(BVREngage) (priority 11) in
        // the 8..35 NM overlap region, since WVREngage has higher priority.
        // (The old code's early return achieved the same effect.)
        if (BvrEngageCheck(state_, *selfEntity, *tgt, maxAAWpnRangeFt)) {
            wvrTarget_ = tgt;
            addMode(DigiMode::BVREngage);
        }
        else if (range < maxAAWpnRangeFt) {
            // --- MissileEngage check (within 8 NM, beyond gun range) ---
            // Only enter MissileEngage if we have an SMS with actual missiles.
            if (sms_ && sms_->hasWeaponClass(WeaponClass::AimWpn)) {
                if (MissileEngageCheck(state_, *selfEntity, *tgt, *sms_, true) &&
                    range > 3500.0) {
                    wvrTarget_ = tgt;
                    addMode(DigiMode::MissileEngage);
                }
            }

            // --- Merge check (very close, nose-to-nose) ---
            // FF merge.cpp:9-52: enter Merge when range ≤ ~1000 ft, ata < 45°
            if (MergeCheck(state_, *selfEntity, *tgt)) {
                wvrTarget_ = tgt;
                addMode(DigiMode::Merge);
            }

            // --- Accel check (too slow in combat) ---
            // AccelCheck needs AircraftState for vcas, but resolveMode
            // doesn't receive it. Accel is handled inside the per-mode
            // runners (Merge, WVREngage) instead.

            // --- GunsEngage check (close range) ---
            WeaponSpec gun = gunSpec();  // default: M61, 510 rounds
            if (GunsEngageCheck(state_, *selfEntity, *tgt, gun, true)) {
                wvrTarget_ = tgt;
                addMode(DigiMode::GunsEngage);
            }

            // --- WVREngage (default offensive mode within 8 NM) ---
            // Always queued as the offensive fallback within maxAAWpnRange.
            // addMode() priority resolution ensures any higher-priority
            // offensive mode (MissileEngage/Merge/GunsEngage) queued above
            // wins; if none matched, WVREngage wins.
            wvrTarget_ = tgt;
            addMode(DigiMode::WVREngage);
        }
    }

    // --- Wingman tactical maneuver (FollowOrders mode) ---
    // If this aircraft is a wingman AND has an active maneuver (set by
    // receiveOrders in response to a FlightCmdBreak / FlightCmdClearSix etc.
    // message), queue FollowOrders mode. FollowOrders (18) is lower priority
    // than Wingy (20) in the enum, but we queue it BEFORE Wingy so it wins
    // when both could apply (the wingman executes the maneuver instead of
    // holding formation). When the maneuver completes (mnverTime <= 0 or
    // AiExec* returns false), AiClearManeuver sets currentManeuver=None and
    // the brain naturally falls back to Wingy on the next frame.
    //
    // FreeFalcon winglogic.cpp: AiCheckManeuvers queues FollowOrdersMode
    // when mpActionFlags[AI_EXECUTE_MANEUVER] is set and currentManeuver
    // is not WMSNone.
    const bool followOrdersActive =
        state_.formation.isWing &&
        state_.formation.wingman.currentManeuver != WingmanManeuver::None;
    if (followOrdersActive) {
        // If mnverTime is 0 (maneuver just initiated), arm it to the default.
        // The host (or receiveOrders) can set a custom duration by writing
        // nav.mnverTime directly before the maneuver starts.
        if (state_.nav.mnverTime <= 0.0) {
            state_.nav.mnverTime = kDefaultManeuverTimeSec;
        }
        addMode(DigiMode::FollowOrders);
    }

    // --- SEAD Target of Opportunity (TOO) and Self Protect (SP) target resolution ---
    if (state_.ag.agProfile == AgAttackProfile::SeadHarm && sensorFusionActive && selfEntity) {
        if (state_.ag.htsMode == HtsMode::SelfProtect) {
            // SP Mode: target the source of any active spike
            if (pic.spiked) {
                const SensorContact* threatContact = nullptr;
                double closestThreatDist = std::numeric_limits<double>::infinity();
                for (const auto& c : pic.contacts) {
                    if (c.detectedBy(SensorType::RWR) && c.isThreat) {
                        const RelativeGeometry rg = computeRelativeGeometry(*selfEntity,
                            DigiEntity{c.x, c.y, c.z, c.vx, c.vy, c.vz, c.yaw, c.pitch, c.roll, c.speed});
                        if (rg.range < closestThreatDist) {
                            closestThreatDist = rg.range;
                            threatContact = &c;
                        }
                    }
                }
                // Fallback to highest threat if we are spiked but didn't find specific RWR threat
                if (!threatContact && pic.highestThreat) {
                    threatContact = pic.highestThreat;
                }
                if (threatContact) {
                    // Update/set ground target dynamically from the threat contact
                    groundTargetAuto_ = toDigiEntity(*threatContact);
                    state_.ag.groundTarget = &*groundTargetAuto_;
                    state_.ag.groundTargetId = threatContact->entityId;
                }
            } else {
                // If not spiked, clear the auto-detected threat target
                if (state_.ag.groundTarget == &*groundTargetAuto_) {
                    state_.ag.groundTarget = nullptr;
                    state_.ag.groundTargetId = kInvalidEntityId;
                    groundTargetAuto_.reset();
                }
            }
        } else if (state_.ag.htsMode == HtsMode::TargetOfOpportunity) {
            // TOO Mode: target any active SAM site or emitting radar contact
            const SensorContact* samContact = nullptr;
            double closestSamDist = std::numeric_limits<double>::infinity();
            for (const auto& c : pic.contacts) {
                if (c.type == ContactType::SAM || c.isRadarEmitting) {
                    const RelativeGeometry rg = computeRelativeGeometry(*selfEntity,
                        DigiEntity{c.x, c.y, c.z, c.vx, c.vy, c.vz, c.yaw, c.pitch, c.roll, c.speed});
                    if (rg.range < closestSamDist) {
                        closestSamDist = rg.range;
                        samContact = &c;
                    }
                }
            }
            if (samContact) {
                groundTargetAuto_ = toDigiEntity(*samContact);
                state_.ag.groundTarget = &*groundTargetAuto_;
                state_.ag.groundTargetId = samContact->entityId;
            } else {
                // If emitter went silent / disappeared, clear
                if (state_.ag.groundTarget == &*groundTargetAuto_) {
                    state_.ag.groundTarget = nullptr;
                    state_.ag.groundTargetId = kInvalidEntityId;
                    groundTargetAuto_.reset();
                }
            }
        }
    }

    // --- Wingman formation following (Wingy mode) ---
    // If this aircraft is a wingman with an assigned lead and an injected
    // lead entity, queue Wingy mode. Wingy (20) is lower priority than
    // Waypoint (12) in the enum, so we queue it BEFORE Waypoint — that way
    // Waypoint won't replace it (12 < 20 would replace, but we skip the
    // Waypoint addMode entirely when Wingy is queued).
    //
    // FreeFalcon winglogic.cpp:919: AiCheckFormation queues WingyMode when
    // mpActionFlags[AI_FOLLOW_FORMATION] is set and no engage/RTB/landing
    // mode is active.
    const bool wingyActive =
        state_.formation.isWing &&
        state_.formation.flightLeadId != kInvalidEntityId &&
        frameInputs_.injectedLead != nullptr;
    if (wingyActive) {
        addMode(DigiMode::Wingy);
    } else if (frameInputs_.injectedTanker) {
        // --- Refueling (AAR) ---
        // If a tanker is injected, enter Refueling mode to fly to the
        // tanker's boom position and hold for fuel transfer. Refueling (13)
        // is higher priority than GroundMnvr (22) and Waypoint (12) —
        // refueling takes precedence over navigation.
        addMode(DigiMode::Refueling);
    } else if (state_.ag.groundTarget) {
        // --- GroundMnvr (A/G attack) ---
        // If a ground target is injected (and we're not a wingman in
        // formation), enter GroundMnvr mode to execute the dive-bomb
        // attack profile. GroundMnvr (22) is lower priority than Wingy
        // (20) — wingmen in formation don't break formation to attack
        // ground targets unless explicitly ordered.
        addMode(DigiMode::GroundMnvr);
    } else {
        // --- Waypoint (lowest-priority fallback) ---
        // FF RunDecisionRoutines ends by queuing WaypointMode as the default.
        // addMode() priority resolution ensures any higher-priority mode queued
        // above wins; if nothing matched, Waypoint wins.
        //
        // NOTE: FF enters LoiterMode when curWaypoint == NULL (waypoint.cpp:63),
        // but we DON'T auto-enter Loiter here. Many scenarios and tests use the
        // brain in "heading hold" mode without waypoints (setHeading +
        // setAltitude but no setWaypoints). Auto-entering Loiter would make
        // those aircraft orbit instead of holding heading. Loiter is available
        // explicitly via SteeringController::Mode::Loiter or forceMode(Loiter).
        addMode(DigiMode::Waypoint);
    }

    // --- Resolve queued mode → curMode_ for this frame ---
    resolveModeConflicts();
}

// ===========================================================================
// addMode — queue a mode for next frame, respecting priority + interlocks.
// Port of FreeFalcon dlogic.cpp:729-762 (DigitalBrain::AddMode).
//
// Rules (in FreeFalcon order):
//   1. BugoutMode is sticky — can't be bumped except by MissileDefeat
//      (TJL 11/08/03: keep BugoutMode set, allow MissileDefeat override)
//   2. LandingMode can't be bumped by WVR-family engagements once set
//      (2000-11-17 S.G.: keep Landing sticky vs defensive + engage modes)
//   3. Don't alternate between Landing and WVR each frame
//      (ME123: prevent ResolveModeConflicts from flooding ATC status flips)
//   4. Standard priority: only accept if newMode < nextMode (smaller = higher)
// ===========================================================================
void DigiBrain::addMode(DigiMode newMode) {
    // BugoutMode is sticky — can't be bumped except by MissileDefeat
    if (nextMode_ == DigiMode::Bugout && newMode != DigiMode::MissileDefeat) {
        return;
    }

    // LandingMode can't be bumped by WVR-family engagements once set.
    // (WVR-family = MissileEngage..WVREngage, the offensive engagement modes.)
    if (nextMode_ == DigiMode::Landing &&
        newMode >= DigiMode::MissileEngage && newMode <= DigiMode::WVREngage) {
        return;
    }

    // Don't alternate between Landing and WVR each frame.
    // (Without this, ResolveModeConflicts would send noATC when entering
    //  WvrEngage, alternating between Landing and WvrEngage each frame.)
    if (curMode_ == DigiMode::Landing && newMode == DigiMode::WVREngage) {
        return;
    }

    // BUG FIX: Waypoint AND Loiter are lowest-priority fallbacks — they must
    // NEVER pre-empt any other queued mode. The Round-2 structural additions
    // (Refueling=13 ... GroundMnvr=22) were placed AFTER Waypoint=12 in the
    // enum, and Loiter=17 falls between them. A naive `newMode < nextMode_`
    // check would let Waypoint (12) or Loiter (17) incorrectly pre-empt
    // RTB (19), Wingy (20), etc. — modes that should always win over both.
    // Without this special-case, fuel-critical RTB is silently overridden
    // by Loiter every frame (Loiter=17 < RTB=19), leaving the AI to orbit
    // instead of returning to base.
    //
    // FreeFalcon's AddMode avoids this by giving Waypoint the highest
    // numerical value (lowest priority) in its mode enum. We preserve the
    // existing enum ordering for backward compatibility with tests that
    // assert `WVREngage < Waypoint`, and instead special-case both fallback
    // modes here.
    if ((newMode == DigiMode::Waypoint || newMode == DigiMode::Loiter) &&
        nextMode_ != DigiMode::NoMode) {
        return;
    }

    // Standard priority: keep the smaller (higher-priority) mode.
    // NoMode is 99 (largest), so the first addMode() always accepts.
    if (newMode < nextMode_) {
        nextMode_ = newMode;
    }
}

// ===========================================================================
// resolveModeConflicts — copy nextMode_ → curMode_, snapshot lastMode_.
// Port of FreeFalcon dlogic.cpp:764-790 (DigitalBrain::ResolveModeConflicts).
//
// After this call:
//   - curMode_  = the mode resolveMode() queued for this frame
//   - lastMode_ = the previous frame's curMode_ (for newTurn detection)
//   - nextMode_ = NoMode (ready for next frame's queueing pass)
// ===========================================================================
void DigiBrain::resolveModeConflicts() {
    lastMode_ = curMode_;
    curMode_ = nextMode_;
    nextMode_ = DigiMode::NoMode;

    // --- Radio calls on mode transitions ---
    //
    // When the active mode changes, generate a radio call appropriate to
    // the new mode. Port of FreeFalcon's AiMakeRadioResponse calls scattered
    // across dlogic.cpp, bvrengage.cpp, separate.cpp, etc.
    //
    // The calls are throttled (one per 5 seconds) and "once-only" (each call
    // type is made at most once per scenario — reset clears the callsMade
    // bitmask). This prevents radio spam during rapid mode oscillation.
    if (curMode_ != lastMode_) {
        const EntityId self = state_.comm.selfId;
        auto tryCall = [&](RadioCallType type) {
            const uint32_t bit = 1u << static_cast<uint32_t>(type);
            if (!(state_.comm.callsMade & bit)) {
                if (makeRadioCall(state_.comm.radioCalls, type, simTime_, self)) {
                    state_.comm.callsMade |= bit;
                }
            }
        };

        switch (curMode_) {
            case DigiMode::BVREngage:
            case DigiMode::WVREngage:
            case DigiMode::GunsEngage:
            case DigiMode::MissileEngage:
                tryCall(RadioCallType::Engage);
                break;
            case DigiMode::Wingy:
                tryCall(RadioCallType::Rejoin);
                break;
            case DigiMode::RTB:
                tryCall(RadioCallType::RTB);
                break;
            case DigiMode::MissileDefeat:
                tryCall(RadioCallType::Missile);
                break;
            case DigiMode::Refueling:
                // AAR contact/disconnect calls are handled by runRefueling
                break;
            default:
                break;
        }
    }
}

// ===========================================================================
// Per-mode runners
// ===========================================================================
void DigiBrain::runWaypoint(const AircraftState& as, double dt,
                             const FlightControlSystem& fcs, FcsState& fcsState) {
    if (curWp_ >= wps_.size()) {
        ManeuverPrimitives::HeadingAndAltitudeHold(state_.nav.holdPsi, state_.nav.holdAlt,
                                                    state_, as, fcs, fcsState, state_.config.maxGs);
        // cornerSpeed is CAS-kts. Use the typed machHoldCas API.
        ManeuverPrimitives::machHoldCas(cas_kts(state_.config.cornerSpeed), true,
                                         state_, as, 200.0, 800.0, dt, 700.0);
        return;
    }

    const Vec3& wp = wps_[curWp_];
    const double dx = wp.x - as.kin.x;
    const double dy = wp.y - as.kin.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < captureRadius_) {
        ++curWp_;
        if (curWp_ >= wps_.size()) {
            ManeuverPrimitives::HeadingAndAltitudeHold(state_.nav.holdPsi, state_.nav.holdAlt,
                                                        state_, as, fcs, fcsState, state_.config.maxGs);
            // cornerSpeed is CAS-kts. Use the typed machHoldCas API.
            ManeuverPrimitives::machHoldCas(cas_kts(state_.config.cornerSpeed), true,
                                             state_, as, 200.0, 800.0, dt, 700.0);
            return;
        }
    }

    const double desHeading = std::atan2(dy, dx);
    const double desAlt = -wp.z;
    ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
                                                state_, as, fcs, fcsState, state_.config.maxGs);
    // cornerSpeed is CAS-kts. Use the typed machHoldCas API.
    ManeuverPrimitives::machHoldCas(cas_kts(state_.config.cornerSpeed), true,
                                     state_, as, 200.0, 800.0, dt, 700.0);
}

void DigiBrain::runMissileDefeat(const AircraftState& as, double dt,
                                  const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity || !state_.missileDefeat.incomingMissile) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    MissileDefeat(state_, *selfEntity, as, fcs, fcsState, dt);
}

void DigiBrain::runGunsJink(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity || !state_.gunsJink.gunsThreat) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    GunsJink(state_, *selfEntity, as, fcs, fcsState, dt);
}

void DigiBrain::runCollisionAvoid(const AircraftState& as, double dt,
                                   const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    CollisionAvoid(state_, *selfEntity, as, fcs, fcsState);
}

void DigiBrain::runWVREngage(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    const DigiEntity* tgt = wvrTarget_;
    if (!selfEntity || !tgt || tgt->isDead) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    RollAndPull(state_, *selfEntity, *tgt, as, fcs, fcsState, dt);
}

void DigiBrain::runGunsEngage(const AircraftState& as, double dt,
                               const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    const DigiEntity* tgt = wvrTarget_;
    if (!selfEntity || !tgt || tgt->isDead) {
        // No target — fall back to waypoint
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    // Resolve the gun spec. If the host provided an SMS, use it;
    // otherwise default to the standard M61 gun.
    WeaponSpec gun = gunSpec();  // default: M61, 510 rounds

    GunsEngage(state_, *selfEntity, *tgt, as, gun, fcs, fcsState, dt);
}

void DigiBrain::runMissileEngage(const AircraftState& as, double dt,
                                  const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    const DigiEntity* tgt = wvrTarget_;
    if (!selfEntity || !tgt || tgt->isDead) {
        // No target — fall back to waypoint
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }

    // If no SMS provided, fall back to WVR BFM (can't fire missiles
    // without knowing what's loaded)
    if (!sms_) {
        RollAndPull(state_, *selfEntity, *tgt, as, fcs, fcsState, dt);
        return;
    }

    MissileEngage(state_, *selfEntity, *tgt, as, *sms_, fcs, fcsState, dt);
}

void DigiBrain::runBVREngage(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    const DigiEntity* tgt = wvrTarget_;
    if (!selfEntity || !tgt || tgt->isDead) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    // Resolve max A/A weapon range
    double maxAAWpnRangeFt = state_.weapon.maxAAWpnRange;
    if (maxAAWpnRangeFt <= 0.0) maxAAWpnRangeFt = 35.0 * 6076.0;

    BvrEngage(state_, *selfEntity, *tgt, as, fcs, fcsState, dt);
}

void DigiBrain::runMerge(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    const DigiEntity* tgt = wvrTarget_;
    if (!selfEntity || !tgt || tgt->isDead) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    MergeManeuver(state_, *selfEntity, *tgt, as, fcs, fcsState, dt);
}

void DigiBrain::runAccel(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    AccelManeuver(state_, *selfEntity, as, fcs, fcsState, dt);
}

void DigiBrain::runTakeoff(const AircraftState& as, double dt,
                            FcsState& fcsState, double groundZ) {
    RunTakeoff(state_, as, fcsState, dt, simTime_, groundZ);
}

void DigiBrain::runLanding(const AircraftState& as, double dt,
                            FcsState& fcsState, double groundZ) {
    // Set the flight phase based on the ground-ops sub-phase.
    // This overrides the default Approach phase set in the mode dispatch.
    switch (state_.ag.groundOps.phase) {
        case GroundOpsPhase::Flare:
        case GroundOpsPhase::Touchdown:
            state_.nav.flightPhase = FlightPhase::Flare;
            break;
        case GroundOpsPhase::Rollout:
        case GroundOpsPhase::VacatingRunway:
        case GroundOpsPhase::TaxiToRunway:
        case GroundOpsPhase::LiningUp:
        case GroundOpsPhase::TakeoffRoll:
        case GroundOpsPhase::Rotation:
            state_.nav.flightPhase = FlightPhase::GroundOps;
            break;
        default:
            state_.nav.flightPhase = FlightPhase::Approach;
            break;
    }
    RunLanding(state_, as, fcsState, dt, simTime_, groundZ);
}

void DigiBrain::runWingy(const AircraftState& as, double dt,
                          const FlightControlSystem& fcs, FcsState& fcsState) {
    // Build the self entity for AiFollowLead.
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) {
        selfEntityAuto_ = buildSelfEntity(as);
        selfEntity = &selfEntityAuto_;
    }

    // Get the lead entity. Priority: injected lead > none.
    // (A future enhancement: look up the lead in the SensorPicture by
    // formation.flightLeadId — but for now, the host must inject the lead
    // entity directly via FrameInputs.injectedLead.)
    const DigiEntity* lead = frameInputs_.injectedLead;

    if (!selfEntity) {
        // Can't fly formation without a self entity — fall back to waypoint.
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }

    AiFollowLead(state_, *selfEntity, lead, as, fcs, fcsState, dt);
}

void DigiBrain::runRTB(const AircraftState& as, double dt,
                        const FlightControlSystem& fcs, FcsState& fcsState) {
    // If the host hasn't set a divert airbase, fall back to waypoint nav
    // (the host may have set up waypoints back to base).
    if (!state_.fuel.hasDivertAirbase) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }

    // Steer toward the divert airbase. Climb to a safe altitude if we're low
    // (RTB at 200 ft AGL is asking for a CFIT). The "safe altitude" is set
    // to the greater of 10000 ft MSL or current altitude + 1000 ft — this
    // keeps the AI out of terrain during RTB without forcing an unnecessary
    // climb if already cruising high.
    const double desAlt = std::max(10000.0, -as.kin.z + 1000.0);
    const double dx = state_.fuel.divertAirbaseX - as.kin.x;
    const double dy = state_.fuel.divertAirbaseY - as.kin.y;
    const double desHeading = std::atan2(dy, dx);

    ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
                                                state_, as, fcs, fcsState, state_.config.maxGs);
    // RTB cruise speed: corner speed (best range for most fighters).
    // cornerSpeed is CAS-kts. Use the typed machHoldCas API.
    ManeuverPrimitives::machHoldCas(cas_kts(state_.config.cornerSpeed), true,
                                     state_, as, 200.0, 800.0, dt, 700.0);

    // TODO (future): when within ~10 NM of the airbase, transition to
    // Landing mode and request landing clearance via the MessageBus. This
    // requires the AirbaseCheck port from FF separate.cpp / landme.cpp.
}

void DigiBrain::runRefueling(const AircraftState& as, double dt,
                               const FlightControlSystem& fcs, FcsState& fcsState) {
    std::printf("DEBUG runRefueling: 1, brain_ptr=%p\n", (void*)this);
    const DigiEntity* tanker = frameInputs_.injectedTanker;
    if (!tanker || tanker->isDead) {
        std::printf("DEBUG runRefueling: 2\n");
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    std::printf("DEBUG runRefueling: 3\n");

    // kBoomOffsetBack and kBoomOffsetDown: boom's root offset from tanker center
    constexpr double kBoomOffsetBack = 50.0;
    constexpr double kBoomOffsetDown = 20.0;
    const double tankerSigma = tanker->yaw;

    // Boom position: behind and below the tanker center
    state_.refuel.boomX = tanker->x - kBoomOffsetBack * std::cos(tankerSigma);
    state_.refuel.boomY = tanker->y - kBoomOffsetBack * std::sin(tankerSigma);
    state_.refuel.boomZ = tanker->z + kBoomOffsetDown;
    std::printf("DEBUG runRefueling: 4\n");

    // Line 30 degrees below/behind the boom:
    // Direction vector pointing downwards and backwards from the boom
    const double cos30 = std::cos(30.0 * DTR);
    const double sin30 = std::sin(30.0 * DTR);
    const double v_line_x = -cos30 * std::cos(tankerSigma);
    const double v_line_y = -cos30 * std::sin(tankerSigma);
    const double v_line_z = sin30; // NED Z-down is positive below

    // Vector from boom to receiver
    const double rx = as.kin.x - state_.refuel.boomX;
    const double ry = as.kin.y - state_.refuel.boomY;
    const double rz = as.kin.z - state_.refuel.boomZ;
    const double dist_from_boom = std::sqrt(rx * rx + ry * ry + rz * rz);
    std::printf("DEBUG runRefueling: 5\n");

    // Compute deviation angle from the 30-degree line
    double theta_deg = 0.0;
    if (dist_from_boom > 1e-3) {
        const double dot = rx * v_line_x + ry * v_line_y + rz * v_line_z;
        const double cos_theta = std::max(-1.0, std::min(1.0, dot / dist_from_boom));
        theta_deg = std::acos(cos_theta) * RTD;
    }
    std::printf("DEBUG runRefueling: 6\n");

    const double tanker_speed_kts = tanker->speed / KNOTS_TO_FTPSEC;

    // State machine transitions:
    if (state_.refuel.phase == DigiRefuelState::Phase::None) {
        state_.refuel.phase = DigiRefuelState::Phase::Inbound;
    }
    std::printf("DEBUG runRefueling: 7, phase=%d\n", static_cast<int>(state_.refuel.phase));

    switch (state_.refuel.phase) {
        case DigiRefuelState::Phase::None:
            break;

        case DigiRefuelState::Phase::Inbound: {
            std::printf("DEBUG runRefueling: Inbound 1, z=%.3f, zdot=%.3f\n", as.kin.z, as.kin.zdot);
            // Step 1: Descend to 1000 ft below tanker, close to 1 NM
            // If already within 1 NM and at least 100 ft below, clear to Precontact immediately
            const double targetAlt = -tanker->z - 1000.0;
            const double dx_t = tanker->x - as.kin.x;
            const double dy_t = tanker->y - as.kin.y;
            const double dist_to_tanker = std::sqrt(dx_t * dx_t + dy_t * dy_t);
            const double desHeading = std::atan2(dy_t, dx_t);

            std::printf("DEBUG runRefueling: Inbound 2\n");
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, targetAlt,
                state_, as, fcs, fcsState, state_.config.maxGs);

            std::printf("DEBUG runRefueling: Inbound 3\n");
            // Closure speed: fly slightly faster to close the gap
            const double closureKts = std::max(0.0, std::min(50.0, (dist_to_tanker - 6000.0) * 0.01));
            const CasKnots desCas = casFromTas(tas_kts(tanker_speed_kts + closureKts), as);
            ManeuverPrimitives::machHoldCas(desCas, false, state_, as, 150.0, 800.0, dt, 100.0);

            std::printf("DEBUG runRefueling: Inbound 4\n");
            // If established below (at least 10 ft) and within 1 NM, transition immediately
            if (as.kin.z > tanker->z + 10.0 && dist_to_tanker < 6076.0) {
                state_.refuel.phase = DigiRefuelState::Phase::Precontact;
                state_.refuel.contactTimer = 0.0;
                // Radio call: Cleared to Precontact (InPosition)
                makeRadioCall(state_.comm.radioCalls, RadioCallType::InPosition, simTime_, state_.comm.selfId);
            }
            break;
        }

        case DigiRefuelState::Phase::Precontact: {
            std::printf("DEBUG runRefueling: Precontact 1\n");
            // Step 2: Navigate to precontact position on the 30-degree line (100 - 150 ft, target 120 ft)
            const double d_target = 120.0;
            const double target_x = state_.refuel.boomX + d_target * v_line_x;
            const double target_z = state_.refuel.boomZ + d_target * v_line_z;

            const double x_error_p = target_x - as.kin.x;
            const double heading_corr_p = std::max(-0.26, std::min(0.26, x_error_p * 0.005));
            const double desHeading = tankerSigma + heading_corr_p;

            std::printf("DEBUG runRefueling: Precontact 2\n");
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, -target_z,
                state_, as, fcs, fcsState, state_.config.maxGs);

            std::printf("DEBUG runRefueling: Precontact 3\n");
            // Closure speed to precontact target
            const double closureKts = std::max(-20.0, std::min(40.0, (dist_from_boom - 120.0) * 0.15));
            const CasKnots desCas = casFromTas(tas_kts(tanker_speed_kts + closureKts), as);
            ManeuverPrimitives::machHoldCas(desCas, false, state_, as, 150.0, 800.0, dt, 100.0);

            std::printf("DEBUG runRefueling: Precontact 4\n");
            // Stabilize check: if within 15 ft of 120 ft and aligned within 5 degrees
            if (std::abs(dist_from_boom - 120.0) < 15.0 && theta_deg < 5.0) {
                state_.refuel.contactTimer += dt;
                if (state_.refuel.contactTimer >= 3.0) {
                    state_.refuel.phase = DigiRefuelState::Phase::Contact;
                    state_.refuel.contactTimer = 0.0;
                    // Radio call: Cleared to Contact
                    makeRadioCall(state_.comm.radioCalls, RadioCallType::Contact, simTime_, state_.comm.selfId);
                }
            } else {
                state_.refuel.contactTimer = 0.0; // reset if drifted
            }
            break;
        }

        case DigiRefuelState::Phase::Contact: {
            std::printf("DEBUG runRefueling: Contact 1\n");
            // Step 3: Navigate up the 30-degree line to contact position (20 ft)
            // Deviating more than 10 degrees or closing within 10 ft causes backing out to Precontact
            if (theta_deg > 10.0 || dist_from_boom < 10.0) {
                state_.refuel.phase = DigiRefuelState::Phase::Precontact;
                state_.refuel.contactTimer = 0.0;
                break;
            }

            const double d_target = 20.0;
            const double target_x = state_.refuel.boomX + d_target * v_line_x;
            const double target_z = state_.refuel.boomZ + d_target * v_line_z;

            const double x_error_c = target_x - as.kin.x;
            const double heading_corr_c = std::max(-0.26, std::min(0.26, x_error_c * 0.005));
            const double desHeading = tankerSigma + heading_corr_c;

            std::printf("DEBUG runRefueling: Contact 2\n");
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, -target_z,
                state_, as, fcs, fcsState, state_.config.maxGs);

            std::printf("DEBUG runRefueling: Contact 3\n");
            // Maintain exact tanker speed + gentle closure (max 5 kts to prevent overshoot/oscillation)
            const double closureKts = std::max(-10.0, std::min(5.0, (dist_from_boom - 20.0) * 0.1));
            const CasKnots desCas = casFromTas(tas_kts(tanker_speed_kts + closureKts), as);
            ManeuverPrimitives::machHoldCas(desCas, false, state_, as, 150.0, 800.0, dt, 100.0);

            std::printf("DEBUG runRefueling: Contact 4\n");
            // Stabilize and transfer fuel
            if (std::abs(dist_from_boom - 20.0) < 5.0 && theta_deg < 5.0) {
                state_.refuel.contactTimer += dt;
                if (state_.refuel.contactTimer >= state_.refuel.contactDuration) {
                    state_.refuel.phase = DigiRefuelState::Phase::Disconnect;
                    // Radio call: Disconnect
                    makeRadioCall(state_.comm.radioCalls, RadioCallType::Disconnect, simTime_, state_.comm.selfId);
                }
            }
            break;
        }

        case DigiRefuelState::Phase::Disconnect: {
            std::printf("DEBUG runRefueling: Disconnect 1\n");
            // Step 4: Back down the 30-degree line until 1000 ft below tanker
            const double targetAlt = -tanker->z - 1000.0;
            const double awayHeading = tankerSigma + M_PI;

            std::printf("DEBUG runRefueling: Disconnect 2\n");
            ManeuverPrimitives::HeadingAndAltitudeHold(awayHeading, targetAlt,
                state_, as, fcs, fcsState, state_.config.maxGs);

            std::printf("DEBUG runRefueling: Disconnect 3\n");
            const double approachSpeed = 250.0;
            ManeuverPrimitives::machHoldCas(cas_kts(approachSpeed), true,
                state_, as, 150.0, 800.0, dt, 100.0);

            std::printf("DEBUG runRefueling: Disconnect 4\n");
            if (std::abs(-as.kin.z - targetAlt) < 150.0) {
                // Done! Set refuelComplete flag and fall back to waypoint nav
                state_.refuel.refuelComplete = true;
                state_.refuel.phase = DigiRefuelState::Phase::None;
                curWp_++;
                runWaypoint(as, dt, fcs, fcsState);
            }
            break;
        }
    }
    std::printf("DEBUG runRefueling: end\n");
}

void DigiBrain::runGroundAttack(const AircraftState& as, double dt,
                                 const FlightControlSystem& fcs, FcsState& fcsState) {
    // Ground attack dispatcher.
    //
    // Port of FreeFalcon's GroundAttackMode (gndattck.cpp:96-4900), but
    // vastly simplified. FreeFalcon's version is 4900 lines covering bombs,
    // GBUs, AG missiles, rockets, strafing, CCIP/CCRP/DFT modes, target
    // selection, weapon selection, and traffic-pattern attack profiles.
    //
    // Task 11 added a single dive-bomb profile; Task 15-a expands this to
    // three profiles selected by state_.ag.agProfile (host injects via
    // FrameInputs.injectedAgProfile). Each profile is a 4-phase state
    // machine using state_.ag.agApproach as the phase counter (0..3).
    //
    //   DiveBomb       — approach → dive → pullout → egress (Task 11 default)
    //   LevelDelivery  — approach → level → release → egress (Task 15-a)
    //   TossBomb       — approach → pull-up → release → egress (Task 15-a)
    //
    // The host injects the ground target via FrameInputs.injectedGroundTarget.
    // The brain stores it in state_.ag.groundTarget.

    const DigiEntity* target = state_.ag.groundTarget;
    if (!target || target->isDead) {
        // No target — fall back to waypoint navigation.
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }

    switch (state_.ag.agProfile) {
        case AgAttackProfile::DiveBomb:
            runDiveBombAttack(target, as, dt, fcs, fcsState);
            break;
        case AgAttackProfile::LevelDelivery:
            runLevelDeliveryAttack(target, as, dt, fcs, fcsState);
            break;
        case AgAttackProfile::TossBomb:
            runTossBombAttack(target, as, dt, fcs, fcsState);
            break;
        case AgAttackProfile::SeadHarm:
            runSeadHarmAttack(target, as, dt, fcs, fcsState);
            break;
    }
}

// ===========================================================================
// runDiveBombAttack — the original Task 11 dive-bomb profile.
// 4-phase state machine (state_.ag.agApproach):
//   0 = approach, 1 = dive, 2 = pullout, 3 = egress.
// ===========================================================================
void DigiBrain::runDiveBombAttack(const DigiEntity* target,
                                    const AircraftState& as, double dt,
                                    const FlightControlSystem& fcs, FcsState& fcsState) {
    // Compute geometry relative to the target.
    const double dx = target->x - as.kin.x;
    const double dy = target->y - as.kin.y;
    const double horizDist = std::sqrt(dx * dx + dy * dy);
    const double altAGL = -as.kin.z;  // assuming groundZ = 0 for simplicity
    const double desHeading = std::atan2(dy, dx);

    // Attack parameters (dive-bomb profile).
    constexpr double kDiveStartRangeFt  = 18000.0;  // 3 NM — start dive
    constexpr double kDiveAngleDeg      = 30.0;     // 30° dive
    constexpr double kReleaseAltFt      = 4000.0;   // release at 4000 ft AGL
    constexpr double kPulloutAltFt      = 2000.0;   // pull out by 2000 ft AGL
    constexpr double kSafeAltFt         = 12000.0;  // egress altitude
    constexpr double kEgressRangeFt     = 30000.0;  // 5 NM — egress complete
    constexpr double kDiveSpeedKts      = 450.0;    // target dive speed
    constexpr double kApproachSpeedKts  = 350.0;    // approach speed

    // State machine for the attack profile.
    // We use agApproach as a simple phase counter (0=approach, 1=dive,
    // 2=pullout, 3=egress) to avoid depending on the complex AgAttackPhase enum.
    int& phase = state_.ag.agApproach;  // reuse this field as our phase counter

    switch (phase) {
        case 0: {
            // --- Approach phase ---
            // Fly toward the target at cruise altitude and approach speed.
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kSafeAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            // kApproachSpeedKts is a CAS-kts constant. Use the typed machHoldCas API.
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Transition to dive when within range.
            if (horizDist < kDiveStartRangeFt) {
                phase = 1;
            }
            break;
        }

        case 1: {
            // --- Dive phase ---
            // Dive toward the target at the dive angle. Steer toward the
            // target and command a -30° flight path angle.
            const double desGamma = -kDiveAngleDeg;
            // Steer toward target heading.
            const double headingErr = headingError(desHeading, as.kin.sigma);
            state_.commands.rStick = std::max(-1.0, std::min(1.0,
                headingErr * RTD * 2.0 * DTR));
            // Command dive gamma via GammaHold.
            state_.nav.gammaHoldIError = 0.0;
            ManeuverPrimitives::GammaHold(desGamma, state_, as, state_.config.maxGs);
            ManeuverPrimitives::PhugoidDamper(state_, as);
            // Speed: target dive speed (let speed build in the dive).
            // kDiveSpeedKts is a CAS-kts constant. Use the typed machHoldCas API.
            ManeuverPrimitives::machHoldCas(cas_kts(kDiveSpeedKts), false,
                state_, as, 200.0, 800.0, dt, 700.0);
            // Deploy speed brakes if too fast in the dive.
            if (as.vcas > kDiveSpeedKts + 30.0) {
                state_.commands.speedBrakeCmd = 1.0;
            } else {
                state_.commands.speedBrakeCmd = -1.0;
            }

            // Release weapon at the correct altitude.
            // In a 30° dive at 450 kts, the horizontal range at 4000 ft AGL
            // is ~4000/tan(30°) = ~6928 ft. We release when altitude drops
            // to the release altitude.
            if (altAGL <= kReleaseAltFt) {
                state_.weapon.mslFireFlag = true;  // trigger weapon release
                phase = 2;  // transition to pullout
            }
            // Safety: if we somehow get too low without releasing, pull out.
            if (altAGL <= kPulloutAltFt) {
                phase = 2;
            }
            break;
        }

        case 2: {
            // --- Pullout phase ---
            // Arrest the descent and climb back to safe altitude.
            // Command a strong pitch-up (4G) to pull out of the dive.
            state_.weapon.mslFireFlag = false;  // clear release
            state_.commands.speedBrakeCmd = -1.0;  // clean for pullout

            // Use GammaHold to command a climb back to safe altitude.
            const double altErr = (kSafeAltFt + as.kin.z) - as.kin.zdot;
            state_.nav.gammaHoldIError = 0.0;
            ManeuverPrimitives::GammaHold(altErr * 0.02, state_, as, state_.config.maxGs);
            ManeuverPrimitives::PhugoidDamper(state_, as);

            // Continue toward the target heading (fly over the target).
            const double headingErr2 = headingError(desHeading, as.kin.sigma);
            state_.commands.rStick = std::max(-1.0, std::min(1.0,
                headingErr2 * RTD * 2.0 * DTR));

            // Full throttle for the pullout (need energy).
            state_.commands.throttle = 1.5;

            // Transition to egress when back at safe altitude.
            if (altAGL > kSafeAltFt - 500.0) {
                phase = 3;
            }
            break;
        }

        case 3: {
            // --- Egress phase ---
            // Fly away from the target at safe altitude.
            // Reverse heading (fly away from the target).
            const double egressHeading = desHeading + M_PI;  // opposite direction
            ManeuverPrimitives::HeadingAndAltitudeHold(egressHeading, kSafeAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            // kApproachSpeedKts is a CAS-kts constant. Use the typed machHoldCas API.
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Egress complete when far enough from the target.
            if (horizDist > kEgressRangeFt) {
                // Clear the ground target — attack complete.
                state_.ag.groundTarget = nullptr;
                state_.ag.groundTargetId = kInvalidEntityId;
                phase = 0;  // reset for next attack
                // Fall back to waypoint navigation.
                runWaypoint(as, dt, fcs, fcsState);
            }
            break;
        }

        default:
            phase = 0;
            break;
    }
}

// ===========================================================================
// runSeadHarmAttack — phase 2 HARM / SEAD profile.
// 4-phase state machine (state_.ag.agApproach):
//   0 = approach/search, 1 = lock/loft, 2 = release, 3 = egress/beam.
// ===========================================================================
void DigiBrain::runSeadHarmAttack(const DigiEntity* target,
                                  const AircraftState& as, double dt,
                                  const FlightControlSystem& fcs, FcsState& fcsState) {
    // Suppression of Enemy Air Defenses (SEAD) using AGM-88 HARM.
    // Supports Pre-Briefed (PB), Target-Of-Opportunity (TOO), and Self-Protect (SP) modes.

    // Geometry relative to target
    const double dx = target->x - as.kin.x;
    const double dy = target->y - as.kin.y;
    const double horizDist = std::sqrt(dx * dx + dy * dy);
    const double desHeading = std::atan2(dy, dx);

    // SEAD / HARM parameters
    constexpr double kApproachAltFt      = 20000.0;  // 20k ft approach
    constexpr double kApproachSpeedKts   = 400.0;    // 400 kts cruise
    constexpr double kLoftRangeFt        = 25.0 * 6076.0; // 25 NM PB loft range
    constexpr double kTooRangeFt         = 35.0 * 6076.0; // 35 NM TOO lock range
    constexpr double kLoftGammaDeg       = 20.0;     // command 20° loft climb
    constexpr double kLoftGs             = 3.0;      // 3G pull-up for loft
    constexpr double kEgressAltFt        = 25000.0;  // egress at 25k ft
    constexpr double kEgressRangeFt      = 35.0 * 6076.0; // 35 NM egress complete
    constexpr double kEgressSpeedKts     = 420.0;    // fast egress speed

    int& phase = state_.ag.agApproach;

    switch (phase) {
        case 0: {
            // --- Phase 0: Approach/Search ---
            // Fly toward the threat area at approach altitude and speed.
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kApproachAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Check transition to Phase 1 (Lock/Loft) based on HTS mode:
            if (state_.ag.htsMode == HtsMode::SelfProtect) {
                // SP Mode: lock on and launch immediately (emergency)
                phase = 1;
            } else if (state_.ag.htsMode == HtsMode::TargetOfOpportunity) {
                // TOO Mode: lock on when within range
                if (horizDist < kTooRangeFt) {
                    phase = 1;
                }
            } else {
                // PB Mode: loft when within range
                if (horizDist < kLoftRangeFt) {
                    phase = 1;
                }
            }
            break;
        }

        case 1: {
            // --- Phase 1: Lock/Loft ---
            // Direct flight or loft toward the target heading.
            const double headingErr = headingError(desHeading, as.kin.sigma);
            state_.commands.rStick = std::max(-1.0, std::min(1.0, headingErr * RTD * 2.0 * DTR));

            if (state_.ag.htsMode == HtsMode::SelfProtect) {
                // SP Mode: no loft, launch level immediately
                ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, as.kin.z * -1.0,
                    state_, as, fcs, fcsState, state_.config.maxGs);
                phase = 2; // immediate release
            } else {
                // TOO and PB Modes: loft climb (+20° gamma)
                ManeuverPrimitives::GammaHold(kLoftGammaDeg, state_, as, kLoftGs);
                ManeuverPrimitives::PhugoidDamper(state_, as);
                state_.commands.throttle = 1.5; // full military/afterburner power for loft

                // Transition to release once loft angle is established
                const double pitchDeg = as.kin.theta * RTD;
                if (pitchDeg >= 15.0) {
                    phase = 2;
                }
            }
            break;
        }

        case 2: {
            // --- Phase 2: Release ---
            // Trigger HARM release (mslFireFlag = true)
            state_.weapon.mslFireFlag = true;

            // Maintain attitude during release frame
            if (state_.ag.htsMode == HtsMode::SelfProtect) {
                ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, as.kin.z * -1.0,
                    state_, as, fcs, fcsState, state_.config.maxGs);
            } else {
                ManeuverPrimitives::GammaHold(kLoftGammaDeg, state_, as, kLoftGs);
                ManeuverPrimitives::PhugoidDamper(state_, as);
                state_.commands.throttle = 1.5;
            }

            // Move to egress on next frame
            phase = 3;
            break;
        }

        case 3: {
            // --- Phase 3: Defensive Beam / Egress ---
            state_.weapon.mslFireFlag = false; // clear launch consent

            double targetHeading = desHeading + PI; // PB defaults to 180° turn away
            if (state_.ag.htsMode == HtsMode::SelfProtect || state_.ag.htsMode == HtsMode::TargetOfOpportunity) {
                // Perform a 90° beam turn relative to threat to break track/reduce range rate
                double beamHeading = desHeading + PI / 2.0;
                if (std::fabs(headingError(beamHeading, as.kin.sigma)) > std::fabs(headingError(desHeading - PI / 2.0, as.kin.sigma))) {
                    beamHeading = desHeading - PI / 2.0;
                }
                targetHeading = beamHeading;
            }

            // Head back to safe altitude and egress speed
            ManeuverPrimitives::HeadingAndAltitudeHold(targetHeading, kEgressAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kEgressSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Egress complete once range is far enough
            if (horizDist > kEgressRangeFt) {
                state_.ag.groundTarget = nullptr;
                state_.ag.groundTargetId = kInvalidEntityId;
                phase = 0;
                runWaypoint(as, dt, fcs, fcsState);
            }
            break;
        }

        default:
            phase = 0;
            break;
    }
}

// ===========================================================================
// runLevelDeliveryAttack — Task 15-a level bombing profile.
// 4-phase state machine (state_.ag.agApproach):
//   0 = approach, 1 = level (descend to low altitude), 2 = release, 3 = egress.
// ===========================================================================
void DigiBrain::runLevelDeliveryAttack(const DigiEntity* target,
                                         const AircraftState& as, double dt,
                                         const FlightControlSystem& fcs, FcsState& fcsState) {
    // Geometry relative to the target.
    const double dx = target->x - as.kin.x;
    const double dy = target->y - as.kin.y;
    const double horizDist = std::sqrt(dx * dx + dy * dy);
    const double altAGL = -as.kin.z;  // groundZ = 0
    const double desHeading = std::atan2(dy, dx);
    // Attack parameters (level-delivery profile).
    //
    // NOTE: the task spec calls for a 500 ft AGL level run. However, the
    // brain's GroundAvoid overlay (ground_avoid.cpp) triggers a pull-up
    // whenever the 5-second predicted altitude drops below 500 ft — which
    // makes a sustained 500 ft AGL level run impossible (any Phugoid dip
    // triggers an immediate 9G pull-up that pre-empts the A/G state machine).
    // We use 1500 ft AGL for the level run instead — still low enough to be a
    // tactical level delivery, but above the ground-avoid trigger. The
    // descent uses a limited -12° gamma (instead of HeadingAndAltitudeHold,
    // which commands up to -60° for large alt errors) to keep the descent
    // rate manageable and avoid triggering ground avoid during the dive.
    constexpr double kApproachAltFt      = 8000.0;   // cruise-in altitude
    constexpr double kLevelAltFt         = 1500.0;   // level run altitude (AGL)
    constexpr double kDescentGammaDeg    = -12.0;    // shallow descent gamma
    constexpr double kLevelStartRangeFt  = 18228.0;  // 3 NM — start descent
    constexpr double kReleaseRangeFt     = 800.0;    // release when over tgt
    constexpr double kApproachSpeedKts   = 400.0;    // approach speed
    constexpr double kLevelSpeedKts      = 400.0;    // level run speed
    constexpr double kEgressRangeFt      = 18228.0;  // 3 NM — egress complete

    int& phase = state_.ag.agApproach;

    switch (phase) {
        case 0: {
            // --- Approach phase ---
            // Fly toward the target at cruise altitude and approach speed.
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kApproachAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Transition to level descent when within 2 NM.
            if (horizDist < kLevelStartRangeFt) {
                phase = 1;
            }
            break;
        }

        case 1: {
            // --- Level descent + run phase ---
            // Descend to the level-run altitude using a LIMITED shallow gamma
            // (not HeadingAndAltitudeHold, which would command up to -60° for
            // the 6500 ft alt error and trigger GroundAvoid). Once at the
            // level altitude, switch to HeadingAndAltitudeHold to maintain it.
            //
            // Steer toward the target heading throughout.
            const double headingErr = headingError(desHeading, as.kin.sigma);
            state_.commands.rStick = std::max(-1.0, std::min(1.0,
                headingErr * RTD * 2.0 * DTR));

            if (altAGL > kLevelAltFt + 200.0) {
                // Still descending — use limited gamma toward the target alt.
                state_.nav.gammaHoldIError = 0.0;
                ManeuverPrimitives::GammaHold(kDescentGammaDeg, state_, as,
                    state_.config.maxGs);
                ManeuverPrimitives::PhugoidDamper(state_, as);
            } else {
                // At level altitude — hold it.
                ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kLevelAltFt,
                    state_, as, fcs, fcsState, state_.config.maxGs);
                ManeuverPrimitives::PhugoidDamper(state_, as);
            }
            ManeuverPrimitives::machHoldCas(cas_kts(kLevelSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Release when directly over the target.
            if (horizDist < kReleaseRangeFt) {
                phase = 2;
            }
            // Safety: if we somehow descend below 500 ft AGL, bail to egress.
            if (altAGL < 500.0) {
                phase = 3;
            }
            break;
        }

        case 2: {
            // --- Release phase ---
            // Release the bomb when directly over the target. Continue
            // straight and level momentarily so the release geometry is
            // stable, then transition to egress.
            state_.weapon.mslFireFlag = true;  // trigger weapon release
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kLevelAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::PhugoidDamper(state_, as);
            ManeuverPrimitives::machHoldCas(cas_kts(kLevelSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Transition to egress immediately after release (one frame of
            // release-consent is enough — the host reads the flag this frame).
            phase = 3;
            break;
        }

        case 3: {
            // --- Egress phase ---
            // Continue straight ahead (same heading — overfly the target),
            // climb back to the approach altitude, accelerate to cruise.
            state_.weapon.mslFireFlag = false;  // clear release
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kApproachAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Egress complete when far enough past the target.
            if (horizDist > kEgressRangeFt) {
                state_.ag.groundTarget = nullptr;
                state_.ag.groundTargetId = kInvalidEntityId;
                phase = 0;
                runWaypoint(as, dt, fcs, fcsState);
            }
            break;
        }

        default:
            phase = 0;
            break;
    }
}

// ===========================================================================
// runTossBombAttack — Task 15-a toss (loft) bombing profile.
// 4-phase state machine (state_.ag.agApproach):
//   0 = approach, 1 = pull-up, 2 = release, 3 = egress.
// ===========================================================================
void DigiBrain::runTossBombAttack(const DigiEntity* target,
                                    const AircraftState& as, double dt,
                                    const FlightControlSystem& fcs, FcsState& fcsState) {
    // Geometry relative to the target.
    const double dx = target->x - as.kin.x;
    const double dy = target->y - as.kin.y;
    const double horizDist = std::sqrt(dx * dx + dy * dy);
    const double altAGL = -as.kin.z;  // groundZ = 0
    const double desHeading = std::atan2(dy, dx);

    // Attack parameters (toss-bomb profile).
    //
    // NOTE: the task spec calls for a 500 ft AGL / 450 kts approach. However,
    // the brain's GroundAvoid overlay (ground_avoid.cpp) triggers a pull-up
    // whenever the 5-second predicted altitude drops below 500 ft. At 500 ft
    // AGL, any Phugoid dip or machHold pitch-down transient triggers an
    // immediate 9G pull-up that pre-empts the A/G state machine. We use
    // 1500 ft AGL for the approach instead — still low enough to be a
    // tactical toss-bomb approach, but above the ground-avoid trigger.
    // We also use 400 kts (corner speed) instead of 450 to avoid the
    // high-speed pitch-up transient that the F-16 FCS produces at 450 kts /
    // 500 ft (the thrust line is above the CG, so high throttle at low alt
    // produces a nose-up moment that the FCS can't trim out).
    constexpr double kApproachAltFt      = 1500.0;   // 1500 ft AGL run-in
    constexpr double kApproachSpeedKts   = 400.0;    // fast run-in
    constexpr double kPullupRangeFt      = 18228.0;  // 3 NM — start pull-up
    constexpr double kPullupGammaDeg     = 30.0;     // command 30° flight path
    constexpr double kPullupGs           = 4.0;      // 4G climb
    constexpr double kReleasePitchDeg    = 45.0;     // release at 45° pitch attitude
    constexpr double kReleaseAltFt       = 3000.0;   // OR release at 3000 ft AGL
    constexpr double kEgressAltFt        = 10000.0;  // climb to 10000 ft
    constexpr double kEgressRangeFt      = 30000.0;  // 5 NM — egress complete
    constexpr double kEgressSpeedKts     = 400.0;    // egress cruise speed

    int& phase = state_.ag.agApproach;

    switch (phase) {
        case 0: {
            // --- Approach phase ---
            // Fly toward the target at 500 ft AGL / 450 kts.
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kApproachAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kApproachSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Transition to pull-up when within 3 NM.
            if (horizDist < kPullupRangeFt) {
                state_.nav.gammaHoldIError = 0.0;
                phase = 1;
            }
            break;
        }

        case 1: {
            // --- Pull-up phase ---
            // Pull up into a 4G climb (command 30° gamma). Steer toward the
            // target heading to keep the bomb's ballistic trajectory aimed
            // at the target.
            const double headingErr = headingError(desHeading, as.kin.sigma);
            state_.commands.rStick = std::max(-1.0, std::min(1.0,
                headingErr * RTD * 2.0 * DTR));
            // GammaHold with maxGs = 4.0 limits the pull to ~4G.
            ManeuverPrimitives::GammaHold(kPullupGammaDeg, state_, as, kPullupGs);
            ManeuverPrimitives::PhugoidDamper(state_, as);
            // Full throttle during the pull-up (need energy for the climb).
            state_.commands.throttle = 1.5;

            // Release at 45° pitch attitude OR 3000 ft AGL (whichever first).
            // as.kin.theta is body pitch (radians) — convert to degrees.
            const double pitchDeg = as.kin.theta * RTD;
            if (pitchDeg >= kReleasePitchDeg || altAGL >= kReleaseAltFt) {
                state_.weapon.mslFireFlag = true;  // trigger weapon release
                phase = 2;
            }
            // Safety: if we get too high without triggering, transition anyway.
            if (altAGL > kEgressAltFt) {
                state_.weapon.mslFireFlag = true;
                phase = 2;
            }
            break;
        }

        case 2: {
            // --- Release phase (transient) ---
            // One frame of release-consent, then transition to egress. The
            // host reads mslFireFlag this frame.
            state_.weapon.mslFireFlag = true;
            // Continue the climb momentarily to preserve release geometry.
            ManeuverPrimitives::GammaHold(kPullupGammaDeg, state_, as, kPullupGs);
            ManeuverPrimitives::PhugoidDamper(state_, as);
            state_.commands.throttle = 1.5;
            phase = 3;
            break;
        }

        case 3: {
            // --- Egress phase ---
            // Continue the climb to 10000 ft, then level off and fly away
            // from the target. We continue on the same heading (toward the
            // target's bearing) since the bomb's ballistic trajectory
            // carries it forward to the target — the aircraft also continues
            // forward and away from the (now overflown) target area.
            state_.weapon.mslFireFlag = false;
            ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, kEgressAltFt,
                state_, as, fcs, fcsState, state_.config.maxGs);
            ManeuverPrimitives::machHoldCas(cas_kts(kEgressSpeedKts), true,
                state_, as, 200.0, 800.0, dt, 700.0);

            // Egress complete when far enough past the target.
            if (altAGL > kEgressAltFt - 500.0 && horizDist > kEgressRangeFt) {
                state_.ag.groundTarget = nullptr;
                state_.ag.groundTargetId = kInvalidEntityId;
                phase = 0;
                runWaypoint(as, dt, fcs, fcsState);
            }
            break;
        }

        default:
            phase = 0;
            break;
    }
}

void DigiBrain::runFollowOrders(const AircraftState& as, double dt,
                                const FlightControlSystem& fcs, FcsState& fcsState) {
    // Build the self entity for AiPerformManeuver.
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) {
        selfEntityAuto_ = buildSelfEntity(as);
        selfEntity = &selfEntityAuto_;
    }
    if (!selfEntity) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }

    // Resolve the target for engage-style maneuvers (Posthole, Chainsaw).
    // Priority: injected target > SensorPicture bestTarget > wvrTarget_.
    // If WingmanState.designatedTargetId is set, a future enhancement would
    // look it up in the SensorPicture; for now we use wvrTarget_ (which the
    // brain's resolveMode already populated from injectedTarget or auto-track).
    const DigiEntity* target = wvrTarget_;
    if (!target) target = frameInputs_.injectedTarget;

    // Round 6: initialize multi-point maneuvers on their first frame.
    // AiInitPince / AiInitFlex set up the maneuverPoints array; the
    // maneuverPointCounter starts at 0. We detect "first frame" by checking
    // if maneuverPointCounter is 0 AND mnverTime is 0 (the brain's
    // resolveMode arms mnverTime for timer-based maneuvers but not for
    // multi-point maneuvers — those use maneuverPointCounter instead).
    const auto maneuver = state_.formation.wingman.currentManeuver;
    if (maneuver == WingmanManeuver::Pince && state_.formation.maneuverPointCounter == 0
        && state_.nav.mnverTime <= 0.0) {
        AiInitPince(state_, *selfEntity, target, frameInputs_.injectedLead);
        // Set mnverTime to a non-zero sentinel so we don't re-init next frame.
        // The actual completion is driven by maneuverPointCounter, not mnverTime.
        state_.nav.mnverTime = -1.0;  // sentinel: "already initialized"
    } else if (maneuver == WingmanManeuver::Flex && state_.formation.maneuverPointCounter == 0
               && state_.nav.mnverTime <= 0.0) {
        AiInitFlex(state_, *selfEntity, target, frameInputs_.injectedLead);
        state_.nav.mnverTime = -1.0;  // sentinel
    }

    // Dispatch to the active maneuver. AiPerformManeuver returns true while
    // the maneuver is still active; false when it completes (and clears it).
    const bool stillActive = AiPerformManeuver(state_, *selfEntity, target, sms_,
                                                as, fcs, fcsState, dt);
    if (!stillActive) {
        // Maneuver complete — fall back to Wingy (formation following) for
        // this frame so the wingman doesn't drift while the brain re-resolves
        // mode next frame.
        runWingy(as, dt, fcs, fcsState);
    }
}

// ===========================================================================
// Behavior Tree and Nodes Implementation
// ===========================================================================

void DigiBrain::buildBehaviorTree() {
    auto root = std::make_shared<SelectorNode>("Root");
    root->addChild(std::make_shared<ForcedModeNode>());
    root->addChild(std::make_shared<GroundAvoidNode>());
    root->addChild(std::make_shared<CollisionAvoidNode>());
    root->addChild(std::make_shared<MissileDefeatNode>());
    root->addChild(std::make_shared<GunsJinkNode>());
    root->addChild(std::make_shared<TakeoffNode>());
    root->addChild(std::make_shared<LandingNode>());
    root->addChild(std::make_shared<FollowOrdersNode>());
    root->addChild(std::make_shared<RTBNode>());
    root->addChild(std::make_shared<CombatNode>());
    root->addChild(std::make_shared<WingyNode>());
    root->addChild(std::make_shared<RefuelNode>());
    root->addChild(std::make_shared<GroundMnvrNode>());
    root->addChild(std::make_shared<WaypointFollowNode>());
    root->addChild(std::make_shared<DefaultFallbackNode>());
    rootNode_ = root;
}

void DigiBrain::runLegacyMode(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState,
                              double groundZ, const DigiEntity* selfEntity) {
    switch (curMode_) {
    case DigiMode::Waypoint:
        runWaypoint(as, dt, fcs, fcsState);
        break;
    case DigiMode::MissileDefeat:
        runMissileDefeat(as, dt, fcs, fcsState);
        break;
    case DigiMode::GunsJink:
        runGunsJink(as, dt, fcs, fcsState);
        break;
    case DigiMode::CollisionAvoid:
        runCollisionAvoid(as, dt, fcs, fcsState);
        break;
    case DigiMode::WVREngage:
        runWVREngage(as, dt, fcs, fcsState);
        break;
    case DigiMode::Takeoff:
        runTakeoff(as, dt, fcsState, groundZ);
        break;
    case DigiMode::Landing:
        runLanding(as, dt, fcsState, groundZ);
        break;
    case DigiMode::GroundAvoid:
        break;
    case DigiMode::MissileEngage:
        runMissileEngage(as, dt, fcs, fcsState);
        break;
    case DigiMode::GunsEngage:
        runGunsEngage(as, dt, fcs, fcsState);
        break;
    case DigiMode::Merge:
        runMerge(as, dt, fcs, fcsState);
        break;
    case DigiMode::Accel:
        runAccel(as, dt, fcs, fcsState);
        break;
    case DigiMode::BVREngage:
        runBVREngage(as, dt, fcs, fcsState);
        break;
    case DigiMode::NoMode:
        runWaypoint(as, dt, fcs, fcsState);
        break;
    case DigiMode::Roop: {
        if (selfEntity && wvrTarget_) {
            const bool stillActive = ManeuverPrimitives::RollOutOfPlane(
                state_, *selfEntity, as, fcs, fcsState, dt,
                /*firstFrame=*/false);
            if (!stillActive) {
                runWaypoint(as, dt, fcs, fcsState);
            }
        } else {
            runWaypoint(as, dt, fcs, fcsState);
        }
        break;
    }
    case DigiMode::OverB: {
        if (selfEntity && wvrTarget_) {
            ManeuverPrimitives::OverBank(state_, *selfEntity, *wvrTarget_,
                                          fcs, fcsState,
                                          30.0 * DTR, /*firstFrame=*/false);
        } else {
            runWaypoint(as, dt, fcs, fcsState);
        }
        break;
    }
    case DigiMode::Loiter:
        ManeuverPrimitives::Loiter(state_, as, fcs, fcsState, state_.config.maxGs);
        ManeuverPrimitives::machHoldCas(cas_kts(state_.config.cornerSpeed), true,
                                         state_, as, 200.0, 800.0, dt, 700.0);
        break;
    case DigiMode::Bugout:
    case DigiMode::Separate:
        ManeuverPrimitives::WvrBugOut(state_, as, fcs, fcsState, dt);
        break;
    case DigiMode::Refueling:
        runRefueling(as, dt, fcs, fcsState);
        break;
    case DigiMode::FollowOrders:
        runFollowOrders(as, dt, fcs, fcsState);
        break;
    case DigiMode::RTB:
        runRTB(as, dt, fcs, fcsState);
        break;
    case DigiMode::Wingy:
        runWingy(as, dt, fcs, fcsState);
        break;
    case DigiMode::GroundMnvr:
        runGroundAttack(as, dt, fcs, fcsState);
        break;
    }
}

NodeStatus ForcedModeNode::onTick(Blackboard& bb) {
    DigiMode forced = bb.brain->forcedMode();
    if (forced != DigiMode::NoMode) {
        bb.brain->setCurMode(forced);
        bb.brain->runLegacyMode(*bb.as, bb.dt, *bb.fcs, *bb.fcsState, bb.groundZ, bb.self);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus GroundAvoidNode::onTick(Blackboard& bb) {
    if (bb.state->groundAvoid.groundAvoidNeeded) {
        bb.brain->setCurMode(DigiMode::GroundAvoid);
        bb.state->nav.flightPhase = FlightPhase::Combat;
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus CollisionAvoidNode::onTick(Blackboard& bb) {
    const DigiEntity* collTarget = bb.brain->frameInputs().injectedTarget;
    if (!collTarget) collTarget = bb.brain->wvrTarget();
    if (bb.self && collTarget && !collTarget->isDead) {
        if (CollisionCheck(*bb.state, *bb.self, *collTarget)) {
            bb.brain->setCurMode(DigiMode::CollisionAvoid);
            bb.state->nav.flightPhase = FlightPhase::Combat;
            bb.brain->runCollisionAvoid(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus MissileDefeatNode::onTick(Blackboard& bb) {
    if (bb.self && bb.state->missileDefeat.incomingMissile) {
        if (MissileDefeatCheck(*bb.state, *bb.self, bb.dt)) {
            bb.brain->setCurMode(DigiMode::MissileDefeat);
            bb.state->nav.flightPhase = FlightPhase::Combat;
            bb.brain->runMissileDefeat(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus GunsJinkNode::onTick(Blackboard& bb) {
    if (bb.self && bb.state->gunsJink.gunsThreat) {
        if (GunsJinkCheck(*bb.state, *bb.self)) {
            bb.brain->setCurMode(DigiMode::GunsJink);
            bb.state->nav.flightPhase = FlightPhase::Combat;
            bb.brain->runGunsJink(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus TakeoffNode::onTick(Blackboard& bb) {
    const auto gp = bb.state->ag.groundOps.phase;
    const bool isTakeoff = (gp == GroundOpsPhase::TakeoffRoll || gp == GroundOpsPhase::Rotation ||
                            gp == GroundOpsPhase::AfterTakeoff || gp == GroundOpsPhase::LiningUp ||
                            gp == GroundOpsPhase::TaxiToRunway || gp == GroundOpsPhase::HoldingShort ||
                            gp == GroundOpsPhase::Parking || gp == GroundOpsPhase::RequestTaxi);
    if (isTakeoff) {
        bb.brain->setCurMode(DigiMode::Takeoff);
        bb.state->nav.flightPhase = FlightPhase::GroundOps;
        bb.brain->runTakeoff(*bb.as, bb.dt, *bb.fcsState, bb.groundZ);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus LandingNode::onTick(Blackboard& bb) {
    const auto gp = bb.state->ag.groundOps.phase;
    const bool isLanding = (gp == GroundOpsPhase::Approach || gp == GroundOpsPhase::Flare ||
                            gp == GroundOpsPhase::Touchdown || gp == GroundOpsPhase::Rollout ||
                            gp == GroundOpsPhase::VacatingRunway);
    if (isLanding) {
        bb.brain->setCurMode(DigiMode::Landing);
        bb.state->nav.flightPhase = FlightPhase::Approach;
        bb.brain->runLanding(*bb.as, bb.dt, *bb.fcsState, bb.groundZ);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus FollowOrdersNode::onTick(Blackboard& bb) {
    const bool followOrdersActive =
        bb.state->formation.isWing &&
        bb.state->formation.wingman.currentManeuver != WingmanManeuver::None;
    if (followOrdersActive) {
        if (bb.state->nav.mnverTime <= 0.0) {
            bb.state->nav.mnverTime = kDefaultManeuverTimeSec;
        }
        bb.brain->setCurMode(DigiMode::FollowOrders);
        bb.state->nav.flightPhase = FlightPhase::Formation;
        bb.brain->runFollowOrders(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus RTBNode::onTick(Blackboard& bb) {
    const bool fuelCritical =
        (bb.state->fuel.phase == DigiFuelState::Phase::Bingo ||
         bb.state->fuel.phase == DigiFuelState::Phase::Fumes ||
         bb.state->fuel.phase == DigiFuelState::Phase::Flameout);
    const bool winchesterRTB =
        bb.state->fuel.winchester &&
        !bb.state->missileDefeat.incomingMissile &&
        !bb.state->gunsJink.gunsThreat;
    const bool isRtbTask = (bb.flightPlan && !bb.flightPlan->isComplete() && bb.flightPlan->currentTask().type == TaskType::RTB);

    if (fuelCritical || winchesterRTB || isRtbTask) {
        if (bb.self) {
            const AirbaseAction abAction = AirbaseCheck(*bb.state, *bb.self, bb.brain->frameInputs(), bb.simTime);
            if (abAction == AirbaseAction::Landing) {
                bb.brain->setCurMode(DigiMode::Landing);
                bb.state->nav.flightPhase = FlightPhase::Approach;
                bb.brain->runLanding(*bb.as, bb.dt, *bb.fcsState, bb.groundZ);
                return NodeStatus::Running;
            } else if (abAction == AirbaseAction::RTB || fuelCritical || winchesterRTB || isRtbTask) {
                bb.brain->setCurMode(DigiMode::RTB);
                bb.state->nav.flightPhase = FlightPhase::Cruise;
                bb.brain->runRTB(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
                return NodeStatus::Running;
            }
        } else {
            bb.brain->setCurMode(DigiMode::RTB);
            bb.state->nav.flightPhase = FlightPhase::Cruise;
            bb.brain->runRTB(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus CombatNode::onTick(Blackboard& bb) {
    const DigiEntity* tgt = bb.brain->frameInputs().injectedTarget;
    if (tgt && tgt->isDead) tgt = nullptr;

    const SensorPicture& pic = bb.brain->sensorFusion().picture();
    const bool sensorFusionActive = (bb.brain->frameInputs().truth != nullptr);

    if (sensorFusionActive && bb.brain->targetEntityAuto().has_value() &&
        tgt == &(*bb.brain->targetEntityAuto()) && !pic.bestTarget) {
        tgt = nullptr;
        bb.brain->clearTargetEntityAuto();
    }

    if (!tgt && pic.bestTarget) {
        bb.brain->setTargetEntityAuto(toDigiEntity(*pic.bestTarget));
        tgt = &(*bb.brain->targetEntityAuto());
    }

    if (!tgt && sensorFusionActive && bb.self) {
        const DigiEntity* autoTarget = DoTargeting(*bb.state, pic, *bb.self);
        if (autoTarget) {
            tgt = autoTarget;
        }
    }

    if (!tgt && bb.self) {
        if (sensorFusionActive) {
            const DigiEntity* autoTarget = DoTargeting(*bb.state, pic, *bb.self);
            if (autoTarget) {
                tgt = autoTarget;
            }
        }
    }

    if (bb.self && tgt && !tgt->isDead) {
        const double dx = tgt->x - bb.self->x;
        const double dy = tgt->y - bb.self->y;
        const double dz = tgt->z - bb.self->z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        const double maxAAWpnRangeFt = bb.state->weapon.maxAAWpnRange > 0
            ? bb.state->weapon.maxAAWpnRange
            : 35.0 * 6076.0;

        bb.brain->setWvrTarget(tgt);

        if (BvrEngageCheck(*bb.state, *bb.self, *tgt, maxAAWpnRangeFt)) {
            bb.brain->setCurMode(DigiMode::BVREngage);
            bb.state->nav.flightPhase = FlightPhase::Combat;
            bb.brain->runBVREngage(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
        else if (range < maxAAWpnRangeFt) {
            if (bb.brain->sms() && bb.brain->sms()->hasWeaponClass(WeaponClass::AimWpn)) {
                if (MissileEngageCheck(*bb.state, *bb.self, *tgt, *bb.brain->sms(), true) && range > 3500.0) {
                    bb.brain->setCurMode(DigiMode::MissileEngage);
                    bb.state->nav.flightPhase = FlightPhase::Combat;
                    bb.brain->runMissileEngage(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
                    return NodeStatus::Running;
                }
            }

            if (MergeCheck(*bb.state, *bb.self, *tgt)) {
                bb.brain->setCurMode(DigiMode::Merge);
                bb.state->nav.flightPhase = FlightPhase::Combat;
                bb.brain->runMerge(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
                return NodeStatus::Running;
            }

            WeaponSpec gun = gunSpec();
            if (GunsEngageCheck(*bb.state, *bb.self, *tgt, gun, true)) {
                bb.brain->setCurMode(DigiMode::GunsEngage);
                bb.state->nav.flightPhase = FlightPhase::Combat;
                bb.brain->runGunsEngage(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
                return NodeStatus::Running;
            }

            bb.brain->setCurMode(DigiMode::WVREngage);
            bb.state->nav.flightPhase = FlightPhase::Combat;
            bb.brain->runWVREngage(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
            return NodeStatus::Running;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus WingyNode::onTick(Blackboard& bb) {
    const bool wingyActive =
        bb.state->formation.isWing &&
        bb.state->formation.flightLeadId != kInvalidEntityId &&
        bb.brain->frameInputs().injectedLead != nullptr;
    if (wingyActive) {
        bb.brain->setCurMode(DigiMode::Wingy);
        bb.state->nav.flightPhase = FlightPhase::Formation;
        bb.brain->runWingy(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus RefuelNode::onTick(Blackboard& bb) {
    const bool isRefuelTask = (bb.flightPlan && !bb.flightPlan->isComplete() && bb.flightPlan->currentTask().type == TaskType::Refuel);
    if (isRefuelTask || bb.brain->frameInputs().injectedTanker) {
        bb.brain->setCurMode(DigiMode::Refueling);
        bb.state->nav.flightPhase = FlightPhase::Formation;
        bb.brain->runRefueling(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

NodeStatus GroundMnvrNode::onTick(Blackboard& bb) {
    const bool isStrikeTask = (bb.flightPlan && !bb.flightPlan->isComplete() && bb.flightPlan->currentTask().type == TaskType::Strike);
    if (isStrikeTask || bb.state->ag.groundTarget) {
        bb.brain->setCurMode(DigiMode::GroundMnvr);
        bb.state->nav.flightPhase = FlightPhase::Combat;
        bb.brain->runGroundAttack(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
        return NodeStatus::Running;
    }
    return NodeStatus::Failure;
}

WaypointFollowNode::WaypointFollowNode(double captureRadius) 
    : BehaviorNode("WaypointFollow"), captureRadius_(captureRadius) {
    captureCheckNode_ = std::make_shared<WaypointCaptureCheckNode>(captureRadius_);
    activeTaskSelectorNode_ = std::make_shared<ActiveTaskSelectorNode>();
    activeTaskSelectorNode_->addTaskNode(TaskType::Navigate, std::make_shared<NavigateTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::CAP, std::make_shared<LoiterTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::Takeoff, std::make_shared<TakeoffTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::Landing, std::make_shared<LandingTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::RTB, std::make_shared<RTBTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::Refuel, std::make_shared<RefuelTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::Strike, std::make_shared<StrikeTaskNode>());
    activeTaskSelectorNode_->addTaskNode(TaskType::Assemble, std::make_shared<AssembleTaskNode>());
}

void WaypointFollowNode::reset() {
    BehaviorNode::reset();
    if (captureCheckNode_) captureCheckNode_->reset();
    if (activeTaskSelectorNode_) activeTaskSelectorNode_->reset();
}

NodeStatus WaypointFollowNode::onTick(Blackboard& bb) {
    if (bb.flightPlan && !bb.flightPlan->isComplete()) {
        bb.brain->setCurMode(DigiMode::Waypoint);
        bb.state->nav.flightPhase = FlightPhase::Cruise;
        captureCheckNode_->tick(bb);
        NodeStatus status = activeTaskSelectorNode_->tick(bb);
        if (status == NodeStatus::Running || status == NodeStatus::Success) {
            return status;
        }
    }
    return NodeStatus::Failure;
}

NodeStatus DefaultFallbackNode::onTick(Blackboard& bb) {
    bb.brain->setCurMode(DigiMode::Waypoint);
    bb.state->nav.flightPhase = FlightPhase::Cruise;
    bb.brain->runWaypoint(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);
    return NodeStatus::Running;
}

} // namespace digi
} // namespace f4flight
