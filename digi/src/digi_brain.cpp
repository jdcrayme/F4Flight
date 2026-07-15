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
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/digi/ground/ground_ops.h"
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
    }

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
                // Documented no-op: ground avoidance runs as a concurrent
                // overlay in compute() (RunGroundAvoid → pullingUp), not as
                // a dispatched mode. This case is reachable only via
                // forceMode(GroundAvoid) when there is no terrain danger.
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

            // -----------------------------------------------------------------
            // Round-2 structural additions (Rec 6): dispatch stubs for the 9
            // new DigiMode values. Each falls through to Waypoint navigation
            // until its behavior is ported. This means the brain can RESOLVE
            // to these modes (so future porting work can incrementally wire
            // them up) without producing dead code or surprise behavior.
            //
            // The 4 modes with primitive targets (Roop, OverB, Loiter, Bugout)
            // DO call their primitive — so the primitive is exercised even
            // before the full mode logic lands.
            // -----------------------------------------------------------------
            case DigiMode::Roop: {
                // selfEntity is already resolved at the top of compute()
                // (frameInputs_.selfEntity or &selfEntityAuto_). Reuse it.
                if (selfEntity && wvrTarget_) {
                    const bool stillActive = ManeuverPrimitives::RollOutOfPlane(
                        state_, *selfEntity, as, fcs, fcsState, dt,
                        /*firstFrame=*/false);
                    if (!stillActive) {
                        // Maneuver timer expired — fall back to waypoint nav
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
                // Loiter already exists as a ManeuverPrimitives method — use it.
                ManeuverPrimitives::Loiter(state_, as, fcs, fcsState, state_.config.maxGs);
                ManeuverPrimitives::MachHold(state_.config.cornerSpeed, as.vcas, true,
                                              state_, as, 200.0, 800.0, dt, 700.0);
                break;
            case DigiMode::Bugout:
            case DigiMode::Separate:
                // Both modes disengage — use WvrBugOut (hold heading + alt,
                // accelerate to 2x corner speed). The full separate.cpp port
                // will add RangeAtTailChase geometry + bugoutTimer.
                ManeuverPrimitives::WvrBugOut(state_, as, fcs, fcsState, dt);
                break;
            case DigiMode::Refueling:
                // Not yet ported — fall through to Waypoint navigation.
                runWaypoint(as, dt, fcs, fcsState);
                break;
            case DigiMode::FollowOrders:
                // PORTED (basic): execute the wingman's current tactical
                // maneuver (BreakLeft/Right, ClearSix, Posthole, Chainsaw)
                // via AiPerformManeuver. If no maneuver is active, falls
                // back to Wingy (formation following).
                runFollowOrders(as, dt, fcs, fcsState);
                break;
            case DigiMode::RTB:
                // PORTED (basic): navigate to the divert airbase via
                // HeadingAndAltitudeHold + MachHold. Falls back to waypoint
                // nav if the host hasn't set a divert airbase. Full RTB
                // (AirbaseCheck + landing clearance request) lands with the
                // landme.cpp pattern-work port.
                runRTB(as, dt, fcs, fcsState);
                break;
            case DigiMode::Wingy:
                // Wingman formation following — delegates to AiFollowLead.
                runWingy(as, dt, fcs, fcsState);
                break;
            case DigiMode::GroundMnvr:
                // Not yet ported — fall through to Waypoint navigation.
                runWaypoint(as, dt, fcs, fcsState);
                break;
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
        } else if (state_.fuel.jokerFuelLbs > 0.0 && state_.fuel.fuelLbs <= state_.fuel.jokerFuelLbs) {
            state_.fuel.phase = DigiFuelState::Phase::Joker;
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
    // Priority: injected target > SensorPicture bestTarget
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
    } else {
        // --- Waypoint (lowest-priority fallback) ---
        // FF RunDecisionRoutines ends by queuing WaypointMode as the default.
        // addMode() priority resolution ensures any higher-priority mode queued
        // above wins; if nothing matched, Waypoint wins.
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

    // BUG FIX: Waypoint is the lowest-priority fallback — it must NEVER
    // pre-empt any other queued mode. The Round-2 structural additions
    // (Refueling=13 ... GroundMnvr=22) were placed AFTER Waypoint=12 in the
    // enum, so a naive `newMode < nextMode_` check would let Waypoint (12)
    // incorrectly pre-empt RTB (19), Wingy (20), etc. — modes that should
    // always win over Waypoint. Without this special-case, fuel-critical
    // RTB and formation Wingy are silently overridden by Waypoint nav every
    // frame, leaving the AI to fly past its divert airbase and out of
    // formation indefinitely.
    //
    // FreeFalcon's AddMode avoids this by giving Waypoint the highest
    // numerical value (lowest priority) in its mode enum. We preserve the
    // existing enum ordering for backward compatibility with tests that
    // assert `WVREngage < Waypoint`, and instead special-case Waypoint here.
    if (newMode == DigiMode::Waypoint && nextMode_ != DigiMode::NoMode) {
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
}

// ===========================================================================
// Per-mode runners
// ===========================================================================
void DigiBrain::runWaypoint(const AircraftState& as, double dt,
                             const FlightControlSystem& fcs, FcsState& fcsState) {
    if (curWp_ >= wps_.size()) {
        ManeuverPrimitives::HeadingAndAltitudeHold(state_.nav.holdPsi, state_.nav.holdAlt,
                                                    state_, as, fcs, fcsState, state_.config.maxGs);
        ManeuverPrimitives::MachHold(state_.config.cornerSpeed, as.vcas, true,
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
            ManeuverPrimitives::MachHold(state_.config.cornerSpeed, as.vcas, true,
                                          state_, as, 200.0, 800.0, dt, 700.0);
            return;
        }
    }

    const double desHeading = std::atan2(dy, dx);
    const double desAlt = -wp.z;
    ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
                                                state_, as, fcs, fcsState, state_.config.maxGs);
    ManeuverPrimitives::MachHold(state_.config.cornerSpeed, as.vcas, true,
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
    ManeuverPrimitives::MachHold(state_.config.cornerSpeed, as.vcas, true,
                                  state_, as, 200.0, 800.0, dt, 700.0);

    // TODO (future): when within ~10 NM of the airbase, transition to
    // Landing mode and request landing clearance via the MessageBus. This
    // requires the AirbaseCheck port from FF separate.cpp / landme.cpp.
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

} // namespace digi
} // namespace f4flight
