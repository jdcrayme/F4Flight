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
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/offensive/missile_engage.h"
#include "f4flight/digi/offensive/bvr_engage.h"
#include "f4flight/digi/offensive/merge.h"
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
    state_.skill = makeSkillParams(SkillLevel::Veteran);
}

// ===========================================================================
// Configuration
// ===========================================================================
void DigiBrain::configure(const DigiConfig& cfg) {
    state_.skill          = makeSkillParams(cfg.skillLevel);
    state_.cornerSpeed    = cfg.cornerSpeedKts;
    state_.maxGs          = cfg.maxGs;
    state_.maxRoll        = cfg.maxBankDeg;
    state_.maxGammaDeg    = cfg.maxGammaDeg;
    state_.turnLoadFactor = cfg.turnLoadFactor;
}

DigiConfig DigiBrain::config() const {
    DigiConfig cfg;
    cfg.skillLevel     = state_.skill.level;
    cfg.cornerSpeedKts = state_.cornerSpeed;
    cfg.maxGs          = state_.maxGs;
    cfg.maxBankDeg     = state_.maxRoll;
    cfg.maxGammaDeg    = state_.maxGammaDeg;
    cfg.turnLoadFactor = state_.turnLoadFactor;
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
    return e;
}

// ===========================================================================
// Commands (asynchronous)
// ===========================================================================
void DigiBrain::commandTakeoff(RunwayId rwy, double rwyHeading,
                                double rwyThresholdX, double rwyThresholdY,
                                double rwyAlt) {
    auto& go = state_.groundOps;
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
    auto& go = state_.groundOps;
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
    state_.dt = dt;
    simTime_ += dt;

    // --- Clear fire flags at frame start (FF digimain.cpp:599) ---
    state_.gunFireFlag = false;
    state_.mslFireFlag = false;

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
        state_.incomingMissile = frameInputs_.injectedMissile;
        // Reset per-missile state if the injected missile changed.
        // (The deprecated setIncomingMissile shim already does this; for
        //  the new setFrameInputs path, we do it here on first sight.)
        if (state_.missileDefeatTtgo < 0.0) {
            // Already in "new missile" state — nothing to do.
        }
    } else if (frameInputs_.injectedGunsThreat) {
        // Only clear missile if no injected missile AND no sensor fusion
        // (sensor fusion path handles its own missile tracking below).
        // Don't clear here — sensor fusion may still be tracking.
    }

    if (frameInputs_.injectedGunsThreat) {
        state_.gunsThreat = frameInputs_.injectedGunsThreat;
    }

    // --- Run sensor fusion if truth state is provided ---
    // This builds a SensorPicture the brain uses for autonomous detection.
    // If injected threats are also set, resolveMode() gives them priority.
    const TruthState* truth = frameInputs_.truth;
    if (truth && selfEntity) {
        sensorFusion_.update(*selfEntity, *truth, state_.skill, dt);
    }

    // Process incoming messages (ATC clearances, flight commands)
    ProcessATCMessages(state_, state_.mailbox);

    // --- 1. Ground avoidance ---
    // FreeFalcon explicitly disables GroundCheck during LandingMode AND
    // TakeoffMode (dlogic.cpp:49-52: the ATC state machine owns terrain
    // logic for both). TakeoffRoll/Rotation/AfterTakeoff operate on the
    // ground or in the early climbout where the aircraft is below the 500 ft
    // kMinClearance threshold — GroundCheck would trigger a PullUp every
    // frame, fighting the takeoff roll with max-G pitch commands and
    // producing the erratic pitch/yaw seen at takeoff start.
    //
    // We check state_.groundOps.phase (set by commandTakeoff/commandLanding)
    // rather than curMode_ (which hasn't been resolved yet this frame).
    const bool isGroundOps = (state_.groundOps.phase == GroundOpsPhase::TakeoffRoll ||
                              state_.groundOps.phase == GroundOpsPhase::Rotation ||
                              state_.groundOps.phase == GroundOpsPhase::AfterTakeoff ||
                              state_.groundOps.phase == GroundOpsPhase::LiningUp ||
                              state_.groundOps.phase == GroundOpsPhase::Approach ||
                              state_.groundOps.phase == GroundOpsPhase::Flare ||
                              state_.groundOps.phase == GroundOpsPhase::Touchdown ||
                              state_.groundOps.phase == GroundOpsPhase::Rollout ||
                              state_.groundOps.phase == GroundOpsPhase::VacatingRunway);

    bool pullingUp = false;
    if (isGroundOps) {
        // Ground ops owns terrain logic — suppress ground avoid.
        state_.groundAvoidNeeded = false;
        state_.pullupTimer = 0.0;
    } else {
        pullingUp = RunGroundAvoid(state_, as, groundZ,
                                   state_.cornerSpeed, dt,
                                   fcsState, state_.maxGs);
    }

    // --- 2. Resolve mode ---
    // Pass the resolved selfEntity so resolveMode doesn't need to re-resolve.
    resolveMode(as, groundZ, dt);

    // --- 3. Actions ---
    if (!pullingUp) {
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
                ManeuverPrimitives::Loiter(state_, as, fcs, fcsState, state_.maxGs);
                ManeuverPrimitives::MachHold(state_.cornerSpeed, as.vcas, true,
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
            case DigiMode::FollowOrders:
            case DigiMode::RTB:
            case DigiMode::Wingy:
            case DigiMode::GroundMnvr:
                // Not yet ported — fall through to Waypoint navigation.
                // (Each of these needs a substantial port: refuel.cpp,
                // wingman system, AirbaseCheck, formdata.cpp, gndattck.cpp.)
                runWaypoint(as, dt, fcs, fcsState);
                break;
        }
    }

    // --- 4. Clamp outputs + map fire flags ---
    out.pstick = limit(state_.pStick, -1.0, 1.0);
    out.rstick = limit(state_.rStick, -1.0, 1.0);
    out.ypedal = limit(state_.yPedal, -1.0, 1.0);
    out.throttle = limit(state_.throttle, 0.0, 1.5);
    // Round-2 structural fix (Rec 7): map digi brake / speed-brake / gear
    // commands to PilotInput. Previously the brain wrote only pStick/
    // rStick/yPedal/throttle — PilotInput.wheelBrakes / speedBrake /
    // gearHandle were dead fields. Now RunLanding::Rollout can actually
    // command wheel brakes, and a future A/G dive-bomb mode can command
    // speed brakes.
    out.wheelBrakes  = state_.wheelBrakes;
    out.parkingBrake = state_.parkingBrake;
    out.speedBrake   = state_.speedBrakeCmd;
    out.gearHandle   = state_.gearHandleCmd;
    out.refueling = false;
    // Map digi fire flags to PilotInput (host reads these to fire weapons)
    out.fireGun        = state_.gunFireFlag;
    out.releaseConsent = state_.mslFireFlag;
    out.weaponStation  = state_.fireStation;
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
    //   c. Otherwise, fall back to whatever state_.incomingMissile points
    //      to (set by deprecated shim or prior frame).
    //
    // We COMMIT the pointer to state_.incomingMissile so that
    // runMissileDefeat sees it, and MissileDefeatCheck's clearing
    // side-effect persists.

    // Step a: host-injected missile
    if (frameInputs_.injectedMissile) {
        state_.incomingMissile = frameInputs_.injectedMissile;
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
            state_.incomingMissile == &(*missileEntityAuto_);

        // Clear auto-tracked missile if SensorFusion no longer sees one.
        if (sensorFusionActive && autoTracking && !pic.incomingMissile) {
            state_.incomingMissile = nullptr;
            state_.incomingMissileId = kInvalidEntityId;
            missileEntityAuto_.reset();
        }
        // SensorFusion sees a missile we're not tracking (or a different one).
        else if (sensorFusionActive && pic.incomingMissile &&
                 (!autoTracking ||
                  pic.incomingMissile->entityId != state_.incomingMissileId)) {
            missileEntityAuto_ = DigiEntity{};
            missileEntityAuto_->x  = pic.incomingMissile->x;
            missileEntityAuto_->y  = pic.incomingMissile->y;
            missileEntityAuto_->z  = pic.incomingMissile->z;
            missileEntityAuto_->vx = pic.incomingMissile->vx;
            missileEntityAuto_->vy = pic.incomingMissile->vy;
            missileEntityAuto_->vz = pic.incomingMissile->vz;
            missileEntityAuto_->yaw       = pic.incomingMissile->yaw;
            missileEntityAuto_->speed     = pic.incomingMissile->speed;
            missileEntityAuto_->seekerType = DigiEntity::SeekerType::Radar;
            missileEntityAuto_->isDead    = false;
            state_.incomingMissile = &(*missileEntityAuto_);

            // Reset per-missile state on identity change.
            const EntityId newId = pic.incomingMissile->entityId;
            if (newId != state_.incomingMissileId) {
                state_.incomingMissileId = newId;
                state_.missileDefeatTtgo = -1.0;
                state_.incomingMissileEvadeTimer = 0.0;
            }
        }
        // Same missile — refresh position/velocity.
        else if (sensorFusionActive && autoTracking && pic.incomingMissile &&
                 pic.incomingMissile->entityId == state_.incomingMissileId) {
            missileEntityAuto_->x  = pic.incomingMissile->x;
            missileEntityAuto_->y  = pic.incomingMissile->y;
            missileEntityAuto_->z  = pic.incomingMissile->z;
            missileEntityAuto_->vx = pic.incomingMissile->vx;
            missileEntityAuto_->vy = pic.incomingMissile->vy;
            missileEntityAuto_->vz = pic.incomingMissile->vz;
            missileEntityAuto_->yaw       = pic.incomingMissile->yaw;
            missileEntityAuto_->speed     = pic.incomingMissile->speed;
        }
    }

    if (selfEntity && state_.incomingMissile) {
        if (MissileDefeatCheck(state_, *selfEntity, dt)) {
            addMode(DigiMode::MissileDefeat);
        }
    }

    // ===================================================================
    // --- Guns threat ---
    // ===================================================================
    if (frameInputs_.injectedGunsThreat) {
        state_.gunsThreat = frameInputs_.injectedGunsThreat;
        gunsEntityAuto_.reset();
    } else {
        // Clear auto-tracked guns threat if SensorFusion no longer sees one.
        if (sensorFusionActive && gunsEntityAuto_.has_value() &&
            state_.gunsThreat == &(*gunsEntityAuto_) &&
            !pic.gunsThreat) {
            state_.gunsThreat = nullptr;
            gunsEntityAuto_.reset();
        }

        if (!state_.gunsThreat && pic.gunsThreat) {
            gunsEntityAuto_ = DigiEntity{};
            gunsEntityAuto_->x  = pic.gunsThreat->x;
            gunsEntityAuto_->y  = pic.gunsThreat->y;
            gunsEntityAuto_->z  = pic.gunsThreat->z;
            gunsEntityAuto_->vx = pic.gunsThreat->vx;
            gunsEntityAuto_->vy = pic.gunsThreat->vy;
            gunsEntityAuto_->vz = pic.gunsThreat->vz;
            gunsEntityAuto_->yaw    = pic.gunsThreat->yaw;
            gunsEntityAuto_->speed  = pic.gunsThreat->speed;
            gunsEntityAuto_->isFiring = true;
            gunsEntityAuto_->isDead   = false;
            state_.gunsThreat = &(*gunsEntityAuto_);
        }
    }

    if (selfEntity && state_.gunsThreat) {
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
    const auto gp = state_.groundOps.phase;
    if (gp == GroundOpsPhase::TakeoffRoll || gp == GroundOpsPhase::Rotation ||
        gp == GroundOpsPhase::AfterTakeoff || gp == GroundOpsPhase::LiningUp) {
        addMode(DigiMode::Takeoff);
    }
    if (gp == GroundOpsPhase::Approach || gp == GroundOpsPhase::Flare ||
        gp == GroundOpsPhase::Touchdown || gp == GroundOpsPhase::Rollout ||
        gp == GroundOpsPhase::VacatingRunway) {
        addMode(DigiMode::Landing);
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
        targetEntityAuto_ = DigiEntity{};
        targetEntityAuto_->x = pic.bestTarget->x;
        targetEntityAuto_->y = pic.bestTarget->y;
        targetEntityAuto_->z = pic.bestTarget->z;
        targetEntityAuto_->vx = pic.bestTarget->vx;
        targetEntityAuto_->vy = pic.bestTarget->vy;
        targetEntityAuto_->vz = pic.bestTarget->vz;
        targetEntityAuto_->yaw = pic.bestTarget->yaw;
        targetEntityAuto_->pitch = pic.bestTarget->pitch;
        targetEntityAuto_->roll = pic.bestTarget->roll;
        targetEntityAuto_->speed = pic.bestTarget->speed;
        targetEntityAuto_->isDead = false;
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
        const double maxAAWpnRangeFt = state_.maxAAWpnRange > 0
            ? state_.maxAAWpnRange
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

    // --- Waypoint (lowest-priority fallback) ---
    // FF RunDecisionRoutines ends by queuing WaypointMode as the default.
    // addMode() priority resolution ensures any higher-priority mode queued
    // above wins; if nothing matched, Waypoint wins.
    addMode(DigiMode::Waypoint);

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
        ManeuverPrimitives::HeadingAndAltitudeHold(state_.holdPsi, state_.holdAlt,
                                                    state_, as, fcs, fcsState, state_.maxGs);
        ManeuverPrimitives::MachHold(state_.cornerSpeed, as.vcas, true,
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
            ManeuverPrimitives::HeadingAndAltitudeHold(state_.holdPsi, state_.holdAlt,
                                                        state_, as, fcs, fcsState, state_.maxGs);
            ManeuverPrimitives::MachHold(state_.cornerSpeed, as.vcas, true,
                                          state_, as, 200.0, 800.0, dt, 700.0);
            return;
        }
    }

    const double desHeading = std::atan2(dy, dx);
    const double desAlt = -wp.z;
    ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
                                                state_, as, fcs, fcsState, state_.maxGs);
    ManeuverPrimitives::MachHold(state_.cornerSpeed, as.vcas, true,
                                  state_, as, 200.0, 800.0, dt, 700.0);
}

void DigiBrain::runMissileDefeat(const AircraftState& as, double dt,
                                  const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity || !state_.incomingMissile) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    MissileDefeat(state_, *selfEntity, as, fcs, fcsState, dt);
}

void DigiBrain::runGunsJink(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    const DigiEntity* selfEntity = frameInputs_.selfEntity;
    if (!selfEntity) selfEntity = &selfEntityAuto_;
    if (!selfEntity || !state_.gunsThreat) {
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
    double maxAAWpnRangeFt = state_.maxAAWpnRange;
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

} // namespace digi
} // namespace f4flight
