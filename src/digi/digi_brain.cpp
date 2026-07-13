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
//   - WVREngage:    STUB (no offensive targets yet — returns level flight)

#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

DigiBrain::DigiBrain() {
    state_.reset();
    state_.skill = makeSkillParams(SkillLevel::Veteran);
}

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

PilotInput DigiBrain::compute(const AircraftState& as, double dt, double groundZ,
                               const FlightControlSystem& fcs, FcsState& fcsState) {
    PilotInput out;
    state_.dt = dt;
    simTime_ += dt;

    // Auto-sync self entity from AircraftState every frame.
    if (!selfEntityExplicit_) {
        selfEntityAuto_ = buildSelfEntity(as);
        selfEntity_ = &selfEntityAuto_;
    }

    // Run sensor fusion if truth state is provided.
    // This builds a SensorPicture the brain uses for autonomous detection.
    if (truth_ && selfEntity_) {
        sensorFusion_.update(*selfEntity_, *truth_, state_.skill, dt);
    }

    // Process incoming messages (ATC clearances, flight commands)
    ProcessATCMessages(state_, state_.mailbox);

    // --- 1. Ground avoidance (always runs, pre-empts everything) ---
    const bool pullingUp = RunGroundAvoid(state_, as, groundZ,
                                          state_.cornerSpeed, dt,
                                          fcsState, state_.maxGs);

    // --- 2. Resolve mode ---
    resolveMode(as, groundZ, dt);

    // --- 3. Actions ---
    if (!pullingUp) {
        switch (activeMode_) {
            case DigiMode::Waypoint:
                runWaypoint(as, dt, fcs, fcsState);
                break;
            case DigiMode::MissileDefeat:
                runMissileDefeat(as, dt, fcs, fcsState);
                break;
            case DigiMode::GunsJink:
                runGunsJink(as, dt, fcs, fcsState);
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
            case DigiMode::NoMode:
                runWaypoint(as, dt, fcs, fcsState);
                break;
        }
    }

    // --- 4. Clamp outputs ---
    out.pstick = limit(state_.pStick, -1.0, 1.0);
    out.rstick = limit(state_.rStick, -1.0, 1.0);
    out.ypedal = limit(state_.yPedal, -1.0, 1.0);
    out.throttle = limit(state_.throttle, 0.0, 1.5);
    out.refueling = false;
    return out;
}

void DigiBrain::resolveMode(const AircraftState& /*as*/, double /*groundZ*/,
                             double dt) {
    // If a mode is forced (testing), use it.
    if (forcedMode_ != DigiMode::NoMode) {
        activeMode_ = forcedMode_;
        return;
    }

    // Priority stack (highest priority first):
    //   1. MissileDefeat  — incoming missile
    //   2. GunsJink       — guns threat
    //   3. Takeoff/Landing — ground ops (when active)
    //   4. WVREngage      — target within visual range
    //   5. Waypoint       — default navigation
    // GroundAvoid is handled separately in compute() (always pre-empts).

    // --- Threat detection ---
    // Use injected threats (backward compat) OR SensorPicture (autonomous).
    // If truth_ is provided, SensorFusion has already built the picture.
    const SensorPicture& pic = sensorFusion_.picture();

    // Check for incoming missile
    // Priority: injected > SensorPicture
    const DigiEntity* missile = state_.incomingMissile;
    if (!missile && pic.incomingMissile) {
        // Build a DigiEntity from the SensorPicture contact
        missileEntityAuto_ = DigiEntity{};
        missileEntityAuto_->x = pic.incomingMissile->x;
        missileEntityAuto_->y = pic.incomingMissile->y;
        missileEntityAuto_->z = pic.incomingMissile->z;
        missileEntityAuto_->vx = pic.incomingMissile->vx;
        missileEntityAuto_->vy = pic.incomingMissile->vy;
        missileEntityAuto_->vz = pic.incomingMissile->vz;
        missileEntityAuto_->yaw = pic.incomingMissile->yaw;
        missileEntityAuto_->speed = pic.incomingMissile->speed;
        missileEntityAuto_->seekerType = DigiEntity::SeekerType::Radar;
        missileEntityAuto_->isDead = false;
        missile = &(*missileEntityAuto_);
    }

    if (selfEntity_ && missile) {
        // Temporarily set for MissileDefeatCheck
        const DigiEntity* savedMissile = state_.incomingMissile;
        state_.incomingMissile = missile;
        if (MissileDefeatCheck(state_, *selfEntity_, dt)) {
            activeMode_ = DigiMode::MissileDefeat;
            state_.incomingMissile = savedMissile;
            return;
        }
        state_.incomingMissile = savedMissile;
    }

    // Check for guns threat
    const DigiEntity* gunsThreat = state_.gunsThreat;
    if (!gunsThreat && pic.gunsThreat) {
        gunsEntityAuto_ = DigiEntity{};
        gunsEntityAuto_->x = pic.gunsThreat->x;
        gunsEntityAuto_->y = pic.gunsThreat->y;
        gunsEntityAuto_->z = pic.gunsThreat->z;
        gunsEntityAuto_->vx = pic.gunsThreat->vx;
        gunsEntityAuto_->vy = pic.gunsThreat->vy;
        gunsEntityAuto_->vz = pic.gunsThreat->vz;
        gunsEntityAuto_->yaw = pic.gunsThreat->yaw;
        gunsEntityAuto_->speed = pic.gunsThreat->speed;
        gunsEntityAuto_->isFiring = true;
        gunsEntityAuto_->isDead = false;
        gunsThreat = &(*gunsEntityAuto_);
    }

    if (selfEntity_ && gunsThreat) {
        const DigiEntity* savedGuns = state_.gunsThreat;
        state_.gunsThreat = gunsThreat;
        if (GunsJinkCheck(state_, *selfEntity_)) {
            activeMode_ = DigiMode::GunsJink;
            state_.gunsThreat = savedGuns;
            return;
        }
        state_.gunsThreat = savedGuns;
    }

    // Check for active ground ops (takeoff/landing)
    const auto gp = state_.groundOps.phase;
    if (gp == GroundOpsPhase::TakeoffRoll || gp == GroundOpsPhase::Rotation ||
        gp == GroundOpsPhase::AfterTakeoff || gp == GroundOpsPhase::LiningUp) {
        activeMode_ = DigiMode::Takeoff;
        return;
    }
    if (gp == GroundOpsPhase::Approach || gp == GroundOpsPhase::Flare ||
        gp == GroundOpsPhase::Touchdown || gp == GroundOpsPhase::Rollout ||
        gp == GroundOpsPhase::VacatingRunway) {
        activeMode_ = DigiMode::Landing;
        return;
    }

    // Check for WVR target (Tier 2 — offensive)
    // Priority: injected target > SensorPicture bestTarget
    const DigiEntity* tgt = target_;
    // If injected target is dead, don't use it
    if (tgt && tgt->isDead) tgt = nullptr;

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

    if (selfEntity_ && tgt && !tgt->isDead) {
        const double dx = tgt->x - selfEntity_->x;
        const double dy = tgt->y - selfEntity_->y;
        const double dz = tgt->z - selfEntity_->z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (range < 8.0 * 6076.0) {
            // Set the target for runWVREngage (only if not already set by host)
            if (!target_ || target_->isDead) target_ = tgt;
            activeMode_ = DigiMode::WVREngage;
            return;
        }
    }

    // Default: waypoint navigation
    activeMode_ = DigiMode::Waypoint;
}

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

void DigiBrain::runGroundAvoid(const AircraftState& as, double dt,
                                FcsState& fcsState) {
    RunGroundAvoid(state_, as, 0.0, state_.cornerSpeed, dt,
                   fcsState, state_.maxGs);
}

void DigiBrain::runMissileDefeat(const AircraftState& as, double dt,
                                  const FlightControlSystem& fcs, FcsState& fcsState) {
    if (!selfEntity_ || !state_.incomingMissile) {
        // No threat — fall back to waypoint
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    MissileDefeat(state_, *selfEntity_, as, fcs, fcsState, dt);
}

void DigiBrain::runGunsJink(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    if (!selfEntity_ || !state_.gunsThreat) {
        // No threat — fall back to waypoint
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    GunsJink(state_, *selfEntity_, as, fcs, fcsState, dt);
}

void DigiBrain::runWVREngage(const AircraftState& as, double dt,
                              const FlightControlSystem& fcs, FcsState& fcsState) {
    if (!selfEntity_ || !target_ || target_->isDead) {
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    RollAndPull(state_, *selfEntity_, *target_, as, fcs, fcsState, dt);
}

void DigiBrain::startTakeoff(RunwayId rwy, double rwyHeading,
                               double rwyThresholdX, double rwyThresholdY, double rwyAlt) {
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

void DigiBrain::startLanding(RunwayId rwy, double rwyHeading,
                               double rwyThresholdX, double rwyThresholdY, double rwyAlt) {
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
