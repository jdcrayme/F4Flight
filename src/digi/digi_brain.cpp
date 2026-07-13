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
#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
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

    // Auto-sync self entity from AircraftState every frame. This lets the
    // brain compute relative geometry to threats without the host having to
    // manually populate a DigiEntity each frame. If the host HAS set a
    // selfEntity_, we use that instead (the host is responsible for keeping
    // it current).
    if (!selfEntityExplicit_) {
        selfEntityAuto_ = buildSelfEntity(as);
        selfEntity_ = &selfEntityAuto_;
    }

    // --- 1. Ground avoidance (always runs, pre-empts everything) ---
    const bool pullingUp = RunGroundAvoid(state_, as, groundZ,
                                          state_.cornerSpeed, dt,
                                          fcsState, state_.maxGs);

    // --- 2. Resolve mode ---
    resolveMode(as, groundZ, dt);

    // --- 3. Actions ---
    // If ground avoidance commanded a pull-up, it overrides the active mode.
    // (GroundAvoid is priority 0 — highest.)
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
            case DigiMode::GroundAvoid:
                // Already handled above; shouldn't reach here.
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
    //   3. WVREngage      — target within visual range (< 8 NM)
    //   4. Waypoint       — default navigation
    // GroundAvoid is handled separately in compute() (always pre-empts).

    // Check for incoming missile
    if (selfEntity_ && state_.incomingMissile) {
        if (MissileDefeatCheck(state_, *selfEntity_, dt)) {
            activeMode_ = DigiMode::MissileDefeat;
            return;
        }
    }

    // Check for guns threat
    if (selfEntity_ && state_.gunsThreat) {
        if (GunsJinkCheck(state_, *selfEntity_)) {
            activeMode_ = DigiMode::GunsJink;
            return;
        }
    }

    // Check for WVR target (Tier 2 — offensive)
    // WVR range: within 8 NM and not dead
    if (selfEntity_ && target_ && !target_->isDead) {
        const double dx = target_->x - selfEntity_->x;
        const double dy = target_->y - selfEntity_->y;
        const double dz = target_->z - selfEntity_->z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (range < 8.0 * 6076.0) {  // 8 NM WVR threshold
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
        // No target — fall back to waypoint
        runWaypoint(as, dt, fcs, fcsState);
        return;
    }
    // RollAndPull is the universal offensive BFM routine.
    // It selects behavior based on relative geometry (offensive/neutral/defensive)
    // and manages energy via EnergyManagement / MaintainClosure.
    RollAndPull(state_, *selfEntity_, *target_, as, fcs, fcsState, dt);
}

} // namespace digi
} // namespace f4flight
