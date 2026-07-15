// f4flight - digi/offensive/roll_and_pull.cpp
//
// Implementation of RollAndPull — the universal offensive BFM routine.
//
// Direct port of FreeFalcon randp.cpp (612 LOC), simplified to use the
// DigiEntity model. The maneuver selection logic, trackpoint geometry, and
// energy management are faithful ports.
//
// Key simplifications vs. FreeFalcon:
//   - No PitchDelta/YawDelta (target maneuvering detection). We approximate
//     "target is maneuvering" as false (non-maneuvering), which means
//     EnergyManagement uses the simpler non-maneuvering path.
//   - No SetTrackPoint caching (trackX/trackY/trackZ). We call TrackPoint
//     directly each frame.
//   - No debug labels (MANEUVER_DEBUG).

#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/digi/steering.h"  // for headingError

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// CollisionTime
// ===========================================================================
double CollisionTime(const DigiEntity& self, const DigiEntity& target) {
    const double dx = target.x - self.x;
    const double dy = target.y - self.y;
    const double dz = target.z - self.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double vt = std::max(1.0, self.speed);
    // FreeFalcon: tc = min(range / vt, 0.5)
    return std::min(range / vt, 0.5);
}

// ===========================================================================
// MaintainClosure
// ===========================================================================
void MaintainClosure(DigiState& digi, const DigiEntity& self,
                      const DigiEntity& target, const AircraftState& as,
                      const FlightControlSystem& /*fcs*/, FcsState& /*fcsState*/,
                      double dt) {
    // Port of randp.cpp:553-601
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Range to control point
    double rng;
    if (rg.ataFrom >= 90.0 * DTR) {
        rng = rg.range - kControlPointDistance;
    } else {
        rng = rg.range - 6000.0 - kControlPointDistance;
    }

    // Current closure in kts (rangedot is ft/s, negative = closing)
    const double rngdot = -rg.rangedot * FTPSEC_TO_KNOTS;  // positive = closing

    // Desired closure based on range (farmer range*closure function)
    double closure = (((rng - rngdot * 5.0) / 1000.0) * 50.0);
    closure = std::max(std::min(closure, 1000.0), -350.0);

    if (rg.range < 2500.0) {
        closure = std::min(closure, 30.0);
    }

    // Mach hold command
    const double currentKias = as.vcas;
    if (closure - rngdot > 0.0) {
        // Need more closure — speed up (but cap at corner speed if far + closing)
        double targetSpeed;
        if (rg.range > 2.0 * 6076.0 && rngdot > 0 &&
            currentKias + (closure - rngdot) > digi.config.cornerSpeed) {
            targetSpeed = digi.config.cornerSpeed;
        } else {
            targetSpeed = currentKias + (closure - rngdot);
        }
        ManeuverPrimitives::MachHold(targetSpeed, currentKias, false,
                                      digi, as, 200.0, 800.0, dt, 100.0);
    } else if (rg.range < 5000.0) {
        double targetSpeed = std::min(digi.config.cornerSpeed, currentKias + (closure - rngdot));
        ManeuverPrimitives::MachHold(targetSpeed, currentKias, false,
                                      digi, as, 200.0, 800.0, dt, 100.0);
    } else {
        double targetSpeed = currentKias + (closure - rngdot);
        ManeuverPrimitives::MachHold(targetSpeed, currentKias, false,
                                      digi, as, 200.0, 800.0, dt, 100.0);
    }
}

// ===========================================================================
// EnergyManagement
// ===========================================================================
void EnergyManagement(DigiState& digi, const DigiEntity& self,
                       const DigiEntity& target, const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt) {
    // Port of randp.cpp:281-385
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Close range: maintain closure
    if (rg.range <= 1800.0 ||
        (rg.range <= 2500.0 && rg.ata <= 45.0 * DTR && rg.ataFrom >= 90.0 * DTR)) {
        MaintainClosure(digi, self, target, as, fcs, fcsState, dt);
        return;
    }

    // Target maneuvering detection: FreeFalcon uses PitchDelta/YawDelta.
    // We don't have those, so assume non-maneuvering (simpler path).
    // This means we use the "target non-maneuvering" branch:
    //   if ataFrom < 90°: hold corner speed (no AB)
    //   else: maintain closure
    if (rg.ataFrom < 90.0 * DTR) {
        ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, false,
                                      digi, as, 200.0, 800.0, dt, 100.0);
    } else {
        MaintainClosure(digi, self, target, as, fcs, fcsState, dt);
    }
}

// ===========================================================================
// RollAndPull — main offensive BFM routine
// ===========================================================================
void RollAndPull(DigiState& digi, const DigiEntity& self,
                 const DigiEntity& target, const AircraftState& as,
                 const FlightControlSystem& fcs, FcsState& fcsState,
                 double dt) {
    // Port of randp.cpp:23-279

    // Ground avoidance check — if needed, don't do BFM
    if (digi.groundAvoid.groundAvoidNeeded) {
        return;
    }

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // --- Slow-flying competition (rolling scissors, etc.) ---
    // Range <= 500 ft, both near same heading, ata 55-125°, ataFrom 55-125°
    // Use wrapped heading delta so we don't miss the branch when both
    // aircraft are near yaw = ±π.
    if (rg.range <= kSlowFlyRange &&
        std::fabs(headingError(target.yaw, self.yaw)) < 30.0 * DTR &&
        rg.ata >= kSlowFlyAta * DTR && rg.ata <= kSlowFlyAtaHigh * DTR &&
        rg.ataFrom >= kSlowFlyAta * DTR && rg.ataFrom <= kSlowFlyAtaHigh * DTR) {
        // Slow flying competition — track and brake
        // Use AutoTrack (lift-vector-on-target + pull) instead of TrackPoint
        // (velocity-vector-to-heading + altitude-hold). FreeFalcon's randp.cpp
        // calls AutoTrack(maxGs) after SetTrackPoint in every branch.
        digi.nav.trackX = target.x;
        digi.nav.trackY = target.y;
        digi.nav.trackZ = target.z;  // NED (negative up), no negation
        ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
        ManeuverPrimitives::MachHold(0.36 * digi.config.cornerSpeed, as.vcas, false,
                                      digi, as, 200.0, 800.0, dt, 100.0);
        return;
    }

    // --- OFFENSIVE (we have the advantage) ---
    // ata <= ataFrom OR ata <= 90°
    if (rg.ata <= rg.ataFrom || rg.ata <= kOffensiveAtaThreshold * DTR) {
        if (rg.ataFrom <= kHeadOnAtaFrom * DTR) {
            // Head-on (me -> <- him)
            digi.nav.trackX = target.x;
            digi.nav.trackY = target.y;
            digi.nav.trackZ = target.z;  // NED, no negation
            ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);

            if (rg.range > kFarRange) {
                // > 15 NM: hold corner speed
                ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, true,
                                              digi, as, 200.0, 800.0, dt, 100.0);
            } else if (rg.range > kMergeRange) {
                // 6-15 NM: accelerate if facing each other, else corner speed
                if (rg.ata > 15.0 * DTR || rg.ataFrom > 15.0 * DTR) {
                    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, true,
                                                  digi, as, 200.0, 800.0, dt, 100.0);
                } else {
                    ManeuverPrimitives::MachHold(2.0 * digi.config.cornerSpeed, as.vcas, true,
                                                  digi, as, 200.0, 800.0, dt, 100.0);
                }
            } else if (rg.range >= kCloseRange) {
                // 1.5-6 NM: nose up, hold ~corner speed
                ManeuverPrimitives::MachHold(1.05 * digi.config.cornerSpeed, as.vcas, true,
                                              digi, as, 200.0, 800.0, dt, 100.0);
            } else {
                // < 1.5 NM: energy management
                EnergyManagement(digi, self, target, as, fcs, fcsState, dt);
            }
        } else {
            // Chase (me -> him ->)
            digi.nav.trackX = target.x;
            digi.nav.trackY = target.y;
            digi.nav.trackZ = target.z;  // NED, no negation
            ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
            EnergyManagement(digi, self, target, as, fcs, fcsState, dt);
        }
    }
    // --- NEUTRAL (beam geometry, neither has advantage) ---
    else if (rg.ataFrom >= kNeutralAtaFrom * DTR) {
        digi.nav.trackX = target.x;
        digi.nav.trackY = target.y;
        digi.nav.trackZ = target.z;  // NED, no negation
        ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
        EnergyManagement(digi, self, target, as, fcs, fcsState, dt);
    }
    // --- DEFENSIVE (bandit is behind us) ---
    else {
        // Overshoot check
        const double selfAlt = -self.z;
        const double targetAlt = -target.z;
        const double closureKts = -rg.rangedot * FTPSEC_TO_KNOTS;

        if (selfAlt > kOvershootAlt &&
            rg.ata >= 150.0 * DTR &&
            rg.range <= kOvershootRange &&
            closureKts > kOvershootClosure) {
            // Bandit overshooting — brake and turn
            digi.nav.trackX = target.x;
            digi.nav.trackY = target.y;
            digi.nav.trackZ = target.z;  // NED, no negation
            ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
            ManeuverPrimitives::MachHold(0.36 * digi.config.cornerSpeed, as.vcas, false,
                                          digi, as, 200.0, 800.0, dt, 100.0);
        }
        // Not immediately threatened
        else if (targetAlt > 5000.0 &&
                 rg.range > 1000.0 &&
                 rg.ataFrom >= 15.0 * DTR &&
                 as.vcas <= 0.9 * digi.config.cornerSpeed &&
                 self.pitch < -5.0 * DTR) {
            // Unload and accelerate
            digi.nav.trackX = target.x;
            digi.nav.trackY = target.y;
            digi.nav.trackZ = target.z;  // NED, no negation
            ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
            if (as.aero.alpha_deg > 2.0) {
                ManeuverPrimitives::SetPstick(-1.0, digi.config.maxGs,
                                               CommandType::GCommand, digi, as);
            }
            ManeuverPrimitives::MachHold(1.05 * digi.config.cornerSpeed, as.vcas, true,
                                          digi, as, 200.0, 800.0, dt, 100.0);
        }
        // Immediately threatened
        else {
            digi.nav.trackX = target.x;
            digi.nav.trackY = target.y;
            digi.nav.trackZ = target.z;  // NED, no negation
            ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);
            EnergyManagement(digi, self, target, as, fcs, fcsState, dt);
        }
    }
}

} // namespace digi
} // namespace f4flight
