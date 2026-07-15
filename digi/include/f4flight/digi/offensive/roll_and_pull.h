// f4flight - digi/offensive/roll_and_pull.h
//
// RollAndPull — the universal offensive BFM (Basic Fighter Maneuver) routine.
//
// Direct port of FreeFalcon's randp.cpp (612 LOC).
//
// RollAndPull is the core offensive dogfight maneuver: "put the lift vector
// on the bandit and pull." It selects behavior based on the relative geometry:
//
//   OFFENSIVE (we're behind the bandit, ata <= ataFrom or ata <= 90°):
//     - Head-on (ataFrom <= 55°): fly at the bandit, manage energy for the
//       merge. Outside 6 NM: accelerate. Inside 1.5 NM: energy management.
//     - Chase (ataFrom > 55°): track the bandit, energy management.
//
//   NEUTRAL (beam geometry, ataFrom >= 45°):
//     - Track the bandit, energy management.
//
//   DEFENSIVE (bandit is behind us, ataFrom < 45°):
//     - Overshoot check: if bandit overshooting, brake and turn.
//     - Not immediately threatened: unload and accelerate.
//     - Immediately threatened: track and energy manage.
//
// EnergyManagement controls speed based on closure, range, and whether the
// target is maneuvering. MaintainClosure is a closure-rate-based speed
// controller. CollisionTime estimates time to intercept.
//
// Port of: randp.cpp:23 (RollAndPull), randp.cpp:281 (EnergyManagement),
// randp.cpp:553 (MaintainClosure), randp.cpp:603 (CollisionTime).

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// Constants from FreeFalcon randp.cpp
constexpr double kControlPointDistance = 1600.0;    // ft
constexpr double kControlPointElevation = 25.0;     // ft
constexpr double kMagicNumber = 0.5;                // lead time (seconds)
constexpr double kAltRateDeadband = 1000.0;         // fpm
constexpr double kVerticalMagic = 0.025;            // rad — yaw-delta threshold for vertical fight
constexpr double kSlowFlyRange = 500.0;             // ft — slow-flying competition range
constexpr double kSlowFlyAta = 55.0;                // deg — slow-fly ata threshold
constexpr double kSlowFlyAtaHigh = 125.0;           // deg
constexpr double kOffensiveAtaThreshold = 90.0;     // deg
constexpr double kHeadOnAtaFrom = 55.0;             // deg
constexpr double kNeutralAtaFrom = 45.0;            // deg
constexpr double kMergeRange = 6.0 * 6076.0;        // ft — 6 NM
constexpr double kCloseRange = 1.5 * 6076.0;        // ft — 1.5 NM
constexpr double kFarRange = 15.0 * 6076.0;         // ft — 15 NM
constexpr double kOvershootAlt = 3000.0;            // ft
constexpr double kOvershootRange = 2000.0;          // ft
constexpr double kOvershootClosure = 70.0;          // kts

// RollAndPull — the main offensive BFM routine.
//
//   digi    : AI state
//   self    : own aircraft entity
//   target  : the bandit we're fighting
//   as      : aircraft state (for FCS reads)
//   fcs     : flight control system
//   fcsState: FCS state (written)
//   dt      : frame time (seconds)
//
// Updates: digi.commands.pStick, digi.commands.rStick, digi.commands.yPedal, digi.commands.throttle
// Calls: ManeuverPrimitives::TrackPoint, AutoTrack, MachHold, SetPstick
void RollAndPull(DigiState& digi, const DigiEntity& self,
                 const DigiEntity& target, const AircraftState& as,
                 const FlightControlSystem& fcs, FcsState& fcsState,
                 double dt);

// EnergyManagement — speed control based on fight geometry.
//
// Decides whether to maintain closure, hold corner speed, or accelerate
// based on range, ata, ataFrom, and whether the target is maneuvering.
//
// Port of randp.cpp:281.
void EnergyManagement(DigiState& digi, const DigiEntity& self,
                       const DigiEntity& target, const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt);

// MaintainClosure — closure-rate-based speed control.
//
// Computes a desired closure based on range and current closure rate,
// then sets MachHold to achieve it. Used when close to the target.
//
// Port of randp.cpp:553.
void MaintainClosure(DigiState& digi, const DigiEntity& self,
                      const DigiEntity& target, const AircraftState& as,
                      const FlightControlSystem& fcs, FcsState& fcsState,
                      double dt);

// CollisionTime — estimate time to collision with the target.
//
// Returns min(range / self_vt, 0.5) seconds.
// Port of randp.cpp:603.
double CollisionTime(const DigiEntity& self, const DigiEntity& target);

} // namespace digi
} // namespace f4flight
