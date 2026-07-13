// f4flight - digi/defensive/missile_defeat.h
//
// Missile defense — Tier 1 digi capability.
//
// Direct port of FreeFalcon's mdefeat.cpp (732 LOC).
// Recognizes an incoming missile and selects one of three defensive maneuvers:
//
//   MissileBeamManeuver — turn perpendicular to missile heading (notch).
//     Used when the missile is close and closure is high. Puts the aircraft
//     beam-on to the missile to break radar lock or defeat the seeker.
//
//   MissileDragManeuver — turn cold (away from missile heading).
//     Used when the missile is far or closure is low. Runs the missile out
//     of energy by forcing it to chase.
//
//   MissileLastDitch — max-G pull, drop chaff/flare.
//     Used when time-to-go < 1.0 second (LD_TIME). Last-resort maneuver.
//
// Maneuver selection (mdefeat.cpp:499-510):
//   if (range > 2 NM or closure < 400 kts)  → MissileDragManeuver
//   else                                    → MissileBeamManeuver
//   if (TTGO < LD_TIME)                     → MissileLastDitch
//
// IR missile throttle cut: for skill > 2, cap throttle at 0.99 (MIL) when
// an IR missile is incoming (mdefeat.cpp:514-516).
//
// Exit condition: missile range increasing for (6 - skill) seconds
// (mdefeat.cpp:113).

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"

namespace f4flight {
namespace digi {

// Constants from FreeFalcon mdefeat.cpp
constexpr double kLDTime = 1.0;           // seconds — last ditch maneuver time
constexpr double kMissileLethalCone = 45.0;  // degrees
constexpr double kAveAim9Vel = 2000.0;    // ft/s — average AIM-9 velocity
constexpr double kMaxThreatTime = 100.0;  // seconds — cap on threat time
constexpr double kBeamTrackDist = 0.5 * 6076.0;  // ft — 0.5 NM beam trackpoint
constexpr double kDragTrackDist = 20.0 * 6076.0; // ft — 20 NM drag trackpoint
constexpr double kDragRangeThreshold = 2.0 * 6076.0;  // ft — 2 NM
constexpr double kDragClosureThreshold = 400.0;  // ft/s (was kts in FF; we use ft/s)

// MissileDefeatCheck — determine if missile defeat mode should be active.
//
//   digi   : AI state (reads incomingMissile, skill; writes missileDefeatTtgo)
//   self   : own aircraft entity (for position)
//   dt     : frame time (seconds)
//
// Returns: true if MissileDefeat mode should be entered/stayed in.
//
// Logic (simplified from mdefeat.cpp:30-358):
//   - No incoming missile → false
//   - Missile is dead → clear and return false
//   - Missile range > previous range (passing) and evade timer expired → clear, false
//   - Otherwise → true
bool MissileDefeatCheck(DigiState& digi, const DigiEntity& self, double dt);

// MissileDefeat — execute the missile defeat maneuver.
//
// Selects Beam / Drag / LastDitch based on range, closure, and time-to-go.
// Commands stick/throttle via ManeuverPrimitives.
//
//   digi   : AI state
//   self   : own aircraft entity
//   as     : aircraft state (for FCS reads)
//   fcs    : flight control system
//   fcsState : FCS state (written: maxRoll, maxRollDelta)
//   dt     : frame time (seconds)
void MissileDefeat(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt);

// MissileBeamManeuver — turn perpendicular to missile heading.
// Sets a trackpoint 0.5 NM to the beam of the missile's heading and
// AutoTracks to it. Returns true when within 5 ft of trackpoint.
//
// Port of mdefeat.cpp:526-592.
bool MissileBeamManeuver(DigiState& digi, const DigiEntity& self,
                          const AircraftState& as,
                          const FlightControlSystem& fcs, FcsState& fcsState,
                          double dt);

// MissileDragManeuver — turn cold (away from missile).
// Sets a trackpoint 20 NM in the missile's heading direction (cold = same
// direction the missile is going, so it has to chase) and AutoTracks to it.
//
// Port of mdefeat.cpp:594-635.
void MissileDragManeuver(DigiState& digi, const DigiEntity& self,
                          const AircraftState& as,
                          const FlightControlSystem& fcs, FcsState& fcsState,
                          double dt);

// MissileLastDitch — max-G pull + chaff/flare.
// Commands max G, holds corner speed, drops chaff/flare in the 0.25-0.8s
// TTGO window.
//
// Port of mdefeat.cpp:637-725.
void MissileLastDitch(DigiState& digi, const DigiEntity& self,
                       const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt);

} // namespace digi
} // namespace f4flight
