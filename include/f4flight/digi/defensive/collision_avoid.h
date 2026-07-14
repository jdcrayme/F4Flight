// f4flight - digi/defensive/collision_avoid.h
//
// CollisionAvoid — air-air collision avoidance.
//
// Port of FreeFalcon cavoid.cpp (139 LOC). The AI checks for imminent
// collision with the current target by extrapolating both velocity vectors.
// If a collision is predicted within the reaction time, the AI sets a
// trackpoint 45° out of plane and enters CollisionAvoid mode.
//
// Reaction time: (GS_LIMIT / maxGs) * 0.55 seconds.
// Collision threshold: 200 ft horizontal range.
// Evasion: 45° elevation, 45° azimuth (out of plane) trackpoint at 10000 ft.
//
// Source: FreeFalcon src/sim/digi/cavoid.cpp

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// CollisionCheck — test for imminent collision with the target.
//
// Port of FreeFalcon cavoid.cpp:12-134.
//
// Returns true if a collision is predicted within the reaction time.
// Sets digi.trackX/Y/Z to the evasion trackpoint if collision is predicted.
// ===========================================================================
bool CollisionCheck(DigiState& digi, const DigiEntity& self,
                    const DigiEntity& target);

// ===========================================================================
// CollisionAvoid — execute the collision avoidance maneuver.
//
// Port of FreeFalcon cavoid.cpp:136-139.
// Tracks the evasion trackpoint via TrackPoint (which calls AutoTrack).
// ===========================================================================
void CollisionAvoid(DigiState& digi, const DigiEntity& self,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState);

} // namespace digi
} // namespace f4flight
