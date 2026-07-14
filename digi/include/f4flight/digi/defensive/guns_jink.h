// f4flight - digi/defensive/guns_jink.h
//
// Guns defense — Tier 1 digi capability.
//
// Direct port of FreeFalcon's gunsjink.cpp (331 LOC).
// When an enemy aircraft is within gun range and in firing position, the AI
// performs a defensive jink: roll to a new bank angle, then max-G pull for
// ~2 seconds to break the gun tracking solution.
//
// Entry conditions (gunsjink.cpp:30-134):
//   - Target range > 0 and < 6000 ft
//   - Target range < INIT_GUN_VEL (4500 ft)
//   - Target is firing or hostile
//   - azFrom within ±15° (or predicted within ±5° in z seconds)
//   - elFrom within -10° to +4°
//   - tgt_time <= att_time (target can fire before we can)
//
// Maneuver (gunsjink.cpp:144-331):
//   Phase -1 (init): pick roll angle based on aspect
//     - aspect >= 90°: put plane of wings on attacker (droll ± 90°)
//     - aspect < 90°: roll ±70° (special case for in-plane crossing)
//     - AG jettison stores (if not bomber)
//   Phase 0 (rolling): SetPstick(-2G), roll to newRoll, when |eroll| < 5° → phase 1
//   Phase 1 (pulling): max G pull for ~2 seconds, then exit
//
// Exit: jinkTime = -1, ResetMaxRoll

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// Constants from FreeFalcon gunsjink.cpp
constexpr double kInitGunVel = 4500.0;       // ft/s — initial bullet velocity
constexpr double kGunRangeMax = 6000.0;      // ft — max range to consider guns jink
constexpr double kGunJinkExitRange = 4000.0; // ft — exit jink if range exceeds this
constexpr double kJinkPullDuration = 2.0;    // seconds — max-G pull duration
constexpr double kJinkRollTolerance = 5.0;   // degrees — roll capture tolerance
constexpr double kJinkRollAngle = 70.0;      // degrees — normal jink roll angle
constexpr double kJinkWingsOnAttacker = 90.0; // degrees — aspect >= 90 roll offset

// GunsJinkCheck — determine if guns jink mode should be active.
//
//   digi   : AI state (reads gunsThreat)
//   self   : own aircraft entity
//
// Returns: true if GunsJink mode should be entered.
//
// Logic (simplified from gunsjink.cpp:15-142):
//   - No guns threat → false
//   - Threat is dead → false
//   - Range > 6000 ft or range <= 0 → false
//   - Range >= INIT_GUN_VEL (4500 ft) → false
//   - Threat not firing → false (simplified: FF also checks team stance)
//   - azFrom outside ±15° → false
//   - elFrom outside -10° to +4° → false
//   - Otherwise → true
bool GunsJinkCheck(const DigiState& digi, const DigiEntity& self);

// GunsJink — execute the guns jink maneuver.
//
//   digi   : AI state (reads jinkTime, newRoll; writes jinkTime, newRoll, jinkTimer)
//   self   : own aircraft entity
//   as     : aircraft state
//   fcs    : flight control system
//   fcsState : FCS state (written: maxRoll, maxRollDelta)
//   dt     : frame time (seconds)
//
// Returns: true if the jink is still active (stay in GunsJink mode),
//          false if the jink has completed (exit GunsJink mode).
bool GunsJink(DigiState& digi, const DigiEntity& self,
              const AircraftState& as,
              const FlightControlSystem& fcs, FcsState& fcsState,
              double dt);

} // namespace digi
} // namespace f4flight
