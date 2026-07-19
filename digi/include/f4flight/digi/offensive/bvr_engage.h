// f4flight - digi/offensive/bvr_engage.h
//
// BvrEngage — beyond-visual-range engagement.
//
// Port of FreeFalcon bvrengage.cpp (3,238 LOC). The AI enters BvrEngage
// when the target is within maxAAWpnRange × 1.3 (or 45 NM, whichever is
// larger) and beyond the MissileEngage RAP distance (~8 NM).
//
// The mode:
//   1. ChoiceProfile — computes a threat score and selects one of 6
//      tactical profiles (Defensive, Level3c, Level2c, Level3b, Grinder, Wall)
//   2. BvrChooseTactic — maps the profile to a specific tactic
//      (Crank, Beam, Drag, BaseLineIntercept, Pursuit)
//   3. Executes the tactic via StickandThrottle + AutoTrack
//
// This is a BASIC port — FF has 18 profiles and ~30 tactics. We implement
// the 6 primary profiles and 4 tactics (Crank, Beam, Drag, Pursuit) which
// cover the core BVR engagement flow.
//
// Source: FreeFalcon src/sim/digi/bvrengage.cpp

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/flight/core/units.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// BVR profile type (simplified from FF's 18 profiles to 6 primary)
// ===========================================================================
enum class BvrProfile : int {
    None      = 0,
    Wall      = 1,   // offensive, superior — close range, shoot
    Grinder   = 2,   // offensive, slight advantage — press in
    Level3b   = 3,   // neutral — crank and shoot
    Level2c   = 4,   // defensive — beam and drag
    Level3c   = 5,   // more defensive — beam and drag
    Defensive = 6,   // fully defensive — drag and run
};

// ===========================================================================
// BVR tactic type
// ===========================================================================
enum class BvrTactic : int {
    None       = 0,
    Pursuit    = 1,   // head toward target, shoot
    Crank      = 2,   // turn 45° off the target (maintain radar track)
    Beam       = 3,   // turn 90° to the target (break radar lock)
    Drag       = 4,   // turn cold (180° from target), run
};

// ===========================================================================
// BvrEngageCheck — mode entry/exit test.
//
// Port of FreeFalcon bvrengage.cpp:46-216.
//
// Entry: target within engageRange (max(maxAAWpnRange×1.3, 45 NM)), beyond
//        8 NM (RAP distance — inside that, MissileEngage handles it).
// Exit:  target beyond engageRange × 1.1, or no target.
// ===========================================================================
bool BvrEngageCheck(const DigiState& digi, const DigiEntity& self,
                    const DigiEntity& target, double maxAAWpnRangeFt);

// ===========================================================================
// ChoiceProfile — compute threat score and select BVR profile.
//
// Port of FreeFalcon bvrengage.cpp:679-774.
//
// Threat score factors:
//   +20: inferior missile (range < 10 NM)
//   +60: outranged (combat class 7, target beyond missile range)
//   +30: outnumbered
//   +5:  target higher
//   +5:  target faster
//   +20: target on our tail (ataFrom < 90°, ata > 90°)
//   +10: target beyond missile range
//    5:  we're on target's tail (force offensive)
//
// Profile selection:
//   >= 60: Defensive
//   50-59: Level3c
//   30-49: Level2c
//   20-29: Level3b
//   10-19: Grinder
//   < 10:  Wall
// ===========================================================================
BvrProfile ChoiceProfile(const DigiEntity& self, const DigiEntity& target,
                         double maxAAWpnRangeFt);

// ===========================================================================
// BvrChooseTactic — map profile + geometry to a specific tactic.
//
// Port of FreeFalcon bvrengage.cpp:BvrChooseTactic (simplified).
//
// Wall → Pursuit (close and shoot)
// Grinder → Pursuit (press in)
// Level3b → Crank (shoot and crank)
// Level2c → Beam (defensive beam)
// Level3c → Beam (defensive beam)
// Defensive → Drag (turn cold)
// ===========================================================================
BvrTactic BvrChooseTactic(BvrProfile profile, const DigiEntity& self,
                          const DigiEntity& target);

// ===========================================================================
// StickandThrottle — core BVR steering.
//
// Port of FreeFalcon bvrengage.cpp:3014-3073.
//
// Calls AutoTrack to track the target, then MachHold to maintain speed.
// Adjusts trackZ for altitude management.
// ===========================================================================
void StickandThrottle(DigiState& digi, const DigiEntity& self,
                      const AircraftState& as,
                      const FlightControlSystem& fcs, FcsState& fcsState,
                      CasKnots desiredSpeed, double desiredAltFt,
                      double dt);

// ===========================================================================
// Crank — turn 45° off the target heading.
//
// Port of FreeFalcon bvrengage.cpp:2901-2983.
// ===========================================================================
void CrankManeuver(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt, int direction = 0);

// ===========================================================================
// Beam — turn 90° to the target heading.
//
// Port of FreeFalcon bvrengage.cpp:2801-2900.
// ===========================================================================
void BeamManeuver(DigiState& digi, const DigiEntity& self,
                  const DigiEntity& target, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt, int direction = 0);

// ===========================================================================
// Drag — turn cold (away from target).
//
// Port of FreeFalcon bvrengage.cpp:2984-3011.
// ===========================================================================
void DragManeuver(DigiState& digi, const DigiEntity& self,
                  const DigiEntity& target, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt);

// ===========================================================================
// BvrEngage — main mode dispatcher.
//
// Port of FreeFalcon bvrengage.cpp:218-440 (simplified).
// ===========================================================================
void BvrEngage(DigiState& digi, const DigiEntity& self,
               const DigiEntity& target, const AircraftState& as,
               const FlightControlSystem& fcs, FcsState& fcsState,
               double dt);

} // namespace digi
} // namespace f4flight
