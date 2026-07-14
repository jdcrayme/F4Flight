// f4flight - digi/offensive/merge.h
//
// Merge — first-pass merge geometry maneuver.
//
// Port of FreeFalcon merge.cpp (304 LOC). The AI enters MergeMode when the
// target is within ~1000 ft and ata < 45°. The mode picks a merge maneuver
// (one-circle, two-circle, slice, or vertical) based on the aircraft's
// combat class flags, then commands a max-G pull in the chosen direction.
//
// AccelMode is entered when the AI is too slow during combat (below corner
// speed while pitching up). It rolls 170° and pulls to regain speed.
//
// Source: FreeFalcon src/sim/digi/merge.cpp

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// MergeCheck — mode entry/exit test.
//
// Port of FreeFalcon merge.cpp:9-52.
//
// Entry: alt > 3000 ft, range ≤ breakRange (function of speeds + ataFrom),
//        ata < 45°, |pitch| < 45°, ataFrom < 45°.
// Exit:  mergeTimer expires (3 seconds).
// ===========================================================================
bool MergeCheck(const DigiState& digi, const DigiEntity& self,
                const DigiEntity& target);

// ===========================================================================
// MergeManeuver — execute the merge.
//
// Port of FreeFalcon merge.cpp:54-254.
//
// On first pass, picks a maneuver based on aircraft combat class flags:
//   - CanLevelTurn: one-circle or two-circle turn (90° bank)
//   - CanSlice: slice turn (135° bank)
//   - CanUseVertical: wings-level pull (0° bank, full burner)
//   - Combinations: pick based on airspeed vs corner speed
//
// Then commands max-G pull + roll to the target bank angle.
// ===========================================================================
void MergeManeuver(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt);

// ===========================================================================
// AccelCheck — mode entry test for AccelMode.
//
// Port of FreeFalcon merge.cpp:256-273.
//
// Entry: in combat mode (Merge..BVR), pitch > 50° and vcas < 0.4×corner,
//        or pitch > 0° and vcas < 0.35×corner.
// ===========================================================================
bool AccelCheck(const DigiState& digi, const DigiEntity& self,
                const AircraftState& as);

// ===========================================================================
// AccelManeuver — regain corner speed.
//
// Port of FreeFalcon merge.cpp:275-304.
//
// Rolls 170° (inverted) and pulls 4G to dive and accelerate.
// ===========================================================================
void AccelManeuver(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt);

} // namespace digi
} // namespace f4flight
