// f4flight - digi/offensive/guns_engage.h
//
// GunsEngage — offensive gun tracking + firing mode.
//
// Port of FreeFalcon gengage.cpp (580 LOC). The AI enters GunsEngage when
// the target is within 3500 ft and ata < 35°. The mode has two phases:
//
//   Coarse track (CoarseGunsTrack) — lead-angle tracking via GunsAutoTrack.
//     Projects the target ahead by bullet TOF, sets the trackpoint to the
//     lead-aim point, and calls GunsAutoTrack to roll/pull toward it.
//
//   Fine track (FineGunsTrack) — pipper tracking + fire decision.
//     Computes the pipper position (where the bullets will be after TOF),
//     compares to the target's az/el, and fires when the pipper is on
//     target (|azerr| < 1.5°, elerr in [-1.5°, 0.5°], atadot < 50°/s,
//     range < 2× muzzleVel).
//
// The fire decision sets digi.gunFireFlag, which compute() maps to
// PilotInput.fireGun.
//
// Source: FreeFalcon src/sim/digi/gengage.cpp

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/fcs.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// GunsEngageCheck — mode entry/exit test.
//
// Port of FreeFalcon gengage.cpp:22-70.
//
// Entry: range ≤ 3500 ft, ata < 35° (A2A) or 15° (non-A2A), gun has rounds.
// Exit:  range > 3500 ft OR ata > 1.25 × angleLimit OR gun empty.
//
// Returns true if the mode should be entered/stayed.
// ===========================================================================
bool GunsEngageCheck(const DigiState& digi, const DigiEntity& self,
                     const DigiEntity& target, const WeaponSpec& gun,
                     bool isAirToAirMission);

// ===========================================================================
// CoarseGunsTrack — lead-angle tracking via GunsAutoTrack.
//
// Port of FreeFalcon gengage.cpp:198-222.
//
// Projects the target ahead by `leadTof` bullet times of flight, sets
// trackX/Y/Z to the lead-aim point (including gravity drop), and calls
// GunsAutoTrack. Returns the resulting ATA (rad).
// ===========================================================================
double CoarseGunsTrack(DigiState& digi, const DigiEntity& self,
                       const DigiEntity& target, const AircraftState& as,
                       const WeaponSpec& gun, FcsState& fcsState,
                       double leadTof);

// ===========================================================================
// FineGunsTrack — pipper tracking + fire decision.
//
// Port of FreeFalcon gengage.cpp:224-360.
//
// Computes the pipper position (where bullets will be after TOF), compares
// to target az/el. Two phases:
//   Phase A (not waitingForShot): CoarseGunsTrack until pipper near target,
//     then set waitingForShot = true.
//   Phase B (waitingForShot): relax G, fire when pipper on target, then
//     reset.
//
// Sets digi.gunFireFlag when firing. Outputs lagAngle for the caller.
// ===========================================================================
void FineGunsTrack(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const WeaponSpec& gun, const FlightControlSystem& fcs,
                   FcsState& fcsState, double speed, double dt,
                   double& lagAngle);

// ===========================================================================
// GunsEngage — main mode dispatcher.
//
// Port of FreeFalcon gengage.cpp:72-195.
//
// If target is ahead of 3/9 line (ataFrom < 90°): FineGunsTrack directly.
// If behind 3/9: closure-control BFM with possible RollAndPull at close
// range or OverBMode if lagging.
// ===========================================================================
void GunsEngage(DigiState& digi, const DigiEntity& self,
                const DigiEntity& target, const AircraftState& as,
                const WeaponSpec& gun, const FlightControlSystem& fcs,
                FcsState& fcsState, double dt);

} // namespace digi
} // namespace f4flight
