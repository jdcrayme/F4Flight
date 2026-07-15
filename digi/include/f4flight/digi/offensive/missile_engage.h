// f4flight - digi/offensive/missile_engage.h
//
// MissileEngage — offensive missile firing mode.
//
// Port of FreeFalcon mengage.cpp (489 LOC). The AI enters MissileEngage
// when the target is within maxAAWpnRange and ata < 60°. The mode:
//   1. Selects the best missile via WeaponSelection
//   2. Tracks the target with lead (TrackPoint → AutoTrack)
//   3. Fires via FireControl when the target is in the firing envelope
//
// Two range regimes:
//   - Beyond RAP distance (Roll-and-Pull distance, ~8 NM): BVR tracking
//     with lead pursuit + closure control. Fires when in WEZ.
//   - Within RAP distance: defer to RollAndPull (WVR BFM) for the merge.
//
// Source: FreeFalcon src/sim/digi/mengage.cpp

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// MissileEngageCheck — mode entry/exit test.
//
// Port of FreeFalcon mengage.cpp:24-99.
//
// Entry: range ≤ maxAAWpnRange × 1.05, ata < 60° × 1.05, range ≥ 3000 ft
//        (or no gun), have a missile.
// Exit:  range > maxAAWpnRange × 1.09 OR ata > 60° × 1.09 OR no missile.
//        If range < 3000 ft and have gun, transition to GunsEngage.
// ===========================================================================
bool MissileEngageCheck(const DigiState& digi, const DigiEntity& self,
                        const DigiEntity& target,
                        const StoresManagementSystem& sms,
                        bool hasGun);

// ===========================================================================
// WeaponSelection — pick the best missile for the current target.
//
// Port of FreeFalcon mengage.cpp:293-488.
//
// Iterates hardpoints, picks the missile with the best range margin
// (rmax*0.8 - range). IR vs radar preference: prefer radar/ARH for BVR,
// IR for WVR or helicopters. Returns WeaponType::None if no missile
// suitable.
// ===========================================================================
WeaponType WeaponSelection(const StoresManagementSystem& sms,
                           const DigiEntity& self,
                           const DigiEntity& target,
                           double& outMaxAAWpnRangeFt);

// ===========================================================================
// FireControl — firing decision for the current missile.
//
// Port of FreeFalcon dlogic.cpp:792-933.
//
// Checks firing envelope (range in WEZ, ata within seeker limit), then
// applies shoot-shoot / shoot-look doctrine. Sets digi.weapon.mslFireFlag and
// digi.weapon.fireStation when firing.
// ===========================================================================
void FireControlMissile(DigiState& digi, const DigiEntity& self,
                        const DigiEntity& target,
                        const WeaponSpec& missile,
                        const StoresManagementSystem& sms,
                        double dt);

// ===========================================================================
// MissileEngage — main mode dispatcher.
//
// Port of FreeFalcon mengage.cpp:101-291.
//
// Beyond RAP distance: BVR tracking with lead pursuit + closure control.
// Within RAP distance: defer to RollAndPull (WVR BFM).
// ===========================================================================
void MissileEngage(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const StoresManagementSystem& sms,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt);

} // namespace digi
} // namespace f4flight
