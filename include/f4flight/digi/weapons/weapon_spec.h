// f4flight - digi/weapons/weapon_spec.h
//
// WeaponSpec — static weapon envelope data for AI decisions.
//
// This is the AI-facing weapon model. It provides the firing envelope
// (RMax, RMin, seeker limits, muzzle velocity) that FireControl reads to
// decide when to fire. It is NOT a fly-out physics model — the AI doesn't
// need to simulate the missile's flight, only know whether the target is
// in the envelope.
//
// Port concept from FreeFalcon:
//   - MissileInputData (missile.h:165-206) — aerodynamic + guidance constants
//   - MissileAuxData (missile.h:208-273) — AI-runtime fields (WEZmax/min, etc.)
//   - GunClass (guns.h:38-178) — gun parameters
//
// We hardcode the envelopes for the ~6 weapon types that matter. Real
// aircraft .dat files can override these later via JSON config.

#pragma once

#include "f4flight/digi/weapons/weapon_types.h"
#include "f4flight/core/constants.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// WeaponSpec — static weapon envelope (AI-facing)
// ===========================================================================
struct WeaponSpec {
    WeaponType type       {WeaponType::None};
    WeaponClass wclass    {WeaponClass::NoWpn};
    WeaponDomain domain   {WeaponDomain::NoDomain};
    SeekerType seeker     {SeekerType::None};

    // --- Missile envelope (NM) ---
    double wezMaxNm       {0.0};   // max launch range (RMax)
    double wezMinNm       {0.0};   // min launch range (RMin)
    double seekerGimbalRad{0.0};   // seeker off-boresight limit
    double maxG           {0.0};   // missile max G (for evasion assessment)

    // --- Gun parameters ---
    double muzzleVelFtps  {0.0};   // bullet muzzle velocity (ft/s)
    double roundsPerSec   {0.0};   // rate of fire
    int    roundsRemaining{0};     // rounds left (mutable at runtime)
    double maxRangeFt     {0.0};   // effective range (ft)

    // --- Bomb parameters ---
    double bombMaxRangeNm {0.0};   // max release range (for toss/level)

    // --- Convenience ---
    bool isGun() const {
        return wclass == WeaponClass::GunWpn;
    }
    bool isMissile() const {
        return wclass == WeaponClass::AimWpn ||
               wclass == WeaponClass::AgmWpn ||
               wclass == WeaponClass::HARMWpn;
    }
    bool isBomb() const {
        return wclass == WeaponClass::BombWpn ||
               wclass == WeaponClass::GbuWpn;
    }
};

// ===========================================================================
// Hardcoded weapon specs — approximate but realistic values.
//
// Sources: public military handbooks, FreeFalcon .dat files, and
// the constants in FreeFalcon's gengage.cpp / mengage.cpp.
// These are good enough for AI decision-making. A real sim would load
// per-weapon .dat files, but for the digi AI port hardcoded is fine.
// ===========================================================================

// Internal gun (M61 Vulcan or equivalent).
// FreeFalcon gengage.cpp uses initBulletVelocity = 3380 ft/s (M61),
// maxAAWpnRange = 6000 ft (mengage.cpp:318), fire threshold
// range < 2.0 * initBulletVelocity (gengage.cpp:332).
inline WeaponSpec gunSpec(int rounds = 510) {
    WeaponSpec s;
    s.type            = WeaponType::Guns;
    s.wclass          = WeaponClass::GunWpn;
    s.domain          = WeaponDomain::Air;
    s.seeker          = SeekerType::None;
    s.muzzleVelFtps   = 3380.0;     // M61 Vulcan muzzle velocity
    s.roundsPerSec    = 100.0;      // ~6000 rpm
    s.roundsRemaining = rounds;     // M61 carries ~510 rounds
    s.maxRangeFt      = 6000.0;     // effective A/A gun range
    return s;
}

// AIM-9 Sidewinder (IR, WVR).
// FreeFalcon: gimbal ~±28°, WEZmax ~8 NM, WEZmin ~0.5 NM.
inline WeaponSpec aim9Spec() {
    WeaponSpec s;
    s.type            = WeaponType::Aim9;
    s.wclass          = WeaponClass::AimWpn;
    s.domain          = WeaponDomain::Air;
    s.seeker          = SeekerType::IR;
    s.wezMaxNm        = 8.0;
    s.wezMinNm        = 0.5;
    s.seekerGimbalRad = 28.0 * DTR;
    s.maxG            = 35.0;       // AIM-9M can pull ~35G
    return s;
}

// AIM-120 AMRAAM (ARH, BVR).
// FreeFalcon: gimbal ~±35° (ARH seeker), WEZmax ~35 NM, WEZmin ~1 NM.
inline WeaponSpec aim120Spec() {
    WeaponSpec s;
    s.type            = WeaponType::Aim120;
    s.wclass          = WeaponClass::AimWpn;
    s.domain          = WeaponDomain::Air;
    s.seeker          = SeekerType::ARH;
    s.wezMaxNm        = 35.0;
    s.wezMinNm        = 1.0;
    s.seekerGimbalRad = 35.0 * DTR;
    s.maxG            = 30.0;       // AMRAAM can pull ~30G in terminal
    return s;
}

// AGM-65 Maverick (laser/TV, A/G).
inline WeaponSpec agm65Spec() {
    WeaponSpec s;
    s.type            = WeaponType::Agm65;
    s.wclass          = WeaponClass::AgmWpn;
    s.domain          = WeaponDomain::Ground;
    s.seeker          = SeekerType::Laser;
    s.wezMaxNm        = 12.0;       // ~12 NM max range
    s.wezMinNm        = 0.5;
    s.seekerGimbalRad = 15.0 * DTR;
    s.maxG            = 5.0;
    return s;
}

// AGM-88 HARM (anti-radiation, SEAD).
inline WeaponSpec agm88Spec() {
    WeaponSpec s;
    s.type            = WeaponType::Agm88;
    s.wclass          = WeaponClass::HARMWpn;
    s.domain          = WeaponDomain::Ground;
    s.seeker          = SeekerType::AntiRadiation;
    s.wezMaxNm        = 80.0;       // ~80 NM max range
    s.wezMinNm        = 5.0;
    s.seekerGimbalRad = 30.0 * DTR;
    s.maxG            = 10.0;
    return s;
}

// Mk82 dumb bomb (500 lb).
inline WeaponSpec mk82Spec() {
    WeaponSpec s;
    s.type            = WeaponType::Mk82;
    s.wclass          = WeaponClass::BombWpn;
    s.domain          = WeaponDomain::Ground;
    s.seeker          = SeekerType::None;
    s.bombMaxRangeNm  = 5.0;        // ~5 NM toss/level release range
    return s;
}

// GBU-12 laser-guided bomb (500 lb LGB).
inline WeaponSpec gbu12Spec() {
    WeaponSpec s;
    s.type            = WeaponType::GBU;
    s.wclass          = WeaponClass::GbuWpn;
    s.domain          = WeaponDomain::Ground;
    s.seeker          = SeekerType::Laser;
    s.bombMaxRangeNm  = 8.0;        // ~8 NM guided release range
    return s;
}

// Lookup table: get a spec by weapon type.
inline WeaponSpec weaponSpecOf(WeaponType t) {
    switch (t) {
        case WeaponType::Guns:
        case WeaponType::Fixed:   return gunSpec();
        case WeaponType::Aim9:    return aim9Spec();
        case WeaponType::Aim120:  return aim120Spec();
        case WeaponType::Agm65:   return agm65Spec();
        case WeaponType::Agm88:   return agm88Spec();
        case WeaponType::Mk82:    return mk82Spec();
        case WeaponType::GBU:     return gbu12Spec();
        default:                  return WeaponSpec{};
    }
}

} // namespace digi
} // namespace f4flight
