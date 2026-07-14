// f4flight - digi/weapons/weapon_types.h
//
// Weapon type enums — port of FreeFalcon hardpnt.h:11-13.
//
// FreeFalcon has three enums: WeaponType (specific weapon), WeaponClass
// (category), WeaponDomain (air/ground). We port all three with the same
// values so data files and AI logic stay compatible.
//
// Source: FreeFalcon src/sim/include/hardpnt.h

#pragma once

#include <cstdint>

namespace f4flight {
namespace digi {

// WeaponType — specific weapon identifier.
// Port of FreeFalcon hardpnt.h:11 (WeaponType enum).
enum class WeaponType : int {
    Guns    = 0,    // internal gun (M61, GSh-301, etc.)
    Aim9    = 1,    // AIM-9 Sidewinder (IR, WVR)
    Aim120  = 2,    // AIM-120 AMRAAM (ARH, BVR)
    Agm88   = 3,    // AGM-88 HARM (anti-radiation)
    Agm65   = 4,    // AGM-65 Maverick (AG missile)
    Mk82    = 5,    // Mk82 dumb bomb
    Mk84    = 6,    // Mk84 dumb bomb
    GBU     = 7,    // GBU-10/12/16 laser-guided bomb
    SAM     = 8,    // surface-to-air missile (threat, not carried)
    LAU     = 9,    // LAU-3 rocket pod
    Fixed   = 10,   // fixed gun (internal)
    None    = 11,
    GPS     = 12,   // GPS-guided weapon (JDAM, JSOW)
};

// WeaponClass — weapon category (used by SMS + doctrine).
// Port of FreeFalcon hardpnt.h:12 (WeaponClass enum).
enum class WeaponClass : int {
    AimWpn   = 0,   // A/A missile (Aim9, Aim120)
    RocketWpn = 1,  // unguided rocket
    BombWpn  = 2,   // dumb bomb
    GunWpn   = 3,   // gun
    ECM      = 4,   // ECM pod
    Tank     = 5,   // external fuel tank
    AgmWpn   = 6,   // A/G missile (Agm65)
    HARMWpn  = 7,   // HARM
    SamWpn   = 8,   // SAM (threat)
    GbuWpn   = 9,   // laser-guided bomb
    Camera   = 10,  // recon camera
    NoWpn    = 11,
};

// WeaponDomain — air vs ground capability.
// Port of FreeFalcon hardpnt.h:13 (WeaponDomain enum).
enum class WeaponDomain : int {
    NoDomain = 0,
    Air      = 0x1,
    Ground   = 0x2,
    Both     = 0x3,
};

// SeekerType — missile seeker type (extended from DigiEntity::SeekerType).
// Port of FreeFalcon sensor types used by FireControl.
enum class SeekerType : int {
    None         = 0,  // gun, bomb, rocket
    IR           = 1,  // infrared (Aim9)
    SARH         = 2,  // semi-active radar homing (Aim7)
    ARH          = 3,  // active radar homing (Aim120)
    AntiRadiation = 4, // HARM
    Laser        = 5,  // GBU, Maverick
    TV           = 6,  // TV-guided Maverick
};

// --- Helpers ---

inline WeaponClass weaponClassOf(WeaponType t) {
    switch (t) {
        case WeaponType::Aim9:
        case WeaponType::Aim120:   return WeaponClass::AimWpn;
        case WeaponType::Agm65:    return WeaponClass::AgmWpn;
        case WeaponType::Agm88:    return WeaponClass::HARMWpn;
        case WeaponType::Mk82:
        case WeaponType::Mk84:     return WeaponClass::BombWpn;
        case WeaponType::GBU:      return WeaponClass::GbuWpn;
        case WeaponType::LAU:      return WeaponClass::RocketWpn;
        case WeaponType::Guns:
        case WeaponType::Fixed:    return WeaponClass::GunWpn;
        case WeaponType::SAM:      return WeaponClass::SamWpn;
        default:                   return WeaponClass::NoWpn;
    }
}

inline WeaponDomain weaponDomainOf(WeaponType t) {
    switch (t) {
        case WeaponType::Aim9:
        case WeaponType::Aim120:   return WeaponDomain::Air;
        case WeaponType::Guns:
        case WeaponType::Fixed:    return WeaponDomain::Air;
        case WeaponType::Agm65:
        case WeaponType::Agm88:
        case WeaponType::Mk82:
        case WeaponType::Mk84:
        case WeaponType::GBU:
        case WeaponType::LAU:
        case WeaponType::GPS:      return WeaponDomain::Ground;
        case WeaponType::SAM:      return WeaponDomain::Air; // targets air
        default:                   return WeaponDomain::NoDomain;
    }
}

inline SeekerType seekerTypeOf(WeaponType t) {
    switch (t) {
        case WeaponType::Aim9:     return SeekerType::IR;
        case WeaponType::Aim120:   return SeekerType::ARH;
        case WeaponType::Agm88:    return SeekerType::AntiRadiation;
        case WeaponType::Agm65:    return SeekerType::Laser; // or TV
        case WeaponType::GBU:      return SeekerType::Laser;
        case WeaponType::GPS:      return SeekerType::None;  // inertial
        default:                   return SeekerType::None;
    }
}

inline bool isAirToAir(WeaponType t) {
    return weaponDomainOf(t) == WeaponDomain::Air;
}

inline bool isAirToGround(WeaponType t) {
    return weaponDomainOf(t) == WeaponDomain::Ground;
}

} // namespace digi
} // namespace f4flight
