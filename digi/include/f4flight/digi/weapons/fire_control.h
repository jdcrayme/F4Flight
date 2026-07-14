// f4flight - digi/weapons/fire_control.h
//
// FireControl — firing decision logic for the digi AI.
//
// Port of FreeFalcon dlogic.cpp:792-933 (FireControl) + mengage.cpp:293-488
// (WeaponSelection).
//
// The AI calls FireControl each frame to decide:
//   1. Which weapon to select (WeaponSelection)
//   2. Whether the current target is in the firing envelope (CanFire)
//   3. Whether to fire now (shoot-shoot / shoot-look doctrine)
//
// This is a decision layer — it does NOT simulate the weapon's flight.
// The host reads PilotInput.fireGun / releaseConsent and resolves the hit.

#pragma once

#include "f4flight/digi/weapons/weapon_types.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/core/constants.h"

#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// FiringEnvelope — the geometry the AI needs to evaluate CanFire.
//
// All angles in radians, range in feet.
// ===========================================================================
struct FiringEnvelope {
    double rangeFt     {0.0};   // 3D range to target
    double rangedot    {0.0};   // range rate (ft/s, negative = closing)
    double ata         {0.0};   // angle off self's nose to target (rad)
    double ataFrom     {0.0};   // angle off target's nose to self (rad)
    double altFt       {0.0};   // self altitude AGL (ft)
    double vtFtps      {0.0};   // self true airspeed (ft/s)
    double targetVt    {0.0};   // target true airspeed (ft/s)
};

// ===========================================================================
// FireControl — static firing decision functions.
// ===========================================================================
class FireControl {
public:
    // -----------------------------------------------------------------------
    // CanFire — is the target in the firing envelope for this weapon?
    //
    // Port of FreeFalcon dlogic.cpp:792-933 (FireControl).
    //
    // Per-seeker ata gates (from FF):
    //   - Non-radar/IRST (guns, bombs, rockets): ata < 20°
    //   - SARH/ARH (Aim7, Aim120): ata < 35°
    //   - IRST (Aim9): ata < 0.95 × gimbalLimit
    //
    // Range gate:
    //   - Missiles: wezMin < range < wezMax
    //   - Guns: range < 2.0 × muzzleVel (FF gengage.cpp:332)
    // -----------------------------------------------------------------------
    static bool canFire(const WeaponSpec& weapon,
                        const FiringEnvelope& env) {
        if (weapon.isGun()) {
            return canFireGun(weapon, env);
        }
        if (weapon.isMissile()) {
            return canFireMissile(weapon, env);
        }
        if (weapon.isBomb()) {
            return canFireBomb(weapon, env);
        }
        return false;
    }

    // Gun firing envelope.
    // FF: range < 2.0 * initBulletVelocity, ata < 20° (simplified; the
    // real FineGunsTrack uses pipper geometry — see guns_engage.cpp).
    static bool canFireGun(const WeaponSpec& gun,
                           const FiringEnvelope& env) {
        if (!gun.isGun()) return false;
        if (gun.roundsRemaining <= 0) return false;

        // Range gate: effective gun range
        const double maxRangeFt = std::min(gun.maxRangeFt,
                                            2.0 * gun.muzzleVelFtps);
        if (env.rangeFt > maxRangeFt) return false;
        if (env.rangeFt < 100.0) return false;  // too close

        // ATA gate: target must be near the nose
        if (std::fabs(env.ata) > 20.0 * DTR) return false;

        return true;
    }

    // Missile firing envelope.
    // FF: ata < seekerGimbalRad (or 35° for radar), range in WEZ.
    static bool canFireMissile(const WeaponSpec& missile,
                               const FiringEnvelope& env) {
        if (!missile.isMissile()) return false;

        // Range gate (convert NM to ft)
        const double wezMinFt = missile.wezMinNm * 6076.0;
        const double wezMaxFt = missile.wezMaxNm * 6076.0;
        if (env.rangeFt < wezMinFt) return false;
        if (env.rangeFt > wezMaxFt) return false;

        // ATA gate by seeker type
        double ataLimitRad = 0.0;
        switch (missile.seeker) {
            case SeekerType::IR:
                ataLimitRad = 0.95 * missile.seekerGimbalRad;
                break;
            case SeekerType::SARH:
            case SeekerType::ARH:
                ataLimitRad = 35.0 * DTR;
                break;
            case SeekerType::AntiRadiation:
                ataLimitRad = 30.0 * DTR;
                break;
            default:
                ataLimitRad = 20.0 * DTR;
                break;
        }
        if (std::fabs(env.ata) > ataLimitRad) return false;

        return true;
    }

    // Bomb release envelope (simplified).
    static bool canFireBomb(const WeaponSpec& bomb,
                            const FiringEnvelope& env) {
        if (!bomb.isBomb()) return false;

        // Must be roughly level for release
        // (full impl would check altitude, speed, dive angle)
        const double maxRangeFt = bomb.bombMaxRangeNm * 6076.0;
        if (env.rangeFt > maxRangeFt) return false;

        return true;
    }

    // -----------------------------------------------------------------------
    // SelectBestWeapon — pick the best weapon for the current target.
    //
    // Port of FreeFalcon mengage.cpp:293-488 (WeaponSelection).
    //
    // Scoring: score = rmax*0.8 - range (pick the weapon with the best
    // range margin). IR vs radar preference: prefer radar/ARH for BVR,
    // IR for WVR. If target is a helicopter, prefer IR.
    //
    // Returns WeaponType::None if no weapon is suitable.
    // -----------------------------------------------------------------------
    static WeaponType selectBestWeapon(const StoresManagementSystem& sms,
                                       const FiringEnvelope& env,
                                       bool targetIsHelicopter = false) {
        WeaponType best = WeaponType::None;
        double bestScore = -1e18;

        for (const auto& hp : sms.hardpoints()) {
            if (hp.count <= 0) continue;

            const WeaponSpec spec = weaponSpecOf(hp.type);
            if (spec.type == WeaponType::None) continue;

            // Domain check: A/A weapons for air targets, A/G for ground
            if (weaponDomainOf(hp.type) != WeaponDomain::Air) continue;

            // Range check: must be in WEZ (RMin < range < RMax)
            const double wezMaxFt = spec.isGun() ? spec.maxRangeFt
                                                  : spec.wezMaxNm * 6076.0;
            const double wezMinFt = spec.isGun() ? 100.0
                                                  : spec.wezMinNm * 6076.0;
            if (env.rangeFt > wezMaxFt) continue;
            if (env.rangeFt < wezMinFt) continue;

            // ATA check: must be within seeker limit
            double ataLimit = spec.isGun() ? 35.0 * DTR : spec.seekerGimbalRad;
            if (std::fabs(env.ata) > ataLimit * 1.5) continue;  // 1.5× margin

            // Score: range margin (closer to RMax = better)
            double score = wezMaxFt * 0.8 - env.rangeFt;

            // Helicopter preference: IR missiles
            if (targetIsHelicopter && spec.seeker == SeekerType::IR) {
                score += 5000.0;
            }

            // BVR preference: ARH missiles
            if (env.rangeFt > 8.0 * 6076.0 && spec.seeker == SeekerType::ARH) {
                score += 3000.0;
            }

            if (score > bestScore) {
                bestScore = score;
                best = hp.type;
            }
        }

        return best;
    }

    // -----------------------------------------------------------------------
    // ShouldFire — shoot-shoot / shoot-look doctrine.
    //
    // Port of FreeFalcon dlogic.cpp:892-933 (FireControl doctrine).
    //
    // Shoot-shoot: fire 2nd missile 4 seconds after the 1st (radar) or
    // per team doctrine. Shoot-look: wait TOF + 5s + min(TOF*0.5, 5s)
    // before firing again.
    //
    // The host tracks missileShotTimer; this function just returns whether
    // the doctrine allows firing now.
    // -----------------------------------------------------------------------
    static bool shouldFire(SeekerType seeker,
                           double shootShootPct,  // [0,1] from SkillParameters
                           double timeSinceLastShot,  // seconds
                           double missileTof) {        // seconds
        // The doctrine delay does not currently depend on seeker type — both
        // SARH and ARH missiles use the same shoot-shoot / shoot-look timing
        // (FreeFalcon dlogic.cpp:892-933 keys the doctrine on team stance
        // and shot count, not seeker). The parameter is kept in the signature
        // for the future BVR-port work that will differentiate by seeker.
        (void)seeker;

        // Roll the dice on shoot-shoot
        const bool shootShoot = (shootShootPct >= 0.5);

        if (shootShoot) {
            // Shoot-shoot: 2nd missile 4 seconds after 1st
            return timeSinceLastShot >= 4.0;
        } else {
            // Shoot-look: wait TOF + 5 + min(TOF*0.5, 5)
            const double waitTime = missileTof + 5.0 + std::min(missileTof * 0.5, 5.0);
            return timeSinceLastShot >= waitTime;
        }
    }
};

} // namespace digi
} // namespace f4flight
