// f4flight - digi/offensive/missile_engage.cpp
//
// Port of FreeFalcon mengage.cpp (489 LOC) + dlogic.cpp:792-933 (FireControl).
//
// MissileEngage: offensive missile firing. The AI selects the best missile,
// tracks the target with lead, and fires when the target is in the firing
// envelope.

#include "f4flight/digi/offensive/missile_engage.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/weapons/fire_control.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// MissileEngageCheck
// ===========================================================================
bool MissileEngageCheck(const DigiState& digi, const DigiEntity& self,
                        const DigiEntity& target,
                        const StoresManagementSystem& sms,
                        bool hasGun) {
    (void)digi;
    (void)hasGun;

    // Must have at least one A/A missile
    if (!sms.hasWeaponClass(WeaponClass::AimWpn)) return false;

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Max A/A weapon range: find the best missile's RMax
    double maxRangeFt = 0.0;
    for (const auto& hp : sms.hardpoints()) {
        if (hp.wclass != WeaponClass::AimWpn || hp.count <= 0) continue;
        const WeaponSpec spec = weaponSpecOf(hp.type);
        maxRangeFt = std::max(maxRangeFt, spec.wezMaxNm * 6076.0);
    }
    if (maxRangeFt <= 0.0) return false;

    // Entry: range ≤ maxAAWpnRange × 1.05, ata < 60° × 1.05
    // (FF mengage.cpp:55-58)
    if (rg.range > maxRangeFt * 1.05) return false;
    if (rg.ata > 60.0 * DTR * 1.05) return false;

    return true;
}

// ===========================================================================
// WeaponSelection
// ===========================================================================
WeaponType WeaponSelection(const StoresManagementSystem& sms,
                           const DigiEntity& self,
                           const DigiEntity& target,
                           double& outMaxAAWpnRangeFt) {
    (void)self;

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    WeaponType best = WeaponType::None;
    double bestScore = -1e18;
    outMaxAAWpnRangeFt = 0.0;

    for (const auto& hp : sms.hardpoints()) {
        if (hp.wclass != WeaponClass::AimWpn || hp.count <= 0) continue;

        const WeaponSpec spec = weaponSpecOf(hp.type);
        const double rmaxFt = spec.wezMaxNm * 6076.0;

        // Track max A/A weapon range
        if (rmaxFt > outMaxAAWpnRangeFt) outMaxAAWpnRangeFt = rmaxFt;

        // Score: range margin (rmax*0.8 - range)
        // (FF mengage.cpp:393)
        const double thisPctRange = rmaxFt * 0.8 - rg.range;
        if (thisPctRange < 0.0) continue;  // out of range

        double score = thisPctRange;

        // BVR preference: ARH missiles beyond 8 NM
        if (rg.range > 8.0 * 6076.0 && spec.seeker == SeekerType::ARH) {
            score += 50000.0;
        }

        // WVR preference: IR missiles within 8 NM
        if (rg.range < 8.0 * 6076.0 && spec.seeker == SeekerType::IR) {
            score += 10000.0;
        }

        if (score > bestScore) {
            bestScore = score;
            best = hp.type;
        }
    }

    return best;
}

// ===========================================================================
// FireControlMissile — firing decision
// ===========================================================================
void FireControlMissile(DigiState& digi, const DigiEntity& self,
                        const DigiEntity& target,
                        const WeaponSpec& missile,
                        const StoresManagementSystem& sms,
                        double dt) {
    (void)sms;
    (void)dt;

    // Basic check: have a missile, have a target
    if (missile.type == WeaponType::None) return;

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Build the firing envelope
    FiringEnvelope env;
    env.rangeFt  = rg.range;
    env.rangedot = rg.rangedot;
    env.ata      = rg.ata;
    env.ataFrom  = rg.ataFrom;
    env.altFt    = -self.z;
    env.vtFtps   = self.speed;
    env.targetVt = target.speed;

    // Check firing envelope via FireControl
    if (!FireControl::canFireMissile(missile, env)) return;

    // Doctrine: shoot-shoot or shoot-look
    // (FF dlogic.cpp:892-933)
    const double shootShootPct = (missile.seeker == SeekerType::IR)
        ? digi.skill.shootShootPctHeat
        : digi.skill.shootShootPctRadar;

    const bool shootShoot = (shootShootPct >= 0.5);

    // Check timer
    if (digi.missileShotTimer > 0.0) {
        // Still waiting for the next shot
        return;
    }

    // Fire!
    digi.mslFireFlag = true;

    // Set the timer for the next shot
    if (shootShoot) {
        // Shoot-shoot: 2nd missile 4 seconds later
        digi.missileShotTimer = 4.0;
        digi.inShootShoot = true;
    } else {
        // Shoot-look: wait TOF + 5 + min(TOF*0.5, 5)
        // Approximate TOF as range / (missileSpeed + ownshipVt)
        const double missileSpeed = 2000.0;  // ~Mach 2 average
        const double tof = rg.range / std::max(missileSpeed + env.vtFtps, 1.0);
        const double delay = tof + 5.0 + std::min(tof * 0.5, 5.0);
        digi.missileShotTimer = delay;
        digi.inShootShoot = false;
    }
}

// ===========================================================================
// MissileEngage — main mode dispatcher
// ===========================================================================
void MissileEngage(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const StoresManagementSystem& sms,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt) {
    // FF mengage.cpp:101-291

    // Select the best missile
    double maxAAWpnRangeFt = 0.0;
    const WeaponType missileType = WeaponSelection(sms, self, target,
                                                    maxAAWpnRangeFt);
    if (missileType == WeaponType::None) {
        // No missile available — fall back to WVR BFM
        RollAndPull(digi, self, target, as, fcs, fcsState, dt);
        return;
    }

    // Update max A/A weapon range in state (for mode checks)
    digi.maxAAWpnRange = maxAAWpnRangeFt;

    const WeaponSpec missile = weaponSpecOf(missileType);
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // --- Fire control ---
    // Decrement the shot timer
    if (digi.missileShotTimer > 0.0) {
        digi.missileShotTimer -= dt;
    }
    FireControlMissile(digi, self, target, missile, sms, dt);

    // --- Tracking ---
    // FF mengage.cpp:158-290: within RAP distance, defer to RollAndPull.
    // Beyond RAP distance, do BVR tracking with lead + closure control.
    const double rapDistanceFt = 8.0 * 6076.0;  // ~8 NM

    if (rg.range <= rapDistanceFt) {
        // Within RAP — WVR BFM
        RollAndPull(digi, self, target, as, fcs, fcsState, dt);
    } else {
        // BVR tracking — lead pursuit with closure control
        // (FF mengage.cpp:161-289)

        // Compute TOF for lead
        const double missileSpeed = 2000.0;  // ~Mach 2 average
        const double tof = rg.range / std::max(missileSpeed + self.speed, 1.0);

        // Deadband on target zdot
        double zDot = target.vz * 0.1;
        zDot = std::max(std::min(zDot, 100.0), -100.0);

        // Set trackpoint with lead
        if (std::fabs(rg.azFrom) < 90.0 * DTR) {
            // Ahead of target 3/9 line — lead pursuit
            digi.trackX = target.x + target.vx * tof;
            digi.trackY = target.y + target.vy * tof;
            digi.trackZ = target.z + zDot * tof;
        } else {
            // Behind target 3/9 line — closure control.
            // FF mengage.cpp:318-336: rdes is the desired standoff range
            // (40% of missile WEZ). Drive closure to keep range trending
            // toward rdes — fast-closing aircraft should lag, slow-closing
            // aircraft should pure-pursue.
            const double rdes = 0.40 * missile.wezMaxNm * 6076.0;
            double desiredClosure = ((rg.range - rdes) / 1000.0) * 50.0;
            desiredClosure = std::max(std::min(desiredClosure, 2300.0), -100.0);

            const double actualClosure = -rg.rangedot * FTPSEC_TO_KNOTS;

            if (actualClosure > desiredClosure) {
                // Closing too fast — lag the target
                digi.trackX = target.x + target.vx * tof * 0.9;
                digi.trackY = target.y + target.vy * tof * 0.9;
                digi.trackZ = target.z + zDot * tof * 0.9;
                if (rg.range > 10.0 * 6076.0) {
                    digi.trackZ -= 10000.0;  // drop below for separation
                }
            } else {
                // Not closing fast enough — pure lead
                digi.trackX = target.x + target.vx * tof;
                digi.trackY = target.y + target.vy * tof;
                digi.trackZ = target.z + zDot * tof;
            }
        }

        // Clamp trackZ to avoid ground
        digi.trackZ = std::min(digi.trackZ, -4000.0);

        // Track the lead point via AutoTrack
        ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.maxGs);

        // Speed control: 1.3× corner speed for BVR approach
        const double desSpeed = 1.3 * digi.cornerSpeed;
        ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                      digi, as, 200.0, 800.0, dt, 100.0);
    }
}

} // namespace digi
} // namespace f4flight
