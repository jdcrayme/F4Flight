// f4flight - digi/offensive/guns_engage.cpp
//
// Port of FreeFalcon gengage.cpp (580 LOC).
//
// GunsEngage: offensive gun tracking + firing. The AI enters this mode
// when the target is within 3500 ft and ata < 35°. It tracks the target
// with lead (CoarseGunsTrack → GunsAutoTrack), then transitions to fine
// pipper tracking (FineGunsTrack) and fires when the pipper is on target.

#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/core/constants.h"
#include "f4flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// GunsEngageCheck
// ===========================================================================
bool GunsEngageCheck(const DigiState& digi, const DigiEntity& self,
                     const DigiEntity& target, const WeaponSpec& gun,
                     bool isAirToAirMission) {
    (void)self;  // position used in geometry, not here
    (void)digi;

    // Angle limit: 35° for A2A missions, 15° for others (FF gengage.cpp:26-29)
    const double angleLimit = isAirToAirMission ? 35.0 * DTR : 15.0 * DTR;

    // Must have a gun with rounds
    if (!gun.isGun() || gun.roundsRemaining <= 0) return false;

    // Compute relative geometry
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Entry: range ≤ 3500 ft, ata < angleLimit
    // Exit:  range > 3500 ft OR ata > 1.25 × angleLimit OR gun empty
    // (FF gengage.cpp:42-69 — entry uses ata < angleLimit, exit uses
    //  ata < 1.25 × angleLimit for hysteresis)
    if (rg.range > 3500.0) return false;
    if (rg.ata > 1.25 * angleLimit) return false;

    return true;
}

// ===========================================================================
// CoarseGunsTrack
// ===========================================================================
double CoarseGunsTrack(DigiState& digi, const DigiEntity& self,
                       const DigiEntity& target, const AircraftState& as,
                       const WeaponSpec& gun, FcsState& fcsState,
                       double leadTof) {
    // FF gengage.cpp:198-222
    //
    // Project the target ahead by leadTof bullet-TOFs. The gunFactor scales
    // the projection based on range and bullet+ownship velocity.
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    double gunFactor = leadTof * rg.range /
                       (gun.muzzleVelFtps + as.kin.vt);

    // Reduce lead at high ATA (FF gengage.cpp:209-213)
    if (rg.ata > 45.0 * DTR) {
        const double multiplier = std::max(
            1.0 - ((rg.ata - 45.0 * DTR) / (45.0 * DTR)), 0.0);
        gunFactor *= multiplier;
    }

    // Lead-aim point: target pos + target vel * gunFactor, minus gravity drop
    const double tx = target.x + target.vx * gunFactor;
    const double ty = target.y + target.vy * gunFactor;
    const double tz = target.z + target.vz * gunFactor
                    - 0.5 * GRAVITY * gunFactor * gunFactor * 4.0;

    digi.trackX = tx;
    digi.trackY = ty;
    digi.trackZ = tz;

    return ManeuverPrimitives::GunsAutoTrack(digi, as, fcsState, digi.maxGs);
}

// ===========================================================================
// FineGunsTrack
// ===========================================================================
void FineGunsTrack(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const WeaponSpec& gun, const FlightControlSystem& fcs,
                   FcsState& fcsState, double speed, double dt,
                   double& lagAngle) {
    (void)fcs;

    // FF gengage.cpp:224-360
    //
    // Compute the pipper position: where the bullets will be after TOF.
    // TOF = range / (muzzleVel + ownshipVt - rangedot)

    const RelativeGeometry rg = computeRelativeGeometry(self, target);
    const double rangeEst = std::min(rg.range, 2000.0);

    // Bullet time of flight
    const double tf = rangeEst / std::max(
        gun.muzzleVelFtps + as.kin.vt - rg.rangedot, 1.0);

    // Ownship displacement over TOF (velocity + gravity drop)
    double dx = as.kin.vt * as.kin.cosgam * as.kin.cossig * tf;
    double dy = as.kin.vt * as.kin.cosgam * as.kin.sinsig * tf;
    double dz = -(as.kin.vt * as.kin.singam * tf + 0.5 * GRAVITY * tf * tf);

    // Muzzle velocity contribution (bullet leaves along body axes)
    dx += gun.muzzleVelFtps * as.kin.costhe * as.kin.cospsi * tf;
    dy += gun.muzzleVelFtps * as.kin.costhe * as.kin.sinpsi * tf;
    dz -= gun.muzzleVelFtps * as.kin.sinthe * tf;

    // Transform pipper displacement to body frame
    const Matrix3& dcm = as.kin.dcm;
    const double rx = dcm.m[0][0] * dx + dcm.m[1][0] * dy + dcm.m[2][0] * dz;
    const double ry = dcm.m[0][1] * dx + dcm.m[1][1] * dy + dcm.m[2][1] * dz;
    const double rz = dcm.m[0][2] * dx + dcm.m[1][2] * dy + dcm.m[2][2] * dz;

    // Pipper az/el (body frame)
    const double pipperAz = std::atan2(ry, rx);
    const double pipperEl = std::atan2(-rz, std::sqrt(rx * rx + ry * ry + 0.1));
    const double pipperAta = std::sqrt(pipperEl * pipperEl + pipperAz * pipperAz);

    // Pipper rate (for fire decision)
    const double pipperRate = (pipperAta - digi.pastPipperAta) / std::max(dt, 0.001);
    digi.pastPipperAta = pipperAta;
    lagAngle = pipperEl;

    // Error between pipper and target az/el
    const double azerr = rg.az - pipperAz;
    const double elerr = rg.el - pipperEl;
    const double ata = std::sqrt(azerr * azerr + elerr * elerr);

    // ATA rate (for fire decision)
    digi.ataDot = (ata - digi.pastAta) / std::max(dt, 0.001);
    digi.pastAta = ata;

    // --- Phase A: coarse track until pipper near target ---
    if (!digi.waitingForShot) {
        const double leadTime = 2.0 + 2.0 * std::sin(rg.ataFrom);
        double cata = 0.0;
        {
            // CoarseGunsTrack sets trackX/Y/Z and calls GunsAutoTrack.
            // We need to save/restore because FineGunsTrack also writes
            // pStick/rStick below. (FF does the same — CoarseGunsTrack
            // returns ata, then FineGunsTrack may override pStick.)
            const double savedPstick = digi.pStick;
            const double savedRstick = digi.rStick;
            cata = CoarseGunsTrack(digi, self, target, as, gun, fcsState, leadTime);
            // If ata is small, keep the coarse track commands. Otherwise
            // FineGunsTrack will override below.
            if (ata > 60.0 * DTR) {
                speed = digi.cornerSpeed;
            }
            (void)savedPstick;
            (void)savedRstick;
        }

        // Transition to fine track when pipper near target
        // (FF gengage.cpp:319)
        if (std::fabs(azerr) < 2.0 * DTR &&
            elerr < 0.5 * DTR && elerr > -2.0 * DTR) {
            digi.waitingForShot = true;
            digi.pastPstick = as.loads.nzcgb;  // current G
        }

        // Close-range fallback: if the target is very close (< 1500 ft) and
        // roughly on the nose (ata < 5°), fire even without perfect pipper
        // tracking. This mirrors what a real pilot does at close range —
        // you don't need perfect pipper tracking to hit a target at 500 ft.
        // (FF doesn't have this because its pipper geometry is more precise;
        // F4Flight's simplified DCM + bullet model needs a bit more margin.)
        if (rg.range < 1500.0 && std::fabs(rg.ata) < 5.0 * DTR) {
            digi.gunFireFlag = true;
        }
    }
    // --- Phase B: fine track + fire decision ---
    else {
        // Fire when pipper on target
        // (FF gengage.cpp:331-337)
        if (std::fabs(azerr) < 1.5 * DTR &&
            elerr < 0.5 * DTR && elerr > -1.5 * DTR &&
            std::fabs(digi.ataDot) < 50.0 * DTR &&
            rg.range < 2.0 * gun.muzzleVelFtps) {
            digi.gunFireFlag = true;
        }

        // Relax stick — hold the G we had when entering fine track
        ManeuverPrimitives::SetPstick(std::max(digi.pastPstick - 1.0, 0.0),
                                       digi.maxGs, CommandType::GCommand,
                                       digi, as);
        ManeuverPrimitives::SetRstick(0.0, digi, FlightControlSystem{}, fcsState);

        // Stay in fine track if pipper still near target
        if (std::fabs(azerr) < 3.5 * DTR &&
            elerr < 0.5 * DTR && elerr > -1.5 * DTR &&
            std::fabs(pipperRate) < 10.0 * DTR &&
            rg.range < 3000.0) {
            digi.waitingForShot = (std::fabs(digi.ataDot) < 0.01);
        } else {
            digi.waitingForShot = false;
        }
    }

    // Hold speed
    ManeuverPrimitives::MachHold(speed, as.vcas, false,
                                  digi, as, 100.0, 400.0, dt, 100.0);
}

// ===========================================================================
// GunsEngage — main dispatcher
// ===========================================================================
void GunsEngage(DigiState& digi, const DigiEntity& self,
                const DigiEntity& target, const AircraftState& as,
                const WeaponSpec& gun, const FlightControlSystem& fcs,
                FcsState& fcsState, double dt) {
    // FF gengage.cpp:72-195
    //
    // If target is ahead of 3/9 line (ataFrom < 90°): FineGunsTrack directly.
    // If behind 3/9: closure-control BFM.

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    if (rg.ataFrom < 90.0 * DTR) {
        // Ahead of 3/9 — look for a shot
        double lagAngle = 0.0;
        FineGunsTrack(digi, self, target, as, gun, fcs, fcsState,
                      digi.cornerSpeed, dt, lagAngle);
    } else {
        // Behind 3/9 — closure control BFM
        const double CONTROL_POINT_DISTANCE = 1400.0;

        // Range to control point
        double rng;
        if (rg.ataFrom >= 120.0 * DTR) {
            rng = rg.range - CONTROL_POINT_DISTANCE;
        } else {
            rng = -rg.range + 3000.0 - CONTROL_POINT_DISTANCE;
        }

        // Desired closure (kts)
        const double rngdot = -rg.rangedot * FTPSEC_TO_KNOTS;
        double closure = ((rng - rngdot * 5.0) / 1000.0) * 50.0;
        closure = std::max(std::min(closure, 1000.0), -350.0);
        closure = std::min(closure, target.speed + 50.0);
        const double desiredClosure = closure;
        const double actualClosure = rngdot;

        double lagAngle = 0.0;
        if (rg.range < 2000.0) {
            if (actualClosure > desiredClosure) {
                // Too close and too fast — slow down + BFM
                ManeuverPrimitives::MachHold(target.speed - 100.0, as.vcas,
                                              false, digi, as, 100.0, 400.0,
                                              dt, 100.0);
                if (rg.range < 800.0) {
                    // Very close — RollAndPull BFM
                    RollAndPull(digi, self, target, as, fcs, fcsState, dt);
                }
                FineGunsTrack(digi, self, target, as, gun, fcs, fcsState,
                              std::min(target.speed, as.vcas +
                                       (desiredClosure - actualClosure)),
                              dt, lagAngle);
            } else {
                // Too close and slow — point to shoot
                FineGunsTrack(digi, self, target, as, gun, fcs, fcsState,
                              std::min(target.speed, as.vcas +
                                       (desiredClosure - actualClosure)),
                              dt, lagAngle);
            }
        } else {
            // Far range
            if (actualClosure > desiredClosure) {
                // Too far and fast — point to shoot and slow
                FineGunsTrack(digi, self, target, as, gun, fcs, fcsState,
                              std::min(target.speed, as.vcas +
                                       (desiredClosure - actualClosure)),
                              dt, lagAngle);
            } else {
                // Too far and slow — point to shoot, overbank if lagging
                FineGunsTrack(digi, self, target, as, gun, fcs, fcsState,
                              std::min(target.speed + 30.0, as.vcas +
                                       (desiredClosure - actualClosure)),
                              dt, lagAngle);
                // (OverBMode would be added here in full FF port —
                //  we don't have that mode yet, so skip the AddMode call.)
            }
        }
    }
}

} // namespace digi
} // namespace f4flight
