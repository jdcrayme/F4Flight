// f4flight - digi/wingman/wingman_maneuvers.cpp
//
// Wingman tactical maneuver implementation. See wingman_maneuvers.h for the
// architecture rationale and the mapping to FreeFalcon wingactions.cpp.
//
// Round 6 additions:
//   - AiInitPince / AiInitFlex (maneuver-point setup)
//   - AiExecPince / AiExecFlex (multi-point TrackPoint following)
//   - AiExecPosthole / AiExecChainsaw now accept target+sms and can engage

#include "f4flight/digi/wingman/wingman_maneuvers.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/offensive/missile_engage.h"
#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// AiPerformManeuver — dispatch on WingmanState.currentManeuver.
// ===========================================================================
bool AiPerformManeuver(DigiState& digi, const DigiEntity& self,
                       const DigiEntity* target,
                       const StoresManagementSystem* sms,
                       const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt) {
    const auto maneuver = digi.formation.wingman.currentManeuver;

    if (maneuver == WingmanManeuver::None) {
        return false;
    }

    bool stillActive = false;
    switch (maneuver) {
        case WingmanManeuver::None:
            return false;
        case WingmanManeuver::BreakLeft:
        case WingmanManeuver::BreakRight:
            stillActive = AiExecBreakRL(digi, self, as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::ClearSix:
            stillActive = AiExecClearSix(digi, self, as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::Posthole:
            stillActive = AiExecPosthole(digi, self, target, sms,
                                          as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::Chainsaw:
            stillActive = AiExecChainsaw(digi, self, target, sms,
                                          as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::Pince:
            stillActive = AiExecPince(digi, self, as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::Flex:
            stillActive = AiExecFlex(digi, self, as, fcs, fcsState, dt);
            break;
        case WingmanManeuver::SSOffset:
        case WingmanManeuver::CheckSix:
            // Not yet ported — clear and fall back. SSOffset needs the same
            // mpManeuverPoints infrastructure (now available) but with
            // different geometry; CheckSix is a variant of ClearSix.
            AiClearManeuver(digi);
            stillActive = false;
            break;
    }

    if (!stillActive) {
        AiClearManeuver(digi);
    }
    return stillActive;
}

// ===========================================================================
// AiExecBreakRL — break turn.
// FF wingactions.cpp:440-480.
// ===========================================================================
bool AiExecBreakRL(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt) {
    const double heading = digi.formation.wingman.headingOrdered;
    const double tpX = self.x + 1000.0 * std::cos(heading);
    const double tpY = self.y + 1000.0 * std::sin(heading);
    const double tpAlt = (digi.formation.wingman.altitudeOrdered > 0.0)
        ? digi.formation.wingman.altitudeOrdered
        : -self.z;

    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    double desSpeed = digi.formation.wingman.speedOrdered;
    if (desSpeed <= 0.0) desSpeed = digi.config.cornerSpeed;
    desSpeed *= 1.001;
    ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                  digi, as, 150.0, 800.0, dt, 100.0);

    digi.nav.mnverTime -= dt;
    return digi.nav.mnverTime > 0.0;
}

// ===========================================================================
// AiExecClearSix — 180° turn to check six.
// FF wingactions.cpp:611-633.
// ===========================================================================
bool AiExecClearSix(DigiState& digi, const DigiEntity& self,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt) {
    const double heading = digi.formation.wingman.headingOrdered;
    const double tpX = self.x + 1000.0 * std::cos(heading);
    const double tpY = self.y + 1000.0 * std::sin(heading);
    const double tpAlt = (digi.formation.wingman.altitudeOrdered > 0.0)
        ? digi.formation.wingman.altitudeOrdered
        : -self.z;

    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    double desSpeed = digi.formation.wingman.speedOrdered;
    if (desSpeed <= 0.0) desSpeed = digi.config.cornerSpeed;
    desSpeed *= 1.001;
    ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                  digi, as, 150.0, 800.0, dt, 100.0);

    digi.nav.mnverTime -= dt;
    return digi.nav.mnverTime > 0.0;
}

// ===========================================================================
// AiExecPosthole — descend to ordered altitude, then engage.
// FF wingactions.cpp:487-517.
// Round 6: now accepts target + sms so it can actually engage.
// ===========================================================================
bool AiExecPosthole(DigiState& digi, const DigiEntity& self,
                    const DigiEntity* target,
                    const StoresManagementSystem* sms,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt) {
    (void)sms;  // Posthole uses GunsEngage by default (doesn't need sms)

    const int useComplexIdx = static_cast<int>(WingmanAction::UseComplex);
    bool inPhase2 = digi.formation.wingman.actionFlags[useComplexIdx] != 0;

    if (!inPhase2) {
        // Phase 1: descend to ordered altitude at ordered speed.
        const double desAlt = digi.formation.wingman.altitudeOrdered;
        const double desSpeed = (digi.formation.wingman.speedOrdered > 0.0)
            ? digi.formation.wingman.speedOrdered
            : digi.config.cornerSpeed;

        ManeuverPrimitives::TrackPoint(self.x, self.y, desAlt,
                                        digi, as, fcs, fcsState, digi.config.maxGs);
        ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                      digi, as, 150.0, 800.0, dt, 100.0);

        // Transition to phase 2 when within 1000 ft of target altitude.
        if (std::fabs(-as.kin.z - desAlt) < 1000.0) {
            digi.formation.wingman.actionFlags[useComplexIdx] = 1;
            inPhase2 = true;
        }
    }

    if (inPhase2) {
        // Phase 2: engage the target if we have one, else rejoin.
        // FF: if (targetPtr) {
        //       if (curMissile) { FireControl(); MissileEngage(); }
        //       else { GunsEngage(); }
        //     } else { AiRejoin(NULL); }
        //
        // Round 6: we now have target access. If target is null, rejoin
        // (return false). If target is valid, use GunsEngage (the default
        // engage primitive — MissileEngage needs sms which is optional here).
        if (!target || target->isDead) {
            return false;  // no target — rejoin
        }

        // Engage with guns (default). A future enhancement can check sms for
        // missiles and call MissileEngage instead.
        WeaponSpec gun = gunSpec();
        GunsEngage(digi, self, *target, as, gun, fcs, fcsState, dt);
        return true;  // still engaging
    }

    return true;  // still in phase 1
}

// ===========================================================================
// AiExecChainsaw — missile-only engage.
// FF wingactions.cpp:522-534.
// Round 6: now accepts target + sms so it can actually engage.
// ===========================================================================
bool AiExecChainsaw(DigiState& digi, const DigiEntity& self,
                    const DigiEntity* target,
                    const StoresManagementSystem* sms,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt) {
    // FF: if (targetPtr and curMissile) { FireControl(); MissileEngage(); }
    //     else { AiRejoin(NULL); }
    //
    // Round 6: we now have target + sms access. If either is null, or sms
    // has no missiles, rejoin (return false).
    if (!target || target->isDead) {
        return false;  // no target — rejoin
    }
    if (!sms || !sms->hasWeaponClass(WeaponClass::AimWpn)) {
        return false;  // no missiles — rejoin
    }

    // Engage with missiles.
    MissileEngage(digi, self, *target, as, *sms, fcs, fcsState, dt);
    return true;
}

// ===========================================================================
// AiInitPince — set up the 2 Pince maneuver points.
// FF wingai.cpp:745-844.
// ===========================================================================
void AiInitPince(DigiState& digi, const DigiEntity& self,
                 const DigiEntity* target, const DigiEntity* lead) {
    // Determine the maneuver axis (trigYaw).
    // FF: target bearing if target, else lead's yaw, else self's yaw.
    double trigYaw;
    if (target) {
        // TargetAz(self, target) = bearing to target relative to self heading.
        const double dx = target->x - self.x;
        const double dy = target->y - self.y;
        const double bearingToTarget = std::atan2(dy, dx);
        trigYaw = self.yaw + (bearingToTarget - self.yaw);
        // Simplify: trigYaw = bearingToTarget (world-frame bearing).
        trigYaw = bearingToTarget;
        digi.formation.wingman.speedOrdered = self.speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -self.z;
    } else if (lead) {
        trigYaw = lead->yaw;
        digi.formation.wingman.speedOrdered = lead->speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -lead->z;
    } else {
        trigYaw = self.yaw;
        digi.formation.wingman.speedOrdered = self.speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -self.z;
    }

    // Side: odd wingman slots go right (+1), even go left (-1).
    // FF: if (doSplit and (isWing & 1)) side = 1.0; else side = -1.0;
    // We use vehicleInUnit as the slot index.
    const double side = (digi.formation.vehicleInUnit & 1) ? 1.0 : -1.0;

    // FF uses config vars g_fPinceManeuverPoints1a/1b/2a/2b (defaulting to
    // 20/5/4/5 NM). We hardcode the FF defaults here; a future config port
    // can make these tunable.
    constexpr double kPince1a = 20.0;  // NM along bearing
    constexpr double kPince1b = 5.0;   // NM lateral offset
    constexpr double kPince2a = 4.0;   // NM along bearing (second leg)
    constexpr double kPince2b = 5.0;   // NM lateral offset

    const double cosT = std::cos(trigYaw);
    const double sinT = std::sin(trigYaw);

    // Point 0: 20 NM along bearing + 5 NM lateral (mirrored by side).
    digi.formation.maneuverPoints[0].x =
        self.x + cosT * kPince1a * 6076.0 - sinT * kPince1b * 6076.0 * side;
    digi.formation.maneuverPoints[0].y =
        self.y + sinT * kPince1a * 6076.0 + cosT * kPince1b * 6076.0 * side;

    // Point 1: 4 NM along bearing + 5 NM lateral (mirrored by side).
    digi.formation.maneuverPoints[1].x =
        self.x + cosT * kPince2a * 6076.0 - sinT * kPince2b * 6076.0 * side;
    digi.formation.maneuverPoints[1].y =
        self.y + sinT * kPince2a * 6076.0 + cosT * kPince2b * 6076.0 * side;

    digi.formation.maneuverPointCounter = 0;
}

// ===========================================================================
// AiInitFlex — set up the 3 Flex maneuver points.
// FF wingai.cpp:857-884.
// ===========================================================================
void AiInitFlex(DigiState& digi, const DigiEntity& self,
                const DigiEntity* target, const DigiEntity* lead) {
    // FF uses AiInitTrig to compute firstTrig (target/lead/self bearing) and
    // secondTrig (firstTrig + 90°). We inline the equivalent.
    double trigYaw;
    if (target) {
        const double dx = target->x - self.x;
        const double dy = target->y - self.y;
        trigYaw = std::atan2(dy, dx);
        digi.formation.wingman.speedOrdered = self.speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -self.z;
    } else if (lead) {
        trigYaw = lead->yaw;
        digi.formation.wingman.speedOrdered = lead->speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -lead->z;
    } else {
        trigYaw = self.yaw;
        digi.formation.wingman.speedOrdered = self.speed / KNOTS_TO_FTPSEC;
        digi.formation.wingman.altitudeOrdered = -self.z;
    }

    // FF: secondTrig is firstTrig + 90° (perpendicular to the bearing).
    const double firstYaw = trigYaw;
    const double secondYaw = trigYaw + PI / 2.0;

    const double firstSin = std::sin(firstYaw);
    const double secondCos = std::cos(secondYaw);
    const double secondSin = std::sin(secondYaw);

    // FF: spacing = 1 NM. Points are offset perpendicular to the bearing.
    const double spacing = 6076.0;  // 1 NM in feet

    // Point 0: +1 NM perpendicular.
    digi.formation.maneuverPoints[0].x = self.x + secondCos * spacing;
    digi.formation.maneuverPoints[0].y = self.y + firstSin * spacing;

    // Point 1: -2 NM perpendicular (opposite direction, further out).
    digi.formation.maneuverPoints[1].x = self.x - secondCos * 2.0 * spacing;
    digi.formation.maneuverPoints[1].y = self.y - secondSin * 2.0 * spacing;

    // Point 2: -2.1 NM perpendicular (slightly further — S-curve completion).
    digi.formation.maneuverPoints[2].x = self.x - secondCos * 2.1 * spacing;
    digi.formation.maneuverPoints[2].y = self.y - secondSin * 2.1 * spacing;

    digi.formation.maneuverPointCounter = 0;
}

// ===========================================================================
// AiExecPince — fly through the 2 Pince points, then clear.
// FF wingactions.cpp:540-569.
// ===========================================================================
bool AiExecPince(DigiState& digi, const DigiEntity& self,
                 const AircraftState& as,
                 const FlightControlSystem& fcs, FcsState& fcsState,
                 double dt) {
    // Check arrival at the current point.
    const int idx = digi.formation.maneuverPointCounter;
    if (idx >= 2) {
        return false;  // both points visited — maneuver complete
    }

    const double dx = digi.formation.maneuverPoints[idx].x - self.x;
    const double dy = digi.formation.maneuverPoints[idx].y - self.y;
    const double deltaSq = dx * dx + dy * dy;

    // FF: if (deltaSq <= 5000.0 * 5000.0) mPointCounter++;
    if (deltaSq <= kPinceArrivalThresholdFt * kPinceArrivalThresholdFt) {
        ++digi.formation.maneuverPointCounter;
    }

    // If we just advanced past the last point, complete.
    if (digi.formation.maneuverPointCounter >= 2) {
        return false;
    }

    // Fly to the current point.
    const double tpX = digi.formation.maneuverPoints[digi.formation.maneuverPointCounter].x;
    const double tpY = digi.formation.maneuverPoints[digi.formation.maneuverPointCounter].y;
    const double tpAlt = (digi.formation.wingman.altitudeOrdered > 0.0)
        ? digi.formation.wingman.altitudeOrdered
        : -self.z;

    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    // Speed: ordered speed with tiny ramp-up (matches FF's mSpeedOrdered *= 1.001F).
    double desSpeed = digi.formation.wingman.speedOrdered;
    if (desSpeed <= 0.0) desSpeed = digi.config.cornerSpeed;
    desSpeed *= 1.001;
    ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                  digi, as, 150.0, 800.0, dt, 100.0);

    return true;
}

// ===========================================================================
// AiExecFlex — fly through the 3 Flex points, then clear.
// FF wingactions.cpp:575-601.
// ===========================================================================
bool AiExecFlex(DigiState& digi, const DigiEntity& self,
                const AircraftState& as,
                const FlightControlSystem& fcs, FcsState& fcsState,
                double dt) {
    // FF: TOTAL_MANEUVER_PTS = 3 for Flex.
    constexpr int kFlexTotalPoints = 3;

    const int idx = digi.formation.maneuverPointCounter;
    if (idx >= kFlexTotalPoints) {
        return false;  // all points visited — maneuver complete
    }

    const double dx = digi.formation.maneuverPoints[idx].x - self.x;
    const double dy = digi.formation.maneuverPoints[idx].y - self.y;
    const double deltaSq = dx * dx + dy * dy;

    // FF: if (deltaSq <= 900.0 * 900.0) mPointCounter++;
    if (deltaSq <= kFlexArrivalThresholdFt * kFlexArrivalThresholdFt) {
        ++digi.formation.maneuverPointCounter;
    }

    if (digi.formation.maneuverPointCounter >= kFlexTotalPoints) {
        return false;
    }

    const double tpX = digi.formation.maneuverPoints[digi.formation.maneuverPointCounter].x;
    const double tpY = digi.formation.maneuverPoints[digi.formation.maneuverPointCounter].y;
    const double tpAlt = (digi.formation.wingman.altitudeOrdered > 0.0)
        ? digi.formation.wingman.altitudeOrdered
        : -self.z;

    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    double desSpeed = digi.formation.wingman.speedOrdered;
    if (desSpeed <= 0.0) desSpeed = digi.config.cornerSpeed;
    desSpeed *= 1.001;
    ManeuverPrimitives::MachHold(desSpeed, as.vcas, true,
                                  digi, as, 150.0, 800.0, dt, 100.0);

    return true;
}

// ===========================================================================
// AiClearManeuver — clear the current wingman maneuver and reset state.
// ===========================================================================
void AiClearManeuver(DigiState& digi) {
    digi.formation.wingman.currentManeuver = WingmanManeuver::None;
    digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 0;
    digi.formation.wingman.actionFlags[static_cast<int>(WingmanAction::UseComplex)] = 0;
    digi.nav.mnverTime = 0.0;
    digi.formation.maneuverPointCounter = 0;
}

} // namespace digi
} // namespace f4flight
