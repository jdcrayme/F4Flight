// f4flight - digi/offensive/bvr_engage.cpp
//
// Port of FreeFalcon bvrengage.cpp (3,238 LOC, simplified to ~400 LOC).
//
// BvrEngage: beyond-visual-range engagement. The AI selects a tactical
// profile based on threat score, then executes the corresponding tactic
// (Pursuit, Crank, Beam, or Drag) via StickandThrottle + AutoTrack.

#include "f4flight/digi/offensive/bvr_engage.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/airspeed_conversions.h"  // casFromTasFps
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// BvrEngageCheck
// ===========================================================================
bool BvrEngageCheck(const DigiState& digi, const DigiEntity& self,
                    const DigiEntity& target, double maxAAWpnRangeFt) {
    (void)digi;

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Entry range: max(maxAAWpnRange × 1.3, 45 NM)
    // (FF bvrengage.cpp:100-103)
    double engageRange = std::max(maxAAWpnRangeFt * 1.3, 45.0 * 6076.0);

    // Must be beyond 8 NM (RAP distance — inside that, MissileEngage handles)
    if (rg.range < 8.0 * 6076.0) return false;
    // Must be within engage range
    if (rg.range > engageRange) return false;

    return true;
}

// ===========================================================================
// ChoiceProfile
// ===========================================================================
BvrProfile ChoiceProfile(const DigiEntity& self, const DigiEntity& target,
                         double maxAAWpnRangeFt) {
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    int threatScore = 0;
    const double myMissile = maxAAWpnRangeFt;

    // Inferior missile?
    if (myMissile < 10.0 * 6076.0)
        threatScore += 20;

    // Outranged?
    if (rg.range > myMissile)
        threatScore += 60;

    // Height advantage?
    // NED: z negative up. target higher = z more negative.
    if (target.z < self.z)
        threatScore += 5;

    // Speed advantage?
    if (target.speed > self.speed)
        threatScore += 5;

    // Positional advantage: target on our tail
    // (ataFrom < 90° means target pointing at us, ata > 90° means target
    //  behind our nose — i.e. we're chasing but he's on our tail)
    if (rg.ataFrom < 90.0 * DTR && rg.ata > 90.0 * DTR)
        threatScore += 20;

    // Out of missile range?
    if (rg.range > myMissile)
        threatScore += 10;

    // Special case: we're on target's tail — force offensive
    if (rg.ataFrom > 90.0 * DTR && rg.ata < 90.0 * DTR)
        threatScore = 5;

    // Profile selection (FF bvrengage.cpp:749-772)
    if (threatScore >= 60)       return BvrProfile::Defensive;
    else if (threatScore >= 50)  return BvrProfile::Level3c;
    else if (threatScore >= 30)  return BvrProfile::Level2c;
    else if (threatScore >= 20)  return BvrProfile::Level3b;
    else if (threatScore >= 10)  return BvrProfile::Grinder;
    else                         return BvrProfile::Wall;
}

// ===========================================================================
// BvrChooseTactic
// ===========================================================================
BvrTactic BvrChooseTactic(BvrProfile profile, const DigiEntity& self,
                          const DigiEntity& target) {
    (void)self;
    (void)target;

    switch (profile) {
        case BvrProfile::Wall:       return BvrTactic::Pursuit;
        case BvrProfile::Grinder:    return BvrTactic::Pursuit;
        case BvrProfile::Level3b:    return BvrTactic::Crank;
        case BvrProfile::Level2c:    return BvrTactic::Beam;
        case BvrProfile::Level3c:    return BvrTactic::Beam;
        case BvrProfile::Defensive:  return BvrTactic::Drag;
        default:                     return BvrTactic::Pursuit;
    }
}

// ===========================================================================
// StickandThrottle
// ===========================================================================
void StickandThrottle(DigiState& digi, const DigiEntity& self,
                      const AircraftState& as,
                      const FlightControlSystem& fcs, FcsState& fcsState,
                      double desiredSpeedKts, double desiredAltFt,
                      double dt) {
    (void)self;
    (void)fcs;

    // Adjust trackZ for altitude management (FF bvrengage.cpp:3048-3066,
    // simplified). NOTE: this is a placeholder — the FF version trades
    // altitude for speed (descend to accelerate, climb to decelerate) via
    // a proper energy-management loop. Here we only nudge trackZ toward
    // the desired altitude when we're roughly at target speed; the
    // "need speed" branch intentionally leaves trackZ at the target's
    // altitude (set by the caller) rather than commanding a descent.
    // TODO: port the FF energy-management loop when BVR tuning is exercised
    //       by a scenario (currently BVR is smoke-test only — see test audit).
    const double speedDiff = desiredSpeedKts - as.vcas;

    if (speedDiff <= 50.0) {
        // Near target speed — adjust trackZ toward desired altitude.
        if (-as.kin.z < desiredAltFt) {
            // Below desired — command a climb (more-negative NED z).
            digi.nav.trackZ = std::min(as.kin.z + speedDiff * 800.0, -desiredAltFt);
        } else {
            // At or above desired — hold.
            digi.nav.trackZ = -desiredAltFt;
        }
    }
    // Clamp to avoid ground
    digi.nav.trackZ = std::min(digi.nav.trackZ, -4000.0);

    // Track the target via AutoTrack
    ManeuverPrimitives::AutoTrack(digi, as, fcsState, digi.config.maxGs);

    // Hold speed
    //
    // desiredSpeedKts is CALIBRATED airspeed in kts (the callers in this
    // file pass CAS — cornerSpeed, targetCasKts + 100, as.vcas). Use the
    // typed machHoldCas API to make this contract explicit at compile time.
    ManeuverPrimitives::machHoldCas(cas_kts(desiredSpeedKts), true,
                                     digi, as, 200.0, 800.0, dt, 700.0);
}

// ===========================================================================
// CrankManeuver — turn 45° off the target heading
// ===========================================================================
void CrankManeuver(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt, int direction) {
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // If we're already on the target's tail (ataFrom > 90° = we're behind
    // their 3/9 line), cranking makes no sense — just pursue. ataFrom is
    // the angle off the TARGET's nose to us, so > 90° means we're in their
    // rear hemisphere (the advantageous position cranking would throw away).
    if (rg.ataFrom > 90.0 * DTR) {
        digi.nav.trackX = target.x;
        digi.nav.trackY = target.y;
        digi.nav.trackZ = target.z;
        // CAS/TAS CORRECTION: target.speed is TRUE airspeed in ft/s (see
        // DigiEntity.speed, populated from as.kin.vt). StickandThrottle's
        // desiredSpeedKts parameter is compared against as.vcas (CALIBRATED
        // airspeed in kts), so we must convert the target's TAS to the
        // equivalent CAS at our altitude before adding the 100 kt margin.
        // Without this conversion the BVR pursuit would target a CAS equal
        // to (target_TAS_ftps + 100) kts — far above the target's actual CAS,
        // forcing the AI into full afterburner and overspeed.
        //
        // Use the typed casFromTasFps helper, which converts target's TAS
        // (ft/s) to CAS-kts using the wingman's own CAS/TAS ratio. This
        // replaces the manual casToTasRatio idiom that was copy-pasted here.
        const CasKnots targetCas = casFromTasFps(tas_fps(target.speed), as);
        StickandThrottle(digi, self, as, fcs, fcsState,
                         targetCas.count() + 100.0, -target.z, dt);
        return;
    }

    // Pick a heading 45° off the target bearing
    // (FF bvrengage.cpp:2916-2954)
    double az = self.yaw + rg.az;  // world-frame bearing to target
    double nh1 = az + 45.0 * DTR;
    double nh2 = az - 45.0 * DTR;
    // Wrap
    while (nh1 >  PI) nh1 -= 2.0 * PI;
    while (nh1 < -PI) nh1 += 2.0 * PI;
    while (nh2 >  PI) nh2 -= 2.0 * PI;
    while (nh2 < -PI) nh2 += 2.0 * PI;

    // Pick direction
    double chosenAz;
    if (direction == 1 || (rg.azFrom < 0.0 && direction != 2)) {
        chosenAz = nh1;  // right
    } else {
        chosenAz = nh2;  // left
    }

    // Set trackpoint 10 NM in the crank direction
    const double trackDist = 10.0 * 6076.0;
    digi.nav.trackX = self.x + trackDist * std::cos(chosenAz);
    digi.nav.trackY = self.y + trackDist * std::sin(chosenAz);
    digi.nav.trackZ = self.z;  // hold altitude

    StickandThrottle(digi, self, as, fcs, fcsState,
                     digi.config.cornerSpeed, -self.z, dt);
}

// ===========================================================================
// BeamManeuver — turn 90° to the target heading
// ===========================================================================
void BeamManeuver(DigiState& digi, const DigiEntity& self,
                  const DigiEntity& target, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt, int direction) {
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Pick a heading 90° off the target bearing
    // (FF bvrengage.cpp:2822-2841)
    double az = self.yaw + rg.az;
    double nh1 = az + 90.0 * DTR;
    double nh2 = az - 90.0 * DTR;
    while (nh1 >  PI) nh1 -= 2.0 * PI;
    while (nh1 < -PI) nh1 += 2.0 * PI;
    while (nh2 >  PI) nh2 -= 2.0 * PI;
    while (nh2 < -PI) nh2 += 2.0 * PI;

    double chosenAz;
    if (direction == 1 || (rg.az < 0.0 && direction != 2)) {
        chosenAz = nh1;
    } else {
        chosenAz = nh2;
    }

    // Set trackpoint 4 NM in the beam direction
    const double trackDist = 4.0 * 6076.0;
    digi.nav.trackX = self.x + trackDist * std::cos(chosenAz);
    digi.nav.trackY = self.y + trackDist * std::sin(chosenAz);
    // Hold altitude (or descend slightly for beam)
    digi.nav.trackZ = std::max(self.z, -10000.0);

    StickandThrottle(digi, self, as, fcs, fcsState,
                     digi.config.cornerSpeed, -digi.nav.trackZ, dt);
}

// ===========================================================================
// DragManeuver — turn cold (away from target)
// ===========================================================================
void DragManeuver(DigiState& digi, const DigiEntity& self,
                  const DigiEntity& target, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt) {
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Drag: turn cold — fly directly AWAY from the target so any missile it
    // fires has to chase us through our exhaust path (maximizes its kinematic
    // bleed). az here is the world-frame bearing FROM the target TO us
    // (target.yaw + rg.azFrom = bearingToSelf), so placing the trackpoint
    // along that bearing from our position sends us away from the target.
    // (FF bvrengage.cpp:2989-3005)
    double az = target.yaw + rg.azFrom;  // world-frame bearing from target to us
    while (az >  PI) az -= 2.0 * PI;
    while (az < -PI) az += 2.0 * PI;

    // Set trackpoint 10 NM in the drag direction
    const double trackDist = 10.0 * 6076.0;
    digi.nav.trackX = self.x + trackDist * std::cos(az);
    digi.nav.trackY = self.y + trackDist * std::sin(az);
    digi.nav.trackZ = self.z;

    // Max speed for drag (escape)
    StickandThrottle(digi, self, as, fcs, fcsState,
                     as.vcas, -self.z, dt);
}

// ===========================================================================
// BvrEngage — main mode dispatcher
// ===========================================================================
void BvrEngage(DigiState& digi, const DigiEntity& self,
               const DigiEntity& target, const AircraftState& as,
               const FlightControlSystem& fcs, FcsState& fcsState,
               double dt) {
    // FF bvrengage.cpp:218-440 (simplified)

    if (digi.groundAvoid.groundAvoidNeeded) return;

    // Set trackpoint to target for AutoTrack
    digi.nav.trackX = target.x;
    digi.nav.trackY = target.y;
    digi.nav.trackZ = target.z;

    // Select profile + tactic
    const BvrProfile profile = ChoiceProfile(self, target, digi.weapon.maxAAWpnRange);
    const BvrTactic tactic = BvrChooseTactic(profile, self, target);

    // Execute tactic
    switch (tactic) {
        case BvrTactic::Pursuit:
            // Head toward target, shoot when in range
            StickandThrottle(digi, self, as, fcs, fcsState,
                             1.3 * digi.config.cornerSpeed, -target.z, dt);
            break;

        case BvrTactic::Crank:
            CrankManeuver(digi, self, target, as, fcs, fcsState, dt);
            break;

        case BvrTactic::Beam:
            BeamManeuver(digi, self, target, as, fcs, fcsState, dt);
            break;

        case BvrTactic::Drag:
            DragManeuver(digi, self, target, as, fcs, fcsState, dt);
            break;

        default:
            StickandThrottle(digi, self, as, fcs, fcsState,
                             digi.config.cornerSpeed, -target.z, dt);
            break;
    }
}

} // namespace digi
} // namespace f4flight
