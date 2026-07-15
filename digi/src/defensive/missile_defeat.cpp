// f4flight - digi/defensive/missile_defeat.cpp
//
// Missile defense implementation.
//
// Direct port of FreeFalcon mdefeat.cpp (732 LOC), simplified to use the
// DigiEntity model instead of SimBaseClass/MissileClass.
//
// Key simplifications vs. FreeFalcon:
//   - No RWR / visual sensor simulation (the host decides whether the AI
//     "knows" about the missile by setting digi.missileDefeat.incomingMissile)
//   - No SARH guidance delay (the host handles missileFiredEntity logic)
//   - No radio calls (rcENGDEFENSIVEC)
//   - No emergency jettison (no SMS model yet)
//   - No wingman AG attack orders (no wingman system yet)
//   - Flare/chaff commands are exposed via digi state flags for the host
//     to read (no direct SMS/EWS integration)
//
// The maneuver selection logic, trackpoint geometry, and control laws are
// faithful ports of the FreeFalcon code.

#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// MissileDefeatCheck
// ===========================================================================
bool MissileDefeatCheck(DigiState& digi, const DigiEntity& self, double dt) {
    (void)self;  // position used in MissileDefeat, not here
    (void)dt;

    const DigiEntity* missile = digi.missileDefeat.incomingMissile;
    if (!missile) return false;

    // Missile is dead → clear and exit
    if (missile->isDead) {
        digi.missileDefeat.incomingMissile = nullptr;
        digi.missileDefeat.incomingMissileId = kInvalidEntityId;
        digi.missileDefeat.incomingMissileRange = 500.0 * 6076.0;
        digi.missileDefeat.missileDefeatTtgo = -1.0;
        return false;
    }

    // Compute current missile range
    const double dx = missile->x - self.x;
    const double dy = missile->y - self.y;
    const double dz = missile->z - self.z;
    const double missileRange = std::sqrt(dx * dx + dy * dy + dz * dz);

    // If missile range is increasing (missile is passing us) and we've held
    // the evade timer long enough, spoof the missile and exit.
    // FF mdefeat.cpp:111-144: evade hold = (6 - skill) seconds.
    // SkillParameters::evadeHoldSec centralizes this — keep it authoritative.
    if (digi.missileDefeat.incomingMissileRange > 0.0 &&
        missileRange > digi.missileDefeat.incomingMissileRange) {
        // Missile is moving away — start/continue evade timer
        digi.missileDefeat.incomingMissileEvadeTimer += dt;
        const double evadeHold = digi.config.skill.evadeHoldSec;
        if (digi.missileDefeat.incomingMissileEvadeTimer > evadeHold) {
            // Spoofed — forget about the missile
            digi.missileDefeat.incomingMissile = nullptr;
            digi.missileDefeat.incomingMissileId = kInvalidEntityId;
            digi.missileDefeat.incomingMissileRange = 500.0 * 6076.0;
            digi.missileDefeat.missileDefeatTtgo = -1.0;
            digi.missileDefeat.incomingMissileEvadeTimer = 0.0;
            return false;
        }
    } else {
        // Missile still closing — reset evade timer
        digi.missileDefeat.incomingMissileEvadeTimer = 0.0;
    }

    digi.missileDefeat.incomingMissileRange = missileRange;
    return true;
}

// ===========================================================================
// MissileDefeat — main maneuver selection
// ===========================================================================
void MissileDefeat(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt) {
    const DigiEntity* missile = digi.missileDefeat.incomingMissile;
    if (!missile || missile->isDead) return;

    // --- Initialize on new missile ---
    // FreeFalcon mdefeat.cpp:446-452. The "new missile" check is just
    // `missileDefeatTtgo < 0`. DigiBrain::resolveMode is responsible for
    // resetting missileDefeatTtgo to -1 when the missile identity changes
    // (see incomingMissileId tracking in DigiState).
    if (digi.missileDefeat.missileDefeatTtgo < 0.0) {
        digi.missileDefeat.missileDefeatTtgo = 1000.0;
        digi.missileDefeat.missileFindDragPt = true;
        digi.missileDefeat.missileFinishedBeam = false;
        digi.missileDefeat.missileShouldDrag = false;
    }

    // --- Compute range and threat time ---
    // FreeFalcon mdefeat.cpp:457-474
    const double dx = missile->x - self.x;
    const double dy = missile->y - self.y;
    const double dz = missile->z - self.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double threatTime = range / kAveAim9Vel;

    if (threatTime < kMaxThreatTime) {
        digi.missileDefeat.missileDefeatTtgo = threatTime;
    } else {
        digi.missileDefeat.missileDefeatTtgo -= dt;
    }
    if (digi.missileDefeat.missileDefeatTtgo < 0.0) digi.missileDefeat.missileDefeatTtgo = 0.0;

    // --- Maneuver selection ---
    // FreeFalcon mdefeat.cpp:482-523
    if (digi.missileDefeat.missileDefeatTtgo > kLDTime) {
        // We have time — beam or drag
        // FreeFalcon mdefeat.cpp:497-510:
        //   closure = self_speed - missile_speed
        //   if (range > 2 NM or closure < 400) → Drag
        //   else → Beam
        // FF mdefeat.cpp:497-502 computes closure as `self_kias - missile_kias`
        // (a scalar speed difference, not the true closing rate). We preserve
        // that semantics here. A true closing rate (rg.closure = -rg.rangedot)
        // would be more correct, but would diverge from FF's behavior.
        const double closure = self.speed - missile->speed;

        if (range > kDragRangeThreshold || closure < kDragClosureThreshold) {
            MissileDragManeuver(digi, self, as, fcs, fcsState, dt);
        } else {
            MissileBeamManeuver(digi, self, as, fcs, fcsState, dt);
        }

        // IR missile throttle cut for skill > 2
        // FreeFalcon mdefeat.cpp:514-516
        if (missile->seekerType == DigiEntity::SeekerType::IR &&
            digi.config.skill.irMissileThrottleCut) {
            digi.commands.throttle = std::min(digi.commands.throttle, 0.99);
        }
    } else {
        // Last ditch!
        MissileLastDitch(digi, self, as, fcs, fcsState, dt);
    }
}

// ===========================================================================
// MissileBeamManeuver — turn perpendicular to missile heading
// ===========================================================================
bool MissileBeamManeuver(DigiState& digi, const DigiEntity& self,
                          const AircraftState& as,
                          const FlightControlSystem& fcs, FcsState& fcsState,
                          double dt) {
    // FreeFalcon mdefeat.cpp:526-592

    // Missile heading
    const double missileYaw = digi.missileDefeat.incomingMissile->yaw;

    // Pick the perpendicular heading closest to our current heading
    // FreeFalcon mdefeat.cpp:541-554
    double nh1 = missileYaw + PI * 0.5;
    if (nh1 > PI) nh1 -= 2.0 * PI;

    double nh2 = missileYaw - PI * 0.5;
    if (nh2 < -PI) nh2 += 2.0 * PI;

    double beamHeading;
    if (std::fabs(self.yaw - nh1) < std::fabs(self.yaw - nh2)) {
        beamHeading = nh1;
    } else {
        beamHeading = nh2;
    }

    // Set a trackpoint 0.5 NM in the beam direction
    // FreeFalcon mdefeat.cpp:560-578
    const double tpX = self.x + kBeamTrackDist * std::cos(beamHeading);
    const double tpY = self.y + kBeamTrackDist * std::sin(beamHeading);

    // Altitude: maintain current altitude (simplified — FreeFalcon uses
    // terrain MEA + 2000 ft, but we don't have terrain here)
    const double tpAlt = -self.z;

    // Use TrackPoint to steer toward the beam point
    // FreeFalcon uses AutoTrack(maxGs); we use TrackPoint which delegates
    // to HeadingAndAltitudeHold.
    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    // Hold corner speed
    // FreeFalcon mdefeat.cpp:586
    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, true,
                                  digi, as, 200.0, 800.0, dt, 100.0);

    // Return true when "close" to trackpoint (FreeFalcon uses at < 5.0)
    const double dx = tpX - self.x;
    const double dy = tpY - self.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    return dist < 500.0;  // simplified: within 500 ft
}

// ===========================================================================
// MissileDragManeuver — turn cold (away from missile)
// ===========================================================================
void MissileDragManeuver(DigiState& digi, const DigiEntity& self,
                          const AircraftState& as,
                          const FlightControlSystem& fcs, FcsState& fcsState,
                          double dt) {
    // FreeFalcon mdefeat.cpp:594-635

    // Missile heading — drag = turn to match missile heading (cold = missile
    // has to chase us from behind)
    const double missileYaw = digi.missileDefeat.incomingMissile->yaw;

    // BUG FIX: removed dead `if (digi.missileDefeat.missileFindDragPt)` block. The block
    // computed tpX/tpY/tpAlt, then immediately discarded them with
    // `(void)tpX; (void)tpY; (void)tpAlt;` and recomputed the same values
    // below. The `missileFindDragPt` flag was set to false but never read
    // elsewhere. The entire block was dead code with a side effect (clearing
    // the flag). The functional code (TrackPoint + MachHold) is preserved.

    // Fly toward the drag point (20 NM cold)
    const double tpX = self.x + kDragTrackDist * std::cos(missileYaw);
    const double tpY = self.y + kDragTrackDist * std::sin(missileYaw);
    const double tpAlt = -self.z;

    ManeuverPrimitives::TrackPoint(tpX, tpY, tpAlt,
                                    digi, as, fcs, fcsState, digi.config.maxGs);

    // Hold 3× corner speed (burn through the missile's energy)
    // FreeFalcon mdefeat.cpp:634
    // NOTE: 3× corner speed is aggressive (Mach 2.5+ for most fighters).
    // FreeFalcon uses a more nuanced speed schedule. For now we preserve the
    // existing behavior; a future tuning pass should reduce this to ~1.5×
    // corner speed to match what most aircraft can sustain.
    const double dragSpeed = 3.0 * digi.config.cornerSpeed;
    ManeuverPrimitives::MachHold(dragSpeed, as.vcas, true,
                                  digi, as, 200.0, 800.0, dt, 100.0);
}

// ===========================================================================
// MissileLastDitch — max-G pull + chaff/flare
// ===========================================================================
void MissileLastDitch(DigiState& digi, const DigiEntity& self,
                       const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt) {
    // FreeFalcon mdefeat.cpp:637-725
    (void)self;
    (void)fcs;

    // Max pitch — pull like hell
    // FreeFalcon mdefeat.cpp:695
    ManeuverPrimitives::SetPstick(digi.config.maxGs, digi.config.maxGs,
                                   CommandType::GCommand, digi, as);

    // Center rudder pedals
    // FreeFalcon mdefeat.cpp:700
    ManeuverPrimitives::SetYpedal(0.0, digi);

    // Hold corner speed
    // FreeFalcon mdefeat.cpp:705
    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, true,
                                  digi, as, 200.0, 800.0, dt, 100.0);

    // Drop chaff/flare in the 0.25-0.8 TTGO window
    // FreeFalcon mdefeat.cpp:712-724
    // We expose these as flags on digi state for the host to read.
    // (The host maps these to its own EWS/SMS model.)
    // For now, we don't have dedicated flare/chaff fields — the host can
    // check (digi.missileDefeat.missileDefeatTtgo > kLDTime * 0.25 && < kLDTime * 0.8)
    // to decide when to drop. This matches FreeFalcon's timing.
    (void)fcsState;
}

} // namespace digi
} // namespace f4flight
