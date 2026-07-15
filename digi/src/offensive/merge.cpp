// f4flight - digi/offensive/merge.cpp
//
// Port of FreeFalcon merge.cpp (304 LOC).

#include "f4flight/digi/offensive/merge.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// MergeCheck
// ===========================================================================
bool MergeCheck(const DigiState& digi, const DigiEntity& self,
                const DigiEntity& target) {
    // BUG FIX: previously stayed in Merge mode indefinitely as long as the
    // geometry matched. Now we exit when mergeTimer expires (3 seconds,
    // matching FreeFalcon merge.cpp:9-52). The timer is decremented in
    // MergeManeuver each frame.
    if (digi.weapon.mergeTimer == 0.0) return false;  // timer expired — exit merge

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Must be above 3000 ft AGL
    const double altAGL = -self.z;
    if (altAGL < 3000.0) return false;

    // Must be close (range ≤ ~1000 ft)
    if (rg.range > 1000.0) return false;

    // Must be pointing near the target
    if (rg.ata > 45.0 * DTR) return false;

    // Must not be in a steep climb/dive
    if (std::fabs(self.pitch) > 45.0 * DTR) return false;

    // Must be a nose-to-nose or stern chase geometry
    if (rg.ataFrom > 45.0 * DTR) return false;

    return true;
}

// ===========================================================================
// MergeManeuver
// ===========================================================================
void MergeManeuver(DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target, const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt) {
    (void)fcs;

    // BUG FIX: actually use the dt parameter (was previously cast away with
    // (void)dt and the code used digi.nav.dt instead — usually the same, but
    // using the parameter is clearer and supports per-call overrides).
    //
    // Decrement mergeTimer to time out the mode (FreeFalcon merge.cpp:9-52
    // uses a 3-second exit). Initialize on first entry.
    if (digi.weapon.mergeTimer < 0.0) {
        digi.weapon.mergeTimer = 3.0;  // 3 seconds in merge mode
    }
    digi.weapon.mergeTimer = std::max(0.0, digi.weapon.mergeTimer - dt);

    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // On first pass, pick a maneuver.
    // F4Flight doesn't have maneuverClassData, so we default to a
    // two-circle level turn (the most common fighter merge maneuver).
    // A real port would read maneuverClassData[combatClass].flags.
    //
    // Combat class flags (FF digi.h:153-163):
    //   CanLevelTurn = 0x1, CanSlice = 0x2, CanUseVertical = 0x4
    //   CanOneCircle = 0x10, CanTwoCircle = 0x20
    //
    // For a generic fighter (F-16 class), flags = CanLevelTurn | CanSlice |
    // CanUseVertical | CanOneCircle | CanTwoCircle. We implement the
    // combination logic from FF merge.cpp:202-243.

    const double vcas = as.vcas;
    const double corner = digi.config.cornerSpeed;

    if (vcas < corner * 0.7) {
        // Slow — slice (135° bank)
        digi.gunsJink.newRoll = (rg.az > 0.0 ? 135.0 * DTR : -135.0 * DTR);
        ManeuverPrimitives::MachHold(0.7 * corner, vcas, false,
                                      digi, as, 100.0, 400.0, digi.nav.dt, 100.0);
    } else if (vcas < corner * 1.2) {
        // Medium — level turn (90° bank)
        if (vcas < corner) {
            // One circle — turn away from target
            digi.gunsJink.newRoll = (rg.az > 0.0 ? -90.0 * DTR : 90.0 * DTR);
            ManeuverPrimitives::MachHold(0.7 * corner, vcas, false,
                                          digi, as, 100.0, 400.0, digi.nav.dt, 100.0);
        } else {
            // Two circle — turn toward target
            digi.gunsJink.newRoll = (rg.az > 0.0 ? 90.0 * DTR : -90.0 * DTR);
            ManeuverPrimitives::MachHold(corner, vcas, true,
                                          digi, as, 100.0, 400.0, digi.nav.dt, 100.0);
        }
    } else {
        // Fast — vertical pull (wings level, full burner)
        digi.gunsJink.newRoll = 0.0;
        ManeuverPrimitives::MachHold(corner * 1.2, vcas, true,
                                      digi, as, 100.0, 400.0, digi.nav.dt, 100.0);
    }

    // Command the roll + max-G pull
    // (FF merge.cpp:247-252)
    const double eDroll = digi.gunsJink.newRoll - self.roll;
    ManeuverPrimitives::SetYpedal(0.0, digi);
    ManeuverPrimitives::SetRstick(eDroll * 2.0 * RTD, digi,
                                   FlightControlSystem{}, fcsState);
    ManeuverPrimitives::SetPstick(digi.config.maxGs, digi.config.maxGs,
                                   CommandType::GCommand, digi, as);
    fcsState.maxRoll = std::fabs(digi.gunsJink.newRoll) * RTD;
    fcsState.maxRollDelta = std::fabs(eDroll) * RTD;
}

// ===========================================================================
// AccelCheck
// ===========================================================================
bool AccelCheck(const DigiState& digi, const DigiEntity& self,
                const AircraftState& as) {
    (void)self;

    // Only in combat modes (Merge..BVR)
    // (FF merge.cpp:260-261)
    // We check the active mode externally; here we just check the speed/pitch
    // condition.

    const double corner = digi.config.cornerSpeed;
    const double pitchDeg = self.pitch * RTD;

    // Pitch > 50° and very slow, OR pitch > 0° and critically slow
    if (pitchDeg > 50.0 && as.vcas < corner * 0.4)
        return true;
    if (pitchDeg > 0.0 && as.vcas < corner * 0.35)
        return true;

    return false;
}

// ===========================================================================
// AccelManeuver
// ===========================================================================
void AccelManeuver(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt) {
    (void)fcs;
    (void)dt;

    // Roll 170° (inverted) to dive and accelerate
    // (FF merge.cpp:282-292)
    const double targetRoll = (self.roll >= 0.0) ? 170.0 * DTR : -170.0 * DTR;
    const double eDroll = targetRoll - self.roll;

    ManeuverPrimitives::SetYpedal(0.0, digi);
    ManeuverPrimitives::SetRstick(eDroll * 2.0 * RTD, digi,
                                   FlightControlSystem{}, fcsState);
    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, false,
                                  digi, as, 100.0, 400.0, digi.nav.dt, 100.0);

    // If still rolling, don't pull; once inverted, pull 4G
    if (std::fabs(eDroll * RTD) > 10.0) {
        ManeuverPrimitives::SetPstick(0.0, digi.config.maxGs,
                                       CommandType::GCommand, digi, as);
    } else {
        ManeuverPrimitives::SetPstick(4.0, digi.config.maxGs,
                                       CommandType::GCommand, digi, as);
    }

    fcsState.maxRoll = 170.0;
}

} // namespace digi
} // namespace f4flight
