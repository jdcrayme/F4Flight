// f4flight - digi/decision/flight_lead.cpp
//
// Flight lead decision-making implementation.
//
// Port of FreeFalcon's flitlead.cpp (CommandFlight) plus tactical
// decision-making: target prioritization, engage/disengage, formation
// management, and wingmen status tracking.

#include "f4flight/digi/decision/flight_lead.h"
#include "f4flight/digi/decision/decision_routines.h"  // CommandFlight
#include "f4flight/digi/steering_utils.h"  // headingError
#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// ===========================================================================
// Constants
// ===========================================================================

// Engagement range thresholds (match FreeFalcon's bvrengage.cpp).
static constexpr double kEngageRangeFt = 45.0 * 6076.0;      // 45 NM
static constexpr double kDisengageRangeFt = 90.0 * 6076.0;    // 90 NM (2x engage)
static constexpr double kWvrRangeFt = 8.0 * 6076.0;           // 8 NM

// Formation rejoin threshold — wingmen farther than this are "scattered".
static constexpr double kFormationRangeFt = 3.0 * 6076.0;     // 3 NM

// ===========================================================================
// FlightLeadDecisions — the flight lead's per-frame tactical decisions.
// ===========================================================================
void FlightLeadDecisions(DigiState& digi, const DigiEntity& self,
                          const DigiEntity* target,
                          const SensorPicture& picture, double dt) {
    (void)dt;
    (void)picture;

    // Only flight leads make lead decisions.
    // A lead is: isWing == false AND vehicleInUnit == 0.
    if (digi.formation.isWing || digi.formation.vehicleInUnit != 0) {
        return;
    }

    // --- Engage / Disengage evaluation ---
    //
    // The lead uses the SAME combat modes as wingmen — it's just another
    // fighter. The difference is that the lead also issues orders. The
    // mode resolution (resolveMode) handles the actual mode selection; here
    // we just set state that influences it.
    //
    // If we should disengage, clear the target pointer so resolveMode
    // doesn't enter offensive modes. The lead will fall back to RTB or
    // waypoint navigation.
    if (target && ShouldDisengage(digi, self, target)) {
        // Clear the target — we're breaking off.
        // The brain's wvrTarget_ / injectedTarget are cleared by the caller
        // (resolveMode) when we return Disengage from SeparateCheck. Here
        // we just note the decision via the damage state.
        digi.damage.saidRTB = true;  // signal RTB intent
    }

    // --- Target prioritization ---
    //
    // If we have a target, evaluate its priority. If the SensorPicture has
    // a better target, the lead could switch — but for now, we keep the
    // current target (the host/injection path already picked it). A future
    // enhancement can use TargetPriority() to pick the best target from
    // the SensorPicture.
    if (target && !target->isDead) {
        const double priority = TargetPriority(self, *target);
        // Store the priority for debugging/trace (future: use to pick targets).
        (void)priority;
    }

    // --- Formation management ---
    //
    // If we have no target and wingmen are scattered, order rejoin.
    // CommandFlight (called from resolveMode) handles the actual radio call.
    // Here we just set the formation state.
    if (!target || target->isDead) {
        // No target — check if wingmen need to rejoin.
        const int inForm = CountWingmenInFormation(digi, self);
        const int active = CountActiveWingmen(digi);
        if (active > 0 && inForm < active) {
            // Some wingmen are scattered — flag for rejoin.
            // CommandFlight will issue the rejoin order.
            digi.formation.wingman.actionFlags[
                static_cast<int>(WingmanAction::ExecuteManeuver)] = 0;
        }
    }
}

// ===========================================================================
// TargetPriority — score a target for engagement.
// ===========================================================================
double TargetPriority(const DigiEntity& self, const DigiEntity& target) {
    const double dx = target.x - self.x;
    const double dy = target.y - self.y;
    const double dz = target.z - self.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Range score: closer = higher (max 100 at 0 ft, 0 at 90 NM).
    double rangeScore = std::max(0.0, 100.0 * (1.0 - range / kDisengageRangeFt));

    // Aspect score: target pointing at us (nose-on) = higher threat.
    // Compute the bearing from target to self, compare with target's heading.
    const double bearingToUs = std::atan2(-dy, -dx);
    double aspectAngle = std::fabs(headingError(bearingToUs, target.yaw));
    // aspectAngle = 0 → target is nose-on to us (highest threat)
    // aspectAngle = π → target is tail-on (lowest threat)
    double aspectScore = 50.0 * (1.0 - aspectAngle / M_PI);

    // Airspeed score: slower targets are easier to kill.
    // target.speed is TAS ft/s. 600 ft/s (~350 kts) is typical fighter.
    double speedScore = std::max(0.0, 30.0 * (1.0 - target.speed / 1000.0));

    // Altitude score: co-altitude targets are in envelope.
    double altDiff = std::fabs(dz);
    double altScore = std::max(0.0, 20.0 * (1.0 - altDiff / 20000.0));

    return rangeScore + aspectScore + speedScore + altScore;
}

// ===========================================================================
// ShouldEngage — should the flight lead engage the target?
// ===========================================================================
bool ShouldEngage(const DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target) {
    // Target must be alive.
    if (target.isDead) return false;

    // Range check: within engagement range.
    const double dx = target.x - self.x;
    const double dy = target.y - self.y;
    const double range = std::sqrt(dx * dx + dy * dy);
    if (range > kEngageRangeFt) return false;

    // Fuel check: not bingo. If at bingo, RTB instead of engaging.
    if (digi.fuel.fuelLbs > 0 && digi.fuel.bingoFuelLbs > 0 &&
        digi.fuel.fuelLbs <= digi.fuel.bingoFuelLbs) {
        return false;
    }

    // Damage check: if critically damaged, don't engage.
    if (digi.damage.pctStrength < 0.3) return false;

    // Weapons check: must have at least one A/A weapon (if SMS is set).
    // If no SMS is set, we assume weapons are available (for testing).
    // (The SMS check is done by the offensive modes themselves.)

    return true;
}

// ===========================================================================
// ShouldDisengage — should the flight lead break off the engagement?
// ===========================================================================
bool ShouldDisengage(const DigiState& digi, const DigiEntity& self,
                      const DigiEntity* target) {
    // Fuel: at or below bingo → RTB.
    if (digi.fuel.fuelLbs > 0 && digi.fuel.bingoFuelLbs > 0 &&
        digi.fuel.fuelLbs <= digi.fuel.bingoFuelLbs) {
        return true;
    }

    // Damage: pctStrength < 0.5 → survive to fight another day.
    if (digi.damage.pctStrength < 0.5) return true;

    // Winchester: out of A/A weapons. The host sets winchester flag.
    if (digi.fuel.winchester) return true;

    // Target escaped: range > 2x engage range, or target is dead/null.
    if (!target || target->isDead) return true;
    const double dx = target->x - self.x;
    const double dy = target->y - self.y;
    const double range = std::sqrt(dx * dx + dy * dy);
    if (range > kDisengageRangeFt) return true;

    return false;
}

// ===========================================================================
// ShouldRejoin — should the flight lead order wingmen to rejoin?
// ===========================================================================
bool ShouldRejoin(const DigiState& digi, const DigiEntity* target) {
    // No active target.
    if (target && !target->isDead) return false;

    // No incoming missile (threat check).
    if (digi.missileDefeat.incomingMissile) return false;

    return true;
}

// ===========================================================================
// Wingmen status tracking
// ===========================================================================

int CountActiveWingmen(const DigiState& digi) {
    // The host sets the wingmen count via the formation state.
    // For now, we count based on vehicleInUnit: if this is the lead (0),
    // and isWing is false, the flight has at least 1 member (us). The
    // actual wingmen count comes from the host's FrameInputs.
    //
    // A future enhancement can use the SensorPicture to count detected
    // friendly aircraft in the flight.
    (void)digi;
    return 0;  // unknown without host injection
}

int CountWingmenInFormation(const DigiState& digi, const DigiEntity& self) {
    // Similar to CountActiveWingmen — needs host-injected wingmen entities
    // to count. For now, returns 0 (no wingmen tracked).
    (void)digi;
    (void)self;
    return 0;
}

} // namespace digi
} // namespace f4flight
