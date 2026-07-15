// f4flight - digi/decision/decision_routines.cpp
//
// Implementation of AirbaseCheck, SeparateCheck, CommandFlight.
// See decision_routines.h for the architecture rationale.

#include "f4flight/digi/decision/decision_routines.h"
#include "f4flight/digi/digi_entity.h"  // computeRelativeGeometry
#include "f4flight/digi/comms/message.h"
#include "f4flight/flight/core/constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace f4flight {
namespace digi {

// ===========================================================================
// AirbaseCheck — auto-pick nearest friendly airbase + RTB→Landing transition.
// ===========================================================================
AirbaseAction AirbaseCheck(DigiState& digi, const DigiEntity& self,
                            const FrameInputs& inputs, double simTime) {
    // No airbases available — can't do anything.
    if (!inputs.airbases || inputs.airbaseCount == 0) {
        return AirbaseAction::None;
    }

    // Throttle: only re-search for nearest airbase every kAirbaseCheckIntervalSec.
    // We store the last check time in nav.mnverTime as a side channel (it's
    // also used by wingman maneuvers, but AirbaseCheck runs in a different
    // phase so there's no conflict). A dedicated field would be cleaner, but
    // this avoids growing DigiState further.
    //
    // Actually, to avoid the conflict, we use a static-ish approach: only
    // re-search if we don't already have a divert airbase, OR if the fuel
    // state just got worse. The simpler "search once when divert is needed"
    // is what FF effectively does (it re-checks every g_nAirbaseCheck seconds
    // but the search is cheap).
    //
    // For F4Flight we search every frame the brain requests an airbase check
    // — the search is O(n) over the airbase list which is typically < 20
    // entries, so it's negligible.

    // Find the nearest airbase.
    std::size_t nearestIdx = 0;
    double nearestDistSq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < inputs.airbaseCount; ++i) {
        const auto& ab = inputs.airbases[i];
        const double dx = ab.x - self.x;
        const double dy = ab.y - self.y;
        const double distSq = dx * dx + dy * dy;
        if (distSq < nearestDistSq) {
            nearestDistSq = distSq;
            nearestIdx = i;
        }
    }

    const auto& nearest = inputs.airbases[nearestIdx];
    const double nearestDist = std::sqrt(nearestDistSq);
    const double nearestDistNm = nearestDist / 6076.0;

    // If we already have a divert airbase, check if we're close enough to land.
    if (digi.fuel.hasDivertAirbase) {
        // Check distance to the CURRENT divert airbase (not necessarily the
        // nearest — we stick with the one we picked when we started RTB).
        const double dx = digi.fuel.divertAirbaseX - self.x;
        const double dy = digi.fuel.divertAirbaseY - self.y;
        const double distToFt = std::sqrt(dx * dx + dy * dy);
        const double distToFtNm = distToFt / 6076.0;

        if (distToFtNm <= kLandingTransitionNm) {
            // Close enough — transition to Landing mode.
            // Set the ground ops runway info so runLanding can use it.
            digi.ag.groundOps.assignedRunway = nearest.id;
            digi.ag.groundOps.runwayHeading = digi.fuel.divertAirbaseHeading;
            digi.ag.groundOps.runwayThresholdX = digi.fuel.divertAirbaseX;
            digi.ag.groundOps.runwayThresholdY = digi.fuel.divertAirbaseY;
            digi.ag.groundOps.runwayAltitude = -digi.fuel.divertAirbaseZ;
            digi.ag.groundOps.phase = GroundOpsPhase::Approach;
            digi.ag.groundOps.hasLandingClearance = true;  // simplified
            return AirbaseAction::Landing;
        }
        // Still en route — keep RTB.
        return AirbaseAction::RTB;
    }

    // No divert airbase set yet. Decide whether to set one.
    // (fuelCritical is computed by the caller — AirbaseCheck here only sets
    //  the divert airbase + transitions. The caller already decided RTB is
    //  needed; we just figure out WHERE to go.)
    const bool damageCritical = digi.damage.pctStrength < kDamageAbortThreshold;

    // FF: when Bingo + no target/threat, check if distance > bingoReturnDistance.
    // If so, set divert + RTB (returnHomebase).
    const bool noTargetOrThreat =
        !digi.threat.threatPtr;  // (target check would need the offensive target ptr)

    if (digi.fuel.phase == DigiFuelState::Phase::Bingo && noTargetOrThreat
        && nearestDistNm > kBingoReturnDistanceNm) {
        // Set divert airbase + RTB.
        digi.fuel.divertAirbaseX = nearest.x;
        digi.fuel.divertAirbaseY = nearest.y;
        digi.fuel.divertAirbaseZ = nearest.z;
        digi.fuel.divertAirbaseHeading = nearest.runwayHeading;
        digi.fuel.hasDivertAirbase = true;
        digi.damage.saidRTB = true;  // mark that we've committed to RTB
        return AirbaseAction::RTB;
    }

    // FF: when Fumes OR pctStrength < 0.50, force divert to nearest + Landing.
    if (digi.fuel.phase == DigiFuelState::Phase::Fumes || damageCritical) {
        digi.fuel.divertAirbaseX = nearest.x;
        digi.fuel.divertAirbaseY = nearest.y;
        digi.fuel.divertAirbaseZ = nearest.z;
        digi.fuel.divertAirbaseHeading = nearest.runwayHeading;
        digi.fuel.hasDivertAirbase = true;
        // FF: airbasediverted = 2 → AddMode(LandingMode) directly.
        // We set the ground ops approach phase + return Landing.
        digi.ag.groundOps.assignedRunway = nearest.id;
        digi.ag.groundOps.runwayHeading = nearest.runwayHeading;
        digi.ag.groundOps.runwayThresholdX = nearest.x;
        digi.ag.groundOps.runwayThresholdY = nearest.y;
        digi.ag.groundOps.runwayAltitude = -nearest.z;
        digi.ag.groundOps.phase = GroundOpsPhase::Approach;
        digi.ag.groundOps.hasLandingClearance = true;
        return AirbaseAction::Landing;
    }

    return AirbaseAction::None;
}

// ===========================================================================
// SeparateCheck — disengage logic.
// ===========================================================================
SeparateAction SeparateCheck(DigiState& digi, const DigiEntity& self,
                              const DigiEntity* target, double dt) {
    // 1. Damage abort: pctStrength < 0.50 → RTB.
    if (digi.damage.pctStrength < kDamageAbortThreshold) {
        // Clear ground target if any (FF: SetGroundTarget(NULL)).
        digi.ag.groundTarget = nullptr;
        digi.ag.groundTargetId = kInvalidEntityId;
        digi.damage.saidRTB = true;
        return SeparateAction::RTB;
    }

    // 2. Bugout timer: target ataFrom > 135° for > 90 s → Bugout.
    if (target && !target->isDead) {
        const RelativeGeometry rg = computeRelativeGeometry(self, *target);

        if (std::fabs(rg.ataFrom) > kBugoutAtaFromThreshold) {
            // Deep six — start/continue bugout timer.
            if (!digi.damage.bugoutTimerActive) {
                digi.damage.bugoutTimer = kBugoutTimerSec;
                digi.damage.bugoutTimerActive = true;
            } else {
                digi.damage.bugoutTimer -= dt;
                if (digi.damage.bugoutTimer <= 0.0) {
                    // Deep six for > 90 s → disengage.
                    return SeparateAction::Bugout;
                }
            }
        } else {
            // Not deep six — reset timer.
            digi.damage.bugoutTimer = 0.0;
            digi.damage.bugoutTimerActive = false;
        }

        // 3. Separate: Bingo/Joker AND target within 2-6 NM AND geometry
        // says "can't be offensive". FF uses a complex RangeAtTailChase +
        // TailChaseRMaxNe calculation. We use a simplified version: if
        // Bingo AND target within kSeparateMinRange..kSeparateMaxRange,
        // enter Separate (the WvrBugOut primitive will run away).
        const bool fuelLow =
            (digi.fuel.phase == DigiFuelState::Phase::Bingo ||
             digi.fuel.phase == DigiFuelState::Phase::Fumes);
        if (fuelLow &&
            rg.range >= kSeparateMinRangeFt &&
            rg.range <= kSeparateMaxRangeFt) {
            return SeparateAction::Separate;
        }
    } else {
        // No target — reset bugout timer.
        digi.damage.bugoutTimer = 0.0;
        digi.damage.bugoutTimerActive = false;
    }

    return SeparateAction::None;
}

// ===========================================================================
// CommandFlight — flight-lead issues orders to wingmen.
// ===========================================================================
void CommandFlight(DigiState& digi, const DigiEntity* target,
                   MessageBus* bus, EntityId selfId, double simTime) {
    if (!bus) return;  // no bus — can't issue orders

    // Only leads issue orders. A lead is: isWing == false AND vehicleInUnit == 0.
    // (vehicleInUnit 0 = lead slot; 1/2/3 = wingmen.)
    if (digi.formation.isWing || digi.formation.vehicleInUnit != 0) {
        return;
    }

    // We need a flight ID to address the wingmen. The host sets comm.selfId
    // to the lead's entity ID; we use a flight-group ID derived from it
    // (convention: flight ID = lead entity ID). The host registers wingmen
    // into this group via bus->addToGroup(leadId, wingmanId).
    const EntityId flightId = digi.comm.selfId;

    // Throttle: only send orders every kCommandFlightOrderIntervalSec.
    // The last order time is stored per-brain in digi.comm.lastOrderTime
    // (Round 6: moved from a thread_local static so tests don't interfere
    // with each other and so multiple brains can coexist).
    if (simTime - digi.comm.lastOrderTime < kCommandFlightOrderIntervalSec) {
        return;  // throttled
    }

    if (target && !target->isDead) {
        // We have a target — order wingmen to engage.
        // FF: AiSendCommand(WMAssignTarget) + AiSendCommand(WMShooterMode).
        Message engageMsg{MessageType::FlightCmdEngage, selfId, kBroadcastId};
        bus->publishToGroup(flightId, engageMsg, simTime);

        Message weaponsFreeMsg{MessageType::FlightCmdWeaponsFree, selfId, kBroadcastId};
        bus->publishToGroup(flightId, weaponsFreeMsg, simTime);

        digi.comm.lastOrderTime = simTime;
    } else {
        // No target — order wingmen to rejoin.
        // FF: AiSendCommand(WMRejoin) + AiSendCommand(WMCoverMode).
        Message rejoinMsg{MessageType::FlightCmdRejoin, selfId, kBroadcastId};
        bus->publishToGroup(flightId, rejoinMsg, simTime);

        digi.comm.lastOrderTime = simTime;
    }
}

} // namespace digi
} // namespace f4flight
