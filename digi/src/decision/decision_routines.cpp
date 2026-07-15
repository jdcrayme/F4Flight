// f4flight - digi/decision/decision_routines.cpp
//
// Implementation of AirbaseCheck, SeparateCheck, CommandFlight.
// See decision_routines.h for the architecture rationale.

#include "f4flight/digi/decision/decision_routines.h"
#include "f4flight/digi/digi_entity.h"  // computeRelativeGeometry
#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/sensors/sensor_picture.h"  // SensorContact, SensorPicture
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

    // Round 7 fix: when fuel is Bingo or worse, ALWAYS set the divert airbase
    // to the nearest one. The previous code only set the divert when the
    // nearest airbase was > 50 NM away (FF's bingoReturnDistance). That
    // meant a Bingo-fuel aircraft with a friendly field 20 NM away would
    // enter RTB mode but have no divert destination — runRTB would fall
    // back to waypoint nav (heading hold), and the aircraft would fly past
    // the airbase without turning toward it.
    //
    // FF's 50 NM threshold is for deciding whether to RTB at all (if the
    // nearest field is close, keep fighting). But once the brain has decided
    // to RTB (fuel <= bingo), it should ALWAYS head to the nearest field.
    // The caller (resolveMode) already made the RTB decision based on fuel;
    // AirbaseCheck just picks the destination.
    const bool fuelBingoOrWorse =
        (digi.fuel.phase == DigiFuelState::Phase::Bingo ||
         digi.fuel.phase == DigiFuelState::Phase::Fumes);

    // FF: when Fumes OR pctStrength < 0.50, force divert + direct Landing.
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

    // Bingo (not Fumes, not damaged): set divert + RTB. The aircraft navigates
    // to the airbase but doesn't start the approach until within 10 NM
    // (the kLandingTransitionNm check at the top of this function handles
    // that transition on subsequent frames).
    if (fuelBingoOrWorse) {
        digi.fuel.divertAirbaseX = nearest.x;
        digi.fuel.divertAirbaseY = nearest.y;
        digi.fuel.divertAirbaseZ = nearest.z;
        digi.fuel.divertAirbaseHeading = nearest.runwayHeading;
        digi.fuel.hasDivertAirbase = true;
        digi.damage.saidRTB = true;
        return AirbaseAction::RTB;
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

// ===========================================================================
// Round 7 (P1): ChooseRadarMode — AI radar mode management.
// Port of FF bvrengage.cpp:3173-3238.
// ===========================================================================
void ChooseRadarMode(DigiState& digi, double simTime, bool hasTWS) {
    // Throttle: re-evaluate every (4 + (4 - skill)) seconds.
    const double throttle = radarModeThrottleSec(
        static_cast<int>(digi.config.skill.level));
    if (simTime - digi.weapon.lastRadarModeTime < throttle) {
        return;  // not time to re-evaluate
    }
    digi.weapon.lastRadarModeTime = simTime;

    // Map radModeSelect → radarMode.
    // FF: switch (radModeSelect) { 0=STT, 1=SAM, 2=TWS, 3=RWS, 4=OFF }
    // Default (any unknown value) = RWS.
    int newMode = static_cast<int>(RadarMode::RWS);  // default
    switch (digi.weapon.radModeSelect) {
        case 0: newMode = static_cast<int>(RadarMode::STT); break;
        case 1: newMode = static_cast<int>(RadarMode::SAM); break;
        case 2:
            // TWS — only if the radar supports it; else fall back to RWS.
            newMode = hasTWS ? static_cast<int>(RadarMode::TWS)
                              : static_cast<int>(RadarMode::RWS);
            break;
        case 3: newMode = static_cast<int>(RadarMode::RWS); break;
        case 4: newMode = static_cast<int>(RadarMode::OFF); break;
        default: newMode = static_cast<int>(RadarMode::RWS); break;
    }

    // FF special case: if a SARH missile (not AIM-120/ARH) is in flight,
    // force STT to maintain the guidance link. We don't currently model
    // SARH vs ARH in the F4Flight missile entity — the host would need to
    // flag this. For now, this is a no-op (the condition is never true
    // because we don't track "curMissile" on DigiState). A future port
    // can add a `guidingMissile` field and check it here.
    // if (digi.weapon.guidingMissileIsSARH) newMode = STT;

    digi.weapon.radarMode = newMode;
}

// ===========================================================================
// Round 7 (P1): ApplyGCI — skill-gated GCI detection.
// Port of FF sfusion.cpp:67-186.
// ===========================================================================
void ApplyGCI(const DigiState& digi, SensorPicture& pic, const DigiEntity& self) {
    if (!digi.config.skill.gciCapable) return;

    // FF: GCI range is 30 NM for veterans/aces.
    constexpr double kGCIMaxRangeFt = 30.0 * 6076.0;

    for (auto& contact : pic.contacts) {
        // Skip missiles and non-aircraft.
        if (contact.isMissile) continue;

        // Compute range from self to contact.
        const double dx = contact.x - self.x;
        const double dy = contact.y - self.y;
        const double dz = contact.z - self.z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (range > kGCIMaxRangeFt) continue;

        // GCI detection: if the contact isn't already detected by a sensor,
        // mark it as Detected (quality = Detected, sensorMask |= GCI).
        // This lets veteran/ace AI "know" about contacts beyond sensor range.
        if (contact.quality == ContactQuality::None) {
            contact.quality = ContactQuality::Detected;
            contact.addSensor(SensorType::GCI);
            contact.confidence = std::max(contact.confidence, 0.3);
        }
    }
}

// ===========================================================================
// Round 7 (P1): ApplyNCTR — radar-based NCTR identification.
// Port of FF sfusion.cpp:209-215.
// ===========================================================================
void ApplyNCTR(const DigiState& digi, SensorPicture& pic, const DigiEntity& self,
               bool hasNCTR, double maxNctrRangeFt) {
    if (!hasNCTR) return;

    // FF: NCTR range is scaled by skill:
    //   effectiveRange = maxNctrRange / (2.0 * (16 - skill) / 16.0)
    // Higher skill = longer effective range (skill=4 → /1.5, skill=0 → /2.0).
    const int skill = static_cast<int>(digi.config.skill.level);
    const double skillScale = 2.0 * (16 - skill) / 16.0;
    const double effectiveRange = maxNctrRangeFt / std::max(0.5, skillScale);

    for (auto& contact : pic.contacts) {
        if (contact.isMissile) continue;
        if (contact.quality >= ContactQuality::Identified) continue;  // already ID'd

        // Compute relative geometry.
        DigiEntity contactEntity;
        contactEntity.x = contact.x; contactEntity.y = contact.y; contactEntity.z = contact.z;
        contactEntity.vx = contact.vx; contactEntity.vy = contact.vy; contactEntity.vz = contact.vz;
        contactEntity.yaw = contact.yaw; contactEntity.pitch = contact.pitch; contactEntity.roll = contact.roll;
        const RelativeGeometry rg = computeRelativeGeometry(self, contactEntity);

        // FF: ataFrom < 45° (target's nose toward us) AND range < effectiveRange.
        if (std::fabs(rg.ataFrom) > 45.0 * DTR) continue;
        if (rg.range > effectiveRange) continue;

        // NCTR succeeded — upgrade the contact to Identified.
        // FF uses the radar's NCTR database to determine the exact type.
        // We don't have that database; we mark the contact as Identified
        // and let the host (or a future NCTR database port) set the type.
        contact.quality = ContactQuality::Identified;
        contact.addSensor(SensorType::Radar);
        contact.confidence = std::max(contact.confidence, 0.8);

        // If the contact type is Unknown, guess Fighter (the most common
        // threat type). A future NCTR database port can replace this with
        // actual type identification based on radar signature.
        if (contact.type == ContactType::Unknown) {
            contact.type = ContactType::Fighter;
        }
    }
}

// ===========================================================================
// Round 7 (P1): DoTargeting — autonomous target selection.
// Port of FF targeting.cpp:DoTargeting + TargetSelection.
// ===========================================================================
const DigiEntity* DoTargeting(DigiState& digi, const SensorPicture& pic,
                               const DigiEntity& self) {
    // If the brain already has an injected target, keep it.
    // (The caller — DigiBrain::resolveMode — checks frameInputs_.injectedTarget
    // before calling DoTargeting. If we get here, there's no injected target.)

    // Scan the SensorPicture for the highest-threat aircraft contact.
    // FF uses a complex threat-scoring algorithm (range, ATA, type, aspect).
    // We use the pre-computed threatScore from SensorFusion.
    const SensorContact* best = nullptr;
    double bestScore = -1.0;

    for (const auto& contact : pic.contacts) {
        // Skip missiles (those are handled by MissileDefeat, not targeting).
        if (contact.isMissile) continue;
        // Skip contacts that aren't at least Detected.
        if (contact.quality < ContactQuality::Detected) continue;
        // Skip dead contacts.
        // (SensorContact doesn't have isDead; the host removes dead contacts
        // from the truth, so they age out of the picture naturally.)

        // Use the pre-computed threatScore. Higher = better target.
        if (contact.threatScore > bestScore) {
            bestScore = contact.threatScore;
            best = &contact;
        }
    }

    if (!best) return nullptr;

    // Convert the best contact to a DigiEntity and return it.
    // The caller (resolveMode) will use this as wvrTarget_.
    // We store the converted entity in a static thread_local to give it
    // a stable address for the frame.
    //
    // NOTE: This is a simplification — the entity is recomputed each frame.
    // A production version would cache the entity by ID and only refresh
    // position/velocity, preserving identity across frames.
    thread_local DigiEntity resolvedTarget;
    resolvedTarget = toDigiEntity(*best);
    return &resolvedTarget;
}

} // namespace digi
} // namespace f4flight
