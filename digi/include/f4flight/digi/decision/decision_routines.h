// f4flight - digi/decision/decision_routines.h
//
// Decision routines — port of FreeFalcon's RunDecisionRoutines helpers
// that don't fit cleanly into the per-mode maneuver files.
//
// These are called from DigiBrain::resolveMode() to decide whether to
// enter specific modes. Each function returns true if the corresponding
// mode should be entered (the caller calls addMode()).
//
// Round 6 additions:
//   - AirbaseCheck    : auto-pick nearest friendly airbase when fuel-critical,
//                       transition RTB → Landing when within range.
//   - SeparateCheck   : disengage logic (fuel/damage/mission abort + bugout).
//   - CommandFlight   : flight-lead issues orders to wingmen via MessageBus.
//
// Round 7 (P1) additions:
//   - ChooseRadarMode : AI radar mode management (RWS/TWS/SAM/STT/OFF).
//   - ApplyGCI        : skill-gated GCI detection (veteran/ace get GCI spotting).
//   - ApplyNCTR       : radar-based NCTR identification (close-range type ID).
//   - DoTargeting     : autonomous target selection from SensorPicture.
// Source mapping (FreeFalcon sim/digi/):
//   AirbaseCheck  <- actions.cpp:489-617
//   SeparateCheck <- separate.cpp:24-253
//   FuelCheck     <- separate.cpp:393-471 (already inlined into resolveMode
//                    in Round 5; the fuel phase transition lives there)
//   CommandFlight <- flitlead.cpp:10-90

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/digi_brain.h"  // for FrameInputs::AirbaseInfo
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/sensors/sensor_picture.h"  // Round 7: ApplyGCI/NCTR/DoTargeting
#include "f4flight/flight/aircraft_state.h"

#include <cstddef>

namespace f4flight {
namespace digi {

// ===========================================================================
// AirbaseCheck — auto-pick nearest friendly airbase + RTB→Landing transition.
// Port of FF actions.cpp:489-617.
//
// Behavior:
//   1. If already diverting (hasDivertAirbase), keep RTB/Landing mode active.
//   2. When Bingo + no target/threat: check distance to nearest airbase.
//      If distance > bingoReturnDistanceNm, set divert airbase + RTB.
//   3. When Fumes OR pctStrength < 0.50: force divert to nearest airbase +
//      transition to Landing mode (the aircraft is in trouble — land now).
//   4. When within landingTransitionNm of the divert airbase AND already in
//      RTB mode: transition to Landing mode (the approach phase begins).
//
// The host provides the airbase list via FrameInputs.airbases. If null/empty,
// AirbaseCheck is a no-op (the brain falls back to waypoint-based RTB).
//
// Returns true if RTB or Landing mode should be entered this frame.
// ===========================================================================

constexpr double kAirbaseCheckIntervalSec = 5.0;     // re-check every 5 s (FF g_nAirbaseCheck)
constexpr double kBingoReturnDistanceNm   = 50.0;    // FF g_fBingoReturnDistance
constexpr double kLandingTransitionNm     = 10.0;    // transition RTB→Landing within 10 NM

// AirbaseCheck — pick the nearest friendly airbase + set divert fields.
// Called from resolveMode() each frame (the internal re-check throttle
// limits the nearest-airbase search to once per kAirbaseCheckIntervalSec).
//
//   digi   : AI state (reads fuel, damage; writes fuel.divert*)
//   self   : own aircraft entity
//   inputs : frame inputs (airbase list)
//   simTime: current sim time (for throttle)
//
// Returns the recommended mode: RTB (divert set, navigate), Landing (close
// enough to land), or None (no airbase action needed).
enum class AirbaseAction : int {
    None    = 0,   // no airbase action
    RTB     = 1,   // divert airbase set, navigate to it
    Landing = 2,   // within landing transition range, begin approach
};

AirbaseAction AirbaseCheck(DigiState& digi, const DigiEntity& self,
                            const FrameInputs& inputs, double simTime);

// ===========================================================================
// SeparateCheck — disengage logic.
// Port of FF separate.cpp:24-253.
//
// Behavior:
//   1. Damage abort: if pctStrength < 0.50, enter RTB (and clear ground target).
//   2. Bugout timer: if target ataFrom > 135° for > 90 seconds, enter Bugout.
//   3. Separate (lateral separation from a too-close target): if Bingo/Joker
//      AND target within 2-6 NM AND geometry says "can't be offensive", enter
//      Separate mode.
//
// Returns true if any disengage mode (RTB/Separate/Bugout) should be entered.
// The caller calls addMode() with the appropriate mode.
// ===========================================================================

constexpr double kBugoutTimerSec        = 90.0;     // FF: 90000 ms
constexpr double kBugoutAtaFromThreshold = 135.0 * DTR;  // FF: 135°
constexpr double kDamageAbortThreshold   = 0.50;    // FF: pctStrength < 0.50
constexpr double kSeparateMinRangeFt     = 2.0 * 6076.0;   // 2 NM
constexpr double kSeparateMaxRangeFt     = 6.0 * 6076.0;   // 6 NM

// SeparateCheck — evaluate disengage conditions.
// Returns the recommended disengage mode (None / RTB / Separate / Bugout).
//
//   digi   : AI state (reads fuel.phase, damage.pctStrength, weapon fields)
//   self   : own aircraft entity
//   target : the offensive target (may be null — bugout check needs it)
//   dt     : frame time (for bugout timer countdown)
enum class SeparateAction : int {
    None     = 0,
    RTB      = 1,   // damage/fuel abort → RTB
    Separate = 2,   // lateral separation from too-close target
    Bugout   = 3,   // deep-six for > 90 s → disengage
};

SeparateAction SeparateCheck(DigiState& digi, const DigiEntity& self,
                              const DigiEntity* target, double dt);

// ===========================================================================
// CommandFlight — flight-lead issues orders to wingmen.
// Port of FF flitlead.cpp:10-90.
//
// Behavior:
//   1. If this aircraft is a lead (formation.isWing == false) AND has wingmen
//      (formation.vehicleInUnit == 0, the lead slot):
//      a. If we have a target AND haven't sent an order recently (5 s throttle):
//         send FlightCmdEngage + FlightCmdWeaponsFree to the flight.
//      b. If we lost the target AND no wingman is still engaging:
//         send FlightCmdRejoin to the flight.
//
// The orders are published via the MessageBus. The host routes them to the
// wingmen's mailboxes. Wingmen process them in ProcessATCMessages →
// receiveOrders.
//
// This function requires a MessageBus. If the brain has no bus set
// (setMessageBus never called), CommandFlight is a no-op.
//
//   digi    : AI state (reads formation.*, target info)
//   target  : the lead's offensive target (may be null)
//   bus     : the message bus (for publishing orders). May be null.
//   selfId  : this aircraft's entity ID (the sender)
//   simTime : current sim time (for throttle)
// ===========================================================================

constexpr double kCommandFlightOrderIntervalSec = 5.0;  // FF: 5000 ms

void CommandFlight(DigiState& digi, const DigiEntity* target,
                   MessageBus* bus, EntityId selfId, double simTime);

// ===========================================================================
// Round 7 (P1): chooseRadarMode — AI radar mode management.
// Port of FF bvrengage.cpp:3173-3238.
//
// Maps digi.weapon.radModeSelect to a RadarMode and applies it to the
// RadarSensor. The offensive modes (BvrEngage, MissileEngage, WvrEngage)
// set radModeSelect; chooseRadarMode translates it to the actual mode.
//
// RadarMode enum:
//   STT  — Single Target Track (hard lock, for missile launch)
//   SAM  — Situation Awareness Mode (intermediate between TWS and STT)
//   TWS  — Track While Scan (track multiple targets)
//   RWS  — Range While Search (default search mode)
//   OFF  — Radar standby/off
//
// Special case: if a SARH missile (not AIM-120/ARH) is in flight, force STT
// to maintain the guidance link.
// ===========================================================================

enum class RadarMode : int {
    STT = 0,   // Single Target Track
    SAM = 1,   // Situation Awareness Mode
    TWS = 2,   // Track While Scan
    RWS = 3,   // Range While Search (default)
    OFF = 4,   // Radar off/standby
};

// Throttle: re-evaluate radar mode every (4 + (4 - skill)) seconds.
// Higher skill = more frequent updates.
inline double radarModeThrottleSec(int skillLevel) {
    return 4.0 + (4 - skillLevel);
}

// ChooseRadarMode — translate radModeSelect → RadarMode and apply.
//   digi      : AI state (reads weapon.radModeSelect, writes weapon.radarMode)
//   simTime   : current sim time (for throttle)
//   hasTWS    : does this aircraft's radar support TWS? (host provides)
void ChooseRadarMode(DigiState& digi, double simTime, bool hasTWS);

// ===========================================================================
// Round 7 (P1): ApplyGCI — skill-gated GCI detection.
// Port of FF sfusion.cpp:67-186.
//
// GCI (Ground Control Intercept) lets veteran/ace AI detect contacts beyond
// sensor range, simulating GCI radar vectors from a ground station. Recruits
// and rookies don't get GCI.
//
// Behavior:
//   - If skill.gciCapable AND contact range < 30 NM: mark the contact as
//     detected (quality = Detected) even if no sensor sees it.
//   - This is applied per-contact in the SensorPicture.
//
//   digi   : AI state (reads config.skill)
//   pic    : the sensor picture (modified — contacts may be upgraded)
//   self   : own aircraft entity
void ApplyGCI(const DigiState& digi, SensorPicture& pic, const DigiEntity& self);

// ===========================================================================
// Round 7 (P1): ApplyNCTR — radar-based NCTR identification.
// Port of FF sfusion.cpp:209-215.
//
// NCTR (Non-Cooperative Target Recognition) identifies the TYPE of a radar
// contact at close range. It requires:
//   - The radar supports NCTR (host provides via hasNCTR flag)
//   - ataFrom < 45° (target's nose is toward us — NCTR needs a face-on view)
//   - range < maxNctrRange (scaled by skill: higher skill = longer NCTR range)
//
// When NCTR succeeds, the contact's type is upgraded from Unknown to the
// identified type (Fighter/Bomber/etc.) and quality is set to Identified.
//
//   digi   : AI state (reads config.skill)
//   pic    : the sensor picture (modified — contacts may be identified)
//   self   : own aircraft entity
//   hasNCTR: does this aircraft's radar support NCTR? (host provides)
//   maxNctrRangeFt: max NCTR range at skill=0 (scaled by skill internally)
void ApplyNCTR(const DigiState& digi, SensorPicture& pic, const DigiEntity& self,
               bool hasNCTR, double maxNctrRangeFt);

// ===========================================================================
// Round 7 (P1): DoTargeting — autonomous target selection.
// Port of FF targeting.cpp:DoTargeting + TargetSelection.
//
// Without this, the brain relies on injected targets (FrameInputs.injectedTarget).
// DoTargeting lets the brain find its own target from the SensorPicture.
//
// Behavior:
//   - If the brain already has an injected target, keep it (DoTargeting is
//     a no-op when the host provides a target).
//   - Otherwise, scan the SensorPicture for the highest-threat contact that
//     is an aircraft (not a missile, not ground) and set it as wvrTarget_.
//   - The host reads the resolved target via brain.state() or the return value.
//
//   digi   : AI state (reads SensorPicture via the brain, writes wvrTarget_)
//   pic    : the sensor picture
//   self   : own aircraft entity
//
// Returns the selected target (nullptr if no suitable target found).
const DigiEntity* DoTargeting(DigiState& digi, const SensorPicture& pic,
                               const DigiEntity& self);

} // namespace digi
} // namespace f4flight
