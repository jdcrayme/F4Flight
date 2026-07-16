// f4flight - digi/decision/flight_lead.h
//
// Flight lead decision-making logic.
//
// Port of FreeFalcon's flitlead.cpp (CommandFlight) plus the tactical
// decision-making that a flight lead performs: target prioritization,
// engage/disengage decisions, formation management, and wingmen status
// tracking.
//
// FreeFalcon's flitlead.cpp is only 90 lines (just CommandFlight — the
// "send orders to wingmen" function). The actual tactical decisions are
// scattered across dlogic.cpp (RunDecisionRoutines), targeting.cpp
// (target selection), and bvrengage.cpp (engagement geometry). This file
// consolidates the flight-lead-specific logic into one place.
//
// DESIGN:
//   - FlightLeadDecisions() is called once per frame from resolveMode(),
//     BEFORE the offensive/defensive checks. It sets state that the rest
//     of resolveMode() uses (e.g. "should we engage?", "is the flight
//     scattered?").
//   - The lead uses the SAME combat modes as wingmen (BVREngage, WVREngage,
//     etc.) — it's just another fighter. The difference is that the lead
//     also issues orders to wingmen via CommandFlight().
//   - The lead tracks wingmen status (alive, in formation, engaging) via
//     the SensorPicture or host-injected wingmen entities.

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/sensors/sensor_picture.h"  // SensorPicture

namespace f4flight {
namespace digi {

// ===========================================================================
// Flight lead decision functions
// ===========================================================================

// FlightLeadDecisions — the flight lead's per-frame tactical decisions.
//
// This is the entry point for flight-lead-specific logic. It:
//   1. Evaluates whether to engage or disengage a target
//   2. Prioritizes targets (nearest, most threatening, highest value)
//   3. Manages the formation (rejoin scattered wingmen, change formation)
//   4. Sets state that resolveMode() uses for mode arbitration
//
// Only runs if this aircraft is a flight lead (isWing == false &&
// vehicleInUnit == 0). Wingmen skip this entirely — they follow orders.
//
// Port of FreeFalcon's RunDecisionRoutines lead-specific parts
// (dlogic.cpp:200-400) + flitlead.cpp:10-90.
void FlightLeadDecisions(DigiState& digi, const DigiEntity& self,
                          const DigiEntity* target,
                          const SensorPicture& picture, double dt);

// ===========================================================================
// Target prioritization
// ===========================================================================

// TargetPriority — score a target for the flight lead's engagement.
//
// Returns a priority score (higher = more important). Factors:
//   - Range: closer targets are higher priority (easier to engage)
//   - Aspect: targets pointing at us are higher priority (threat)
//   - Airspeed: slower targets are higher priority (easier kill)
//   - Altitude: co-altitude targets are higher priority (in envelope)
//
// The lead uses this to pick which target to engage and which to assign
// to wingmen. Port of FreeFalcon's targeting.cpp target scoring logic.
double TargetPriority(const DigiEntity& self, const DigiEntity& target);

// ===========================================================================
// Engage / disengage decisions
// ===========================================================================

// ShouldEngage — should the flight lead engage the target?
//
// Returns true if the target is within engagement range and the lead has
// a valid weapon. Considers:
//   - Range: within max weapon range * 1.3 (BVR) or within WVR range
//   - Weapons: has at least one A/A weapon
//   - Fuel: not bingo (bingo → RTB instead of engaging)
//   - Damage: not critically damaged (damaged → disengage)
bool ShouldEngage(const DigiState& digi, const DigiEntity& self,
                   const DigiEntity& target);

// ShouldDisengage — should the flight lead break off the engagement?
//
// Returns true if the lead should disengage. Factors:
//   - Fuel: at or below bingo (RTB)
//   - Damage: pctStrength < 0.5 (survive to fight another day)
//   - Winchester: out of A/A weapons
//   - Target: target is dead or escaped (range > 2x max weapon range)
bool ShouldDisengage(const DigiState& digi, const DigiEntity& self,
                      const DigiEntity* target);

// ===========================================================================
// Formation management
// ===========================================================================

// ShouldRejoin — should the flight lead order wingmen to rejoin?
//
// Returns true if the wingmen should rejoin formation. Factors:
//   - No active target (target is null or dead)
//   - No threat (missile incoming)
//   - Wingmen are not in formation (scattered)
bool ShouldRejoin(const DigiState& digi, const DigiEntity* target);

// ===========================================================================
// Wingmen status tracking
// ===========================================================================

// WingmanStatus — summary of a wingman's current state.
struct WingmanStatus {
    bool isValid{false};      // is this slot occupied?
    bool isAlive{true};        // not dead/destroyed
    bool inFormation{false};   // within formation range of the lead
    bool isEngaging{false};    // currently in an offensive mode
    double rangeFt{0.0};       // range from lead
};

// CountActiveWingmen — count how many wingmen are alive and in the flight.
// Uses the SensorPicture or host-injected wingmen entities.
int CountActiveWingmen(const DigiState& digi);

// CountWingmenInFormation — count how many wingmen are in formation range.
int CountWingmenInFormation(const DigiState& digi, const DigiEntity& self);

} // namespace digi
} // namespace f4flight
