// f4flight - digi/wingman/wingman_ai.h
//
// Wingman AI — formation following and wingman maneuver execution.
//
// Port of FreeFalcon's AiFollowLead (wingactions.cpp:295-420) and the
// AiPerformManeuver dispatcher (wingactions.cpp:1-100).
//
// The wingman AI is activated when:
//   - DigiState.formation.isWing == true (host set this aircraft as a wingman)
//   - DigiState.formation.flightLeadId != kInvalidEntityId (lead is assigned)
//   - The lead entity is provided via FrameInputs.injectedLead (or via
//     SensorFusion if the lead is a detectable entity)
//
// The wingman AI computes a desired formation slot position relative to the
// lead, then uses HeadingAndAltitudeHold + MachHold to fly to that position.
// This is the core "fly in formation" behavior — the prerequisite for all
// flight-lead gameplay (wingmen following the player, AI package escort, etc.)
//
// Source mapping (FreeFalcon sim/digi/):
//   AiFollowLead    <- wingactions.cpp:295-420
//   AiCheckPosition <- wingactions.cpp:430-470 (simplified — radio call only)

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/digi/formation/formation_geometry.h"

namespace f4flight {
namespace digi {

// AiFollowLead — fly to the wingman's formation slot position relative to the lead.
//
// This is the core wingman behavior. It:
//   1. Looks up the formation slot geometry (relAz, relEl, range) from
//      FormationTable using formation.formationId + formation.vehicleInUnit
//   2. Applies the lateral spacing factor (wingman.formLateralSpaceFactor)
//      and side mirror (wingman.formSide)
//   3. Computes the desired world position: leadPos + offset rotated by
//      lead's velocity heading (sigma)
//   4. Adds a 1 NM lead-ahead offset (so the wingman flies toward where
//      the lead will be, not where it was)
//   5. Applies relative altitude (wingman.formRelativeAltitude)
//   6. Calls HeadingAndAltitudeHold + MachHold to fly to the desired position
//
// If no lead is available (nullptr), falls back to Loiter (orbit in place).
//
// Returns true if "in position" (within 250 ft of the desired slot), false
// otherwise. The caller can use this for radio calls ("in position").
bool AiFollowLead(DigiState& digi, const DigiEntity& self,
                  const DigiEntity* lead, const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt);

// AiCheckInPositionCall — check if the wingman has arrived at the formation
// slot and set the inPosition flag. FreeFalcon uses this to trigger the
// "in position" radio call. Returns true if newly in position this frame.
bool AiCheckInPositionCall(DigiState& digi, const DigiEntity& self,
                           double trackX, double trackY, double trackZ);

} // namespace digi
} // namespace f4flight
