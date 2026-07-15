// f4flight - digi/wingman/wingman_maneuvers.h
//
// Wingman tactical maneuvers — port of FreeFalcon's AiPerformManeuver
// (wingactions.cpp:30-124) and the AiExec* / AiInit* functions.
//
// When the brain is in FollowOrders mode and WingmanState.currentManeuver
// is set (by receiveOrders() in response to a FlightCmdBreak / FlightCmdClearSix
// etc. message), AiPerformManeuver dispatches to the appropriate AiExec*
// function. Each AiExec* function flies a specific tactical maneuver and
// returns true while the maneuver is still active; when it returns false,
// the caller clears the maneuver and the brain falls back to Wingy.
//
// Ported maneuvers (6 of FF's 6 — Round 6 completes the set):
//   AiExecBreakRL    — break turn: track 1000 ft ahead at ordered heading,
//                       hold for mnverTime seconds, then clear.
//   AiExecClearSix   — 180° turn to check six: same math as BreakRL.
//   AiExecPosthole   — descend to ordered altitude, then engage target.
//   AiExecChainsaw   — missile-only engage.
//   AiExecPince      — 2-point bracket maneuver: fly to point 1, then point 2,
//                       then clear. Points set by AiInitPince.
//   AiExecFlex       — 3-point S-curve maneuver: fly to points 1→2→3, then
//                       clear. Points set by AiInitFlex.
//
// Round 6 changes:
//   - Added AiInitPince / AiInitFlex (maneuver-point setup)
//   - Added AiExecPince / AiExecFlex (multi-point TrackPoint following)
//   - Changed AiExecPosthole / AiExecChainsaw signatures to accept an
//     optional target + SMS so they can actually engage (previously stubs).
//   - AiPerformManeuver + AiExec* now accept a target + sms pointer pair
//     so the engage-style maneuvers (Posthole, Chainsaw) can fire weapons.
//
// Source mapping (FreeFalcon sim/digi/):
//   AiPerformManeuver <- wingactions.cpp:30-124
//   AiExecBreakRL     <- wingactions.cpp:440-480
//   AiExecClearSix    <- wingactions.cpp:611-633
//   AiExecPosthole    <- wingactions.cpp:487-517
//   AiExecChainsaw    <- wingactions.cpp:522-534
//   AiExecPince       <- wingactions.cpp:540-569
//   AiExecFlex        <- wingactions.cpp:575-601
//   AiInitPince       <- wingai.cpp:745-844
//   AiInitFlex        <- wingai.cpp:857-884
//   AiClearManeuver   <- wingai.cpp (AiClearManeuver / AiSetManeuver(None))

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// Forward declarations
class StoresManagementSystem;

// Default maneuver duration when a break/clear-six maneuver is initiated.
constexpr double kDefaultManeuverTimeSec = 3.0;

// Pince/Flex maneuver-point arrival thresholds (from FF wingactions.cpp).
// Pince uses 5000 ft (S.G. comment: "1000 feet is too short").
// Flex uses 900 ft.
constexpr double kPinceArrivalThresholdFt = 5000.0;
constexpr double kFlexArrivalThresholdFt  = 900.0;

// AiPerformManeuver — dispatch on WingmanState.currentManeuver to the
// appropriate AiExec* function.
//
//   digi   : AI state
//   self   : own aircraft entity
//   target : the offensive target (may be null — engage-style maneuvers
//            will rejoin if null). The brain resolves this from
//            WingmanState.designatedTargetId or wvrTarget_.
//   sms    : weapon stores (may be null — Chainsaw rejoin if null/no missiles)
//   as     : aircraft state
//   fcs    : flight control system
//   fcsState : FCS state
//   dt     : frame time
//
// Returns true if a maneuver is active (the brain should skip its normal
// per-mode switch). Returns false if no maneuver is active OR the maneuver
// just completed this frame (the caller may then fall back to Wingy).
bool AiPerformManeuver(DigiState& digi, const DigiEntity& self,
                       const DigiEntity* target,
                       const StoresManagementSystem* sms,
                       const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt);

// --- Break / ClearSix (timer-based maneuvers) ---

bool AiExecBreakRL(DigiState& digi, const DigiEntity& self,
                   const AircraftState& as,
                   const FlightControlSystem& fcs, FcsState& fcsState,
                   double dt);

bool AiExecClearSix(DigiState& digi, const DigiEntity& self,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt);

// --- Posthole / Chainsaw (engage-style maneuvers — need target+sms) ---
// Round 6: these now accept target + sms so they can actually engage.
// If target is null, both maneuvers rejoin (clear + fall back to formation).
// If sms is null or has no missiles, Chainsaw rejoin; Posthole falls back
// to GunsEngage (which doesn't need sms — it uses the default gun spec).

bool AiExecPosthole(DigiState& digi, const DigiEntity& self,
                    const DigiEntity* target,
                    const StoresManagementSystem* sms,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt);

bool AiExecChainsaw(DigiState& digi, const DigiEntity& self,
                    const DigiEntity* target,
                    const StoresManagementSystem* sms,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState,
                    double dt);

// --- Pince / Flex (multi-point maneuvers) ---
// Round 6 additions. These fly through a series of pre-computed trackpoints
// (stored in digi.formation.maneuverPoints[]). The points are set by
// AiInitPince / AiInitFlex when the maneuver is initiated.

// AiInitPince — set up the 2 Pince maneuver points.
// Pince is a bracket maneuver: the wingman flies to a point offset laterally
// from the target/lead bearing, then to a second point further along that
// bearing, then clears. The lateral offset is mirrored by slot index parity
// (odd slots go right, even slots go left) so a 4-ship splits into 2 pairs.
//
//   digi   : AI state (writes formation.maneverPoints[0..1], wingman.*)
//   self   : own aircraft entity (maneuver origin)
//   target : the target (used for bearing). If null, uses lead's yaw; if no
//            lead, uses self's yaw.
//   lead   : the flight lead entity (used if target is null). May be null.
void AiInitPince(DigiState& digi, const DigiEntity& self,
                 const DigiEntity* target, const DigiEntity* lead);

// AiInitFlex — set up the 3 Flex maneuver points.
// Flex is an S-curve: the wingman flies to a point 1 NM to one side, then
// 2 NM to the other side, then 2.1 NM further, then clears. The maneuver
// axis is derived from the target/lead/self bearing (same as Pince).
void AiInitFlex(DigiState& digi, const DigiEntity& self,
                const DigiEntity* target, const DigiEntity* lead);

// AiExecPince — fly through the 2 Pince points, then clear.
// Returns true while maneuverPointCounter < 2; false when complete.
bool AiExecPince(DigiState& digi, const DigiEntity& self,
                 const AircraftState& as,
                 const FlightControlSystem& fcs, FcsState& fcsState,
                 double dt);

// AiExecFlex — fly through the 3 Flex points, then clear.
// Returns true while maneuverPointCounter < 3; false when complete.
bool AiExecFlex(DigiState& digi, const DigiEntity& self,
                const AircraftState& as,
                const FlightControlSystem& fcs, FcsState& fcsState,
                double dt);

// AiClearManeuver — clear the current wingman maneuver and reset state.
void AiClearManeuver(DigiState& digi);

} // namespace digi
} // namespace f4flight
