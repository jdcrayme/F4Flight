// f4flight - digi/defensive/handle_threat.h
//
// HandleThreat — react to a "secondary" threat that is NOT the primary
// offensive target, NOT an incoming missile (MissileDefeat) and NOT a guns
// threat (GunsJink). The threat is presumed to have been notified to the AI
// either through a radar spike, a missile-launch call, or a SAM launch call
// from a wingman / RWR.
//
// Direct port of FreeFalcon sim/digi/handlethreat.cpp (54 LOC). This is the
// smallest piece of the digi AI that was previously entirely MISSING from
// F4Flight — DigiState had no `threatPtr` field and there was no code path
// to "engage the secondary threat". The brain only reacted to:
//   - incomingMissile  → MissileDefeat
//   - gunsThreat        → GunsJink
//   - injectedTarget    → offensive engage (WVREngage / BVR / etc.)
//
// A real combat AI also needs to react to "I'm spiked by a bandit 6 NM away
// at my 4 o'clock — go engage him." That's what HandleThreat gives us. The
// host injects the threat entity (from SensorFusion spike detection or a
// wingman radio call); HandleThreat decides whether to engage or drop it,
// then runs WvrEngage against it for up to 10 seconds before re-evaluating.
//
// Behavior (FF handlethreat.cpp:16-54):
//   1. No threatPtr → return false (caller runs normal per-mode dispatch).
//   2. Decrement threatTimer.
//   3. When threatTimer expires, re-evaluate the threat:
//        - Drop it if the threat is dead/missing, > 8 NM away, or
//          > 5 NM AND ataFrom > 90° (running away).
//        - Otherwise, reset threatTimer to 10 s.
//   4. Run RollAndPull against the threat (the universal WVR BFM).
//   5. Return true (caller skips the normal per-mode switch this frame).
//
// The 10 s re-evaluation window is important: without it, the brain would
// chase a threat indefinitely even after the threat disengages. With it,
// the brain periodically re-checks "is this threat still worth chasing?"

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_entity.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// Threat engagement constants (from FreeFalcon handlethreat.cpp).
constexpr double kThreatReevalTimerSec = 10.0;       // re-evaluate every 10 s
constexpr double kThreatMaxRangeFt     = 8.0 * 6076.0;  // 8 NM
constexpr double kThreatBeamRangeFt    = 5.0 * 6076.0;  // 5 NM
constexpr double kThreatBeamAtaFromRad = 90.0 * DTR;    // 90° → beam/stern

// HandleThreat — engage the secondary threat, if any.
//
//   digi   : AI state (reads threatPtr/threatTimer, writes stick/throttle)
//   self   : own aircraft entity
//   as     : aircraft state
//   fcs    : flight control system
//   fcsState : FCS state
//   dt     : frame time
//
// Returns true if the brain should skip its normal per-mode dispatch this
// frame (the threat is being handled). Returns false if there is no threat
// (or the threat was just dropped) — in that case the caller proceeds with
// its normal mode switch.
bool HandleThreat(DigiState& digi, const DigiEntity& self,
                  const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt);

} // namespace digi
} // namespace f4flight
