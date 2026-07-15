// f4flight - digi/defensive/handle_threat.cpp
//
// HandleThreat implementation. See handle_threat.h for the architecture
// rationale and the mapping to FreeFalcon sim/digi/handlethreat.cpp.
//
// Adapted to the Round-5 sub-struct DigiState layout: threat state lives
// in digi.threat.{threatPtr, threatTimer}, commands in digi.commands.*,
// and config (skill) in digi.config.skill.

#include "f4flight/digi/defensive/handle_threat.h"
#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/digi_entity.h"  // computeRelativeGeometry
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <cmath>

namespace f4flight {
namespace digi {

bool HandleThreat(DigiState& digi, const DigiEntity& self,
                  const AircraftState& as,
                  const FlightControlSystem& fcs, FcsState& fcsState,
                  double dt) {
    // 1. No threat → nothing to do.
    if (!digi.threat.threatPtr) return false;

    // 2. Decrement threatTimer. FF uses major-frame time; we use dt.
    digi.threat.threatTimer -= dt;

    // 3. Re-evaluate when the timer expires.
    if (digi.threat.threatTimer <= 0.0) {
        const DigiEntity* threat = digi.threat.threatPtr;

        const bool threatGone =
            !threat ||
            threat->isDead;

        bool dropThreat = threatGone;
        if (!threatGone) {
            const RelativeGeometry rg = computeRelativeGeometry(self, *threat);

            // FF handlethreat.cpp:33-34:
            //   range > 8 NM                       → drop
            //   range > 5 NM AND ataFrom > 90°     → drop (threat is beaming/run)
            if (rg.range > kThreatMaxRangeFt) {
                dropThreat = true;
            } else if (rg.range > kThreatBeamRangeFt &&
                       std::fabs(rg.ataFrom) > kThreatBeamAtaFromRad) {
                dropThreat = true;
            }
        }

        if (dropThreat) {
            // We drop concern of this threat. The host is responsible for
            // clearing digi.threat.threatPtr (it owns the entity lifetime);
            // we just zero the local pointer + timer so the next frame
            // returns false.
            digi.threat.threatPtr = nullptr;
            digi.threat.threatTimer = 0.0;
            return false;
        }

        // Threat is still worth chasing — re-arm the 10 s timer.
        digi.threat.threatTimer = kThreatReevalTimerSec;
    }

    // 4. Engage the threat. FF calls WvrEngage() (which is RollAndPull in
    //    F4Flight's simplified WVR port). Pass the threat as the offensive
    //    target so the BFM geometry is computed against the threat, not
    //    against the primary injectedTarget (which may be a different bandit
    //    the brain was originally engaging).
    //
    //    We deliberately do NOT overwrite digi.threat.threatPtr or wvrTarget_ —
    //    RollAndPull reads its target from the entity passed in, so the
    //    primary offensive target pointer is preserved for when HandleThreat
    //    eventually exits.
    RollAndPull(digi, self, *digi.threat.threatPtr, as, fcs, fcsState, dt);

    // 5. Signal that we handled the frame.
    return true;
}

} // namespace digi
} // namespace f4flight
