// f4flight - digi/defensive/collision_avoid.cpp
//
// Port of FreeFalcon cavoid.cpp (139 LOC).

#include "f4flight/digi/defensive/collision_avoid.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {
namespace digi {

// Constants from FreeFalcon cavoid.cpp
static constexpr double kGSLimit    = 9.0;     // max G for reaction time
static constexpr double kHRange     = 200.0;   // ft — collision threshold
static constexpr double kHRangeSq   = 40000.0; // kHRange²
static constexpr double kReactFact  = 0.55;    // reaction time fudge factor

bool CollisionCheck(DigiState& digi, const DigiEntity& self,
                    const DigiEntity& target) {
    const RelativeGeometry rg = computeRelativeGeometry(self, target);

    // Reaction time: function of G available + aggression factor
    // (FF cavoid.cpp:34)
    const double maxGs = std::max(digi.maxGs, 1.0);
    const double reactTime = (kGSLimit / maxGs) * kReactFact;

    // Time to impact
    const double timeToImpact = (rg.range - kHRange) /
                                std::max(-rg.rangedot, 1.0);

    // Not a problem if impact is far off
    if (timeToImpact > reactTime && rg.range > kHRange) {
        return false;
    }

    // Linearly extrapolate both velocity vectors and check for close approach
    // (FF cavoid.cpp:73-105)
    double pastRngSq = 1.0e8;
    double dt = 0.05;

    while (dt < reactTime) {
        const double ox = self.x + self.vx * dt;
        const double oy = self.y + self.vy * dt;
        const double oz = self.z + self.vz * dt;

        const double tx = target.x + target.vx * dt;
        const double ty = target.y + target.vy * dt;
        const double tz = target.z + target.vz * dt;

        const double rngSq = (ox - tx) * (ox - tx) +
                             (oy - ty) * (oy - ty) +
                             (oz - tz) * (oz - tz);

        if (rngSq <= kHRangeSq) {
            // Collision predicted — set evasion trackpoint
            // (FF cavoid.cpp:117-128)
            const double relEl = 45.0 * DTR;
            double relAz;
            // droll = target.roll - self.roll; if droll > 0, turn right;
            // else turn left. (FF uses localData->droll)
            const double droll = target.roll - self.roll;
            if (droll > 0.0) {
                relAz = -45.0 * DTR;
            } else {
                relAz = 45.0 * DTR;
            }

            const double range = 10000.0;  // evasion trackpoint distance

            // Transform body-frame (relAz, relEl) to world frame
            // Use self's DCM (body-to-world). For simplicity, use yaw only
            // since we don't have the full DCM in DigiEntity.
            const double cosAz = std::cos(relAz + self.yaw);
            const double sinAz = std::sin(relAz + self.yaw);
            const double cosEl = std::cos(relEl);
            const double sinEl = std::sin(relEl);

            digi.trackX = self.x + range * cosEl * cosAz;
            digi.trackY = self.y + range * cosEl * sinAz;
            digi.trackZ = self.z - range * sinEl;  // NED: negative up

            return true;
        }

        // Break if range is diverging
        if (rngSq > pastRngSq) break;
        pastRngSq = rngSq;
        dt += 0.1;
    }

    return false;
}

void CollisionAvoid(DigiState& digi, const DigiEntity& self,
                    const AircraftState& as,
                    const FlightControlSystem& fcs, FcsState& fcsState) {
    (void)self;
    // FF cavoid.cpp:136-139: TrackPoint(maxGs, cornerSpeed)
    // TrackPoint calls AutoTrack internally.
    ManeuverPrimitives::TrackPoint(digi.trackX, digi.trackY, -digi.trackZ,
                                    digi, as, fcs, fcsState, digi.maxGs);
    ManeuverPrimitives::MachHold(digi.cornerSpeed, as.vcas, true,
                                  digi, as, 200.0, 800.0,
                                  digi.dt, 700.0);
}

} // namespace digi
} // namespace f4flight
