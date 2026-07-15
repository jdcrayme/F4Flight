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
    const double maxGs = std::max(digi.config.maxGs, 1.0);
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

            // Transform body-frame (relAz, relEl) to world frame using the
            // full body-to-world DCM.
            //
            // BUG FIX: previously used yaw-only rotation ("for simplicity,
            // since we don't have the full DCM in DigiEntity"). This
            // produced a wrong trackpoint when self was pitched or banked:
            // the evasion direction was rotated by self.yaw only, ignoring
            // pitch/roll, so the trackpoint could end up underground or
            // behind the aircraft. Now that DigiEntity carries the full
            // DCM (populated by buildSelfEntity / toDigiEntity), we use it.
            //
            // Body-frame direction vector:
            //   x = cos(el) * cos(az)  (forward)
            //   y = cos(el) * sin(az)  (right)
            //   z = -sin(el)           (up — body Z is down, so negate)
            const double bx = std::cos(relEl) * std::cos(relAz);
            const double by = std::cos(relEl) * std::sin(relAz);
            const double bz = -std::sin(relEl);
            // World-frame direction = DCM * body (DCM is body-to-world)
            const double wx = self.dcm.m[0][0] * bx + self.dcm.m[0][1] * by + self.dcm.m[0][2] * bz;
            const double wy = self.dcm.m[1][0] * bx + self.dcm.m[1][1] * by + self.dcm.m[1][2] * bz;
            const double wz = self.dcm.m[2][0] * bx + self.dcm.m[2][1] * by + self.dcm.m[2][2] * bz;

            digi.nav.trackX = self.x + range * wx;
            digi.nav.trackY = self.y + range * wy;
            digi.nav.trackZ = self.z + range * wz;  // NED: +wz (wz already accounts for body Z down)

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
    ManeuverPrimitives::TrackPoint(digi.nav.trackX, digi.nav.trackY, -digi.nav.trackZ,
                                    digi, as, fcs, fcsState, digi.config.maxGs);
    ManeuverPrimitives::MachHold(digi.config.cornerSpeed, as.vcas, true,
                                  digi, as, 200.0, 800.0,
                                  digi.nav.dt, 700.0);
}

} // namespace digi
} // namespace f4flight
