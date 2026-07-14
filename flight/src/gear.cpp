// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// gear.cpp

#include "f4flight/flight/gear.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/math.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

GearModel::GearModel(const AircraftGeometry* geom, const AuxAero* aux)
    : geom_(geom), aux_(aux) {}

void GearModel::init(GearState& gear) const {
    if (!geom_) return;
    gear.wheels.assign(geom_->gear.size(), GearState::Wheel{});
}

double GearModel::computeMinHeight(const GearState& gear) const {
    (void)gear; // minHeight depends only on the configured gear points, not the runtime state
    if (!geom_ || geom_->gear.empty()) return 0.0;
    // minHeight is the maximum strut-extended Z (most-positive Z in body axes,
    // since body Z is down). This corresponds to the lowest point on the
    // airframe.
    double h = 0.0;
    for (auto const& gp : geom_->gear) {
        if (gp.z > h) h = gp.z;
    }
    // Account for current gear-up position (gear retracted = less height)
    const double gearFactor = 1.0; // assume gear down for min-height calc
    return h * gearFactor;
}

double GearModel::calcMuFric(bool wheelBrakes, bool parkingBrake,
                             bool onObject, bool overRunway) {
    if (onObject) return 20.0;          // carrier deck -- effectively infinite
    if (parkingBrake) return 0.4;       // parking brake
    if (wheelBrakes)  return 0.4;       // toe brakes
    if (overRunway)   return 0.04;      // rolling on paved runway
    return 0.5;                         // grass / dirt
}

void GearModel::updateStrutCompression(GearState& gear, double groundZ_ft,
                                       double z_ft, double vt_ftps,
                                       double dt) const {
    if (!geom_ || gear.wheels.empty()) return;

    // For each wheel, compute compression as the difference between the
    // strut-extended position and the actual ground clearance.
    const double agl = std::fabs(groundZ_ft - z_ft);
    for (std::size_t i = 0; i < gear.wheels.size() && i < geom_->gear.size(); ++i) {
        auto& w = gear.wheels[i];
        const double strutMax = geom_->gear[i].z;
        const double compression = strutMax - agl;
        w.strutCompression_ft = limit(compression, 0.0, strutMax);
        w.onGround = (compression > 0.0);

        // Wheel rotation
        if (w.onGround) {
            const double wheelRadius = std::max(0.5, strutMax * 0.3);
            w.wheelAngle_rad += (vt_ftps * dt) / wheelRadius;
            // Wrap to [0, 2*pi)
            const double twopi = 2.0 * 3.14159265358979323846;
            w.wheelAngle_rad = std::fmod(w.wheelAngle_rad, twopi);
            if (w.wheelAngle_rad < 0.0) w.wheelAngle_rad += twopi;
        }
    }
}

double GearModel::updateGearPos(double& gearPos_state, double gearHandle, double dt) {
    // gearHandle in [-1, +1]: +1 = down, -1 = up. Map to target [0, 1].
    const double target = (gearHandle > 0.0) ? 1.0 : 0.0;
    const double diff = target - gearPos_state;
    const double rate = GEAR_RATE; // 1/3 per second
    if (std::fabs(diff) < rate * dt) {
        gearPos_state = target;
    } else {
        gearPos_state += (diff > 0.0 ? 1.0 : -1.0) * rate * dt;
    }
    return gearPos_state;
}

} // namespace f4flight
