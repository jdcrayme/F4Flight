// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// gear.h
//
// Landing gear model. The legacy Falcon 4 gear is NOT a spring-damper but
// rather a kinematic constraint: the aircraft is pinned to the ground at
// z = groundZ - minHeight, and friction provides the horizontal reaction.
// Strut compression is integrated as a kinematic state.
//
// Port of gear.cpp + the ground-handling portions of eom.cpp.

#pragma once

#include "f4flight/aircraft_config.h"
#include "f4flight/aircraft_state.h"

namespace f4flight {

class GearModel {
public:
    GearModel() = default;
    GearModel(const AircraftGeometry* geom, const AuxAero* aux);

    // Resize internal wheel state to match the config gear count.
    void init(GearState& gear) const;

    // Compute the minimum body clearance (the lowest gear point in body Z).
    //   minHeight = max over gear points of (gearZ)
    // Used by EOM to pin z = groundZ - minHeight when on ground.
    double computeMinHeight(const GearState& gear) const;

    // Compute the friction coefficient based on brake state, ground type,
    // and carrier deck.
    //   wheelBrakes : true if toe brakes applied
    //   parkingBrake: true if parking brake set
    //   onObject    : true if on carrier deck or hard surface
    //   overRunway  : true if over paved runway
    static double calcMuFric(bool wheelBrakes, bool parkingBrake,
                             bool onObject, bool overRunway);

    // Update strut compression visual state.
    //   groundZ_ft : terrain altitude
    //   z_ft       : aircraft world Z (NED, Z-down)
    //   vt_ftps    : true airspeed
    //   dt         : step size
    void updateStrutCompression(GearState& gear, double groundZ_ft, double z_ft,
                                double vt_ftps, double dt) const;

    // Compute the gear-handle position (0..1 = up..down).
    // The legacy model integrates the gear handle at a fixed rate.
    static double updateGearPos(double& gearPos_state, double gearHandle, double dt);

private:
    const AircraftGeometry* geom_{nullptr};
    const AuxAero*          aux_{nullptr};

    // Legacy gear extension rate (deg/s -> normalized 1/3s)
    static constexpr double GEAR_RATE = 1.0 / 3.0; // 3 seconds to extend
};

} // namespace f4flight
