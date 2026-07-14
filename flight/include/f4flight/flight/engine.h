// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// engine.h
//
// Turbofan engine model with afterburner. Port of engine.cpp.
//
//   - 2-D altitude x mach thrust tables (3 power settings: idle, mil, AB)
//   - Spool dynamics via first-order lag
//   - Per-engine-type RPM schedules (PW-100/220/229, GE-110/129)
//   - Fuel flow from separate tables or legacy factor model
//
// All Imperial units (ft, lb, slugs, lb/hr).

#pragma once

#include "f4flight/flight/aircraft_config.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/core/lookup.h"
#include "f4flight/flight/core/math.h"

namespace f4flight {

// Engine model for a single engine. The FlightModel owns one or two of these.
class EngineModel {
public:
    EngineModel() = default;
    explicit EngineModel(const EngineTable* table, const AuxAero* aux);

    // Recompute thrust, RPM, fuel flow for one time step.
    //   dt           : step size, seconds
    //   alt_ft       : altitude, feet, positive upward
    //   mach         : Mach number
    //   vt_ftps      : true airspeed, ft/s
    //   mass_slugs   : current aircraft mass, slugs
    //   throttle     : 0..1.5 (1.0 = MIL, 1.5 = full AB)
    //   ethrst       : thrust-reverse coefficient from Axial() (1.0 = no reverse)
    //   simplified   : if true, scale fuel flow by 0.75
    void update(double dt,
                double alt_ft,
                double mach,
                double vt_ftps,
                double mass_slugs,
                double throttle,
                double ethrst,
                bool   simplified,
                EngineState& state) const;

    // Compute body-axis thrust force (ft/s^2, i.e. thrust / mass).
    // Returns (xprop, yprop, zprop) in body axes.
    // nozzlePos=0 means straight back; >0 means vectored (Harrier-style).
    static void bodyForces(double thrust_accel,
                           double sinAlpha,
                           double cosAlpha,
                           double nozzlePos,
                           double& xprop,
                           double& yprop,
                           double& zprop,
                           double& xsprop,
                           double& zsprop);

    double fuelFlow() const noexcept { return lastFuelFlow_; }

private:
    double engineRpmMods(double rpmCmd, double alt_ft, double mach, double vcas) const noexcept;

    const EngineTable* table_{nullptr};
    const AuxAero*     aux_{nullptr};

    // Cached lookups (rebuilt if the table changes -- but we treat the table
    // as immutable after construction).
    Lookup2D thrustIdle_;
    Lookup2D thrustMil_;
    Lookup2D thrustAb_;
    Lookup2D ffIdle_;
    Lookup2D ffMil_;
    Lookup2D ffAb_;
    bool     hasFuelFlowTables_{false};

    mutable double lastFuelFlow_{0.0};
};

} // namespace f4flight
