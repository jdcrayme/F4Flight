// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aerodynamics.h
//
// Aerodynamic force model. Port of aero.cpp.
//
// The legacy model uses 2-D Mach x alpha tables for CL, CD and CY. There are
// no separate moment tables (Cm, Cl, Cn) -- moments are synthesised by the
// FCS closed loops. This class only computes the forces.

#pragma once

#include "f4flight/aircraft_config.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/atmosphere.h"
#include "f4flight/core/lookup.h"

namespace f4flight {

class Aerodynamics {
public:
    Aerodynamics() = default;
    explicit Aerodynamics(const AeroTable* table, const AircraftGeometry* geom, const AuxAero* aux);

    // Recompute aerodynamic forces and coefficients for this frame.
    //   alpha_deg  : current angle of attack (degrees)
    //   beta_deg   : current sideslip (degrees)
    //   mach       : Mach number
    //   vt_ftps    : true airspeed (ft/s)
    //   qbar       : dynamic pressure (lb/ft^2)
    //   qsom       : q*S/m (ft/s^2 per unit CL)
    //   qovt       : qbar / vt
    //   altitude   : current altitude (ft)
    //   groundZ    : terrain altitude (ft, NED world)
    //   z          : current world Z (ft, NED Z-down)
    //   vcas       : calibrated airspeed (knots)
    void update(double alpha_deg,
                double beta_deg,
                double mach,
                double vt_ftps,
                double qbar,
                double qsom,
                double qovt,
                double altitude_ft,
                double groundZ_ft,
                double z_ft,
                double vcas_kts,
                double pstick,
                AeroState& aero) const;

private:
    const AeroTable*         table_{nullptr};
    const AircraftGeometry*  geom_{nullptr};
    const AuxAero*           aux_{nullptr};

    Lookup2D cl_;
    Lookup2D cd_;
    Lookup2D cy_;
};

} // namespace f4flight
