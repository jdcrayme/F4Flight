// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// eom.h
//
// Equations of motion: rigid-body 6-DOF integration.
// Port of eom.cpp.
//
// The legacy model uses Forward Euler for the rigid body state. The FCS
// integrators use Adams-Bashforth 2nd-order.
//
// Coordinate frames:
//   World: NED, Z-down (altitude = -z)
//   Body : X-forward, Y-right, Z-down
//   alpha, beta stored in DEGREES (matches legacy)
//   euler angles sigma, gmma, mu, psi, theta, phi in RADIANS

#pragma once

#include "f4flight/flight/aircraft_config.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/core/types.h"

namespace f4flight {

class EquationsOfMotion {
public:
    EquationsOfMotion() = default;
    explicit EquationsOfMotion(const AircraftGeometry* geom, const AuxAero* aux);

    // Run the full EOM update for one time step.
    //   dt          : step size, seconds
    //   input       : pilot input (for ground-turn NWS)
    //   state       : full aircraft state
    void update(double dt, PilotInput const& input, AircraftState& state) const;

private:
    void calcBodyRates(double dt, double qsom, double cnalpha, double cosmu,
                       double cosgam, double singam, double cosbet, double cosalp,
                       double sinalp, double nzcgs, double nycgw, double pstab,
                       double pitchMomentum, double pitchElasticity,
                       AircraftState& state) const;

    void calcBodyOrientation(double dt, AircraftState& state) const;

    void trigonometry(AircraftState& state) const;

    void calculateVt(double dt, double muFric, double singam,
                     double xwaero, double xwprop, AircraftState& state) const;

    void integratePosition(double dt, double cosgam, double singam,
                           double cossig, double sinsig,
                           double windX, double windY,
                           AircraftState& state) const;

    const AircraftGeometry* geom_{nullptr};
    const AuxAero*          aux_{nullptr};
};

} // namespace f4flight
