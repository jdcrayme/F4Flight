// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// fcs.h
//
// Flight Control System: F-16-like AOA / G command system with limiters.
// Port of fcs.cpp, gain.cpp, pitch.cpp, roll.cpp, yaw.cpp.

#pragma once

#include "f4flight/aircraft_config.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/core/lookup.h"

namespace f4flight {

class FlightControlSystem {
public:
    FlightControlSystem() = default;
    FlightControlSystem(const AircraftConfig* cfg, const AircraftGeometry* geom, const AuxAero* aux);

    // Run the full FCS update for one time step.
    void update(double dt,
                double qbar,
                double qsom,
                double mach,
                double vt_ftps,
                double vcas_kts,
                double alpha_deg,
                double beta_deg,
                double cosmu,
                double cosgam,
                double singam,
                double costhe,
                double cosphi,
                double loadingFraction,
                bool   inAir,
                double nzcgs,
                double nycgw,
                bool   gearDown,
                bool   refueling,
                bool   landingGainsActive,
                PilotInput const& input,
                FcsState& fcs,
                AeroState& aero) const;

    // Limiter access
    double applyLimiter(LimiterKey key, double x) const;

private:
    void computeGains(double qbar, double qsom, double vt, double alpha_deg,
                      double clift0, double clalph0, double clalpha, double cnalpha,
                      double cosgam, double cosmu, double costhe, double cosphi,
                      double loadingFraction, bool inAir,
                      bool landingGains, double gearPos, FcsState& fcs) const;

    void runPitch(double dt, double qbar, double qsom, double vt, double vcas_kts,
                  double alpha_deg, double cosmu, double cosgam, double singam,
                  double nzcgs, double cl, double clalpha, double clalph0,
                  double cnalpha, double aoamin, double aoamax, double maxGs,
                  PilotInput const& input, FcsState& fcs, AeroState& aero) const;

    void runRoll(double dt, double qbar, double vcas_kts, double alpha_deg,
                 double gearPos, PilotInput const& input, FcsState& fcs) const;

    void runYaw(double dt, double qbar, double qsom, double vt, double vcas_kts,
                double beta_deg, double nycgw, double betmin, double betmax,
                PilotInput const& input, FcsState& fcs, AeroState& aero) const;

    const AircraftConfig*   cfg_{nullptr};
    const AircraftGeometry* geom_{nullptr};
    const AuxAero*          aux_{nullptr};

    Lookup2D rollCmdTable_;
};

} // namespace f4flight
