// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// flight_model.h
//
// Top-level orchestrator. Owns the subsystems and runs the per-frame update.
// Port of AirframeClass::Exec().

#pragma once

#include "f4flight/aerodynamics.h"
#include "f4flight/aircraft_config.h"
#include "f4flight/aircraft_state.h"
#include "f4flight/atmosphere.h"
#include "f4flight/config/f16c_config.h"
#include "f4flight/eom.h"
#include "f4flight/engine.h"
#include "f4flight/fcs.h"
#include "f4flight/gear.h"

namespace f4flight {

// Flight model orchestrator. Construct one per aircraft.
class FlightModel {
public:
    FlightModel();

    // Initialise the flight model with the given configuration.
    //   initialAltitude_ft : initial altitude AGL (feet, positive up)
    //   initialVt_ftps     : initial true airspeed (ft/s)
    //   initialHeading_rad : initial heading (radians, 0 = North)
    //   inAir              : if true, start airborne; otherwise on ground
    void init(const AircraftConfig& cfg,
              double initialAltitude_ft,
              double initialVt_ftps,
              double initialHeading_rad,
              bool inAir);

    // Run the per-frame update. The host program calls this once per major
    // frame; the model will sub-step internally at the minor-frame rate.
    //   dt                : major-frame step (seconds)
    //   input             : pilot/AI input
    //   groundZ_ft        : terrain altitude at the aircraft position (ft)
    //   groundNormal      : up vector at the terrain point (unit, world frame)
    void update(double dt, const PilotInput& input,
                double groundZ_ft, const Vec3& groundNormal);

    // --- Accessors ---
    const AircraftState& state() const noexcept { return state_; }
    AircraftState&       state()       noexcept { return state_; }
    const AircraftConfig& config() const noexcept { return cfg_; }
    const FlightControlSystem& fcs() const noexcept { return fcs_; }

    // Sub-stepping controls
    void   setMinorPerMajor(int n) noexcept { minorPerMajor_ = std::max(1, n); }
    int    minorPerMajor() const noexcept { return minorPerMajor_; }
    double minorFrameTime() const noexcept { return minorFrameTime_; }

    // For host programs that wish to override the ground-level function
    // (e.g. with a terrain sampler) we expose the ground state for inspection.
    void setGround(double groundZ_ft, const Vec3& normal) {
        state_.gear.groundZ_ft = groundZ_ft;
        state_.gear.groundNormal = normal;
    }

    // Recompute the load factors (body/stability/wind axes) from the current
    // force state. Called by update() but exposed for hosts that need to
    // refresh after manual state mutation.
    void computeLoadFactors();

    // Trim the aircraft at the current state. Iterates alpha and throttle to
    // find a steady-state 1-G condition. Returns true on convergence.
    bool trim();

private:
    // Run one minor-frame step.
    void minorStep(double dt, const PilotInput& input);

    // Compute atmosphere + update aerodynamic state fields used by other
    // subsystems (qsom, qbar, mach, etc.).
    void updateAtmosphere();

    // Recompute strut compression and gear-on-ground flags.
    void updateGear(double dt);

    // Compute accelerometers (load factors) from current force state.
    void accelerometers();

    AircraftConfig cfg_;
    AircraftState  state_;

    Aerodynamics           aero_;
    EngineModel            engine_;
    FlightControlSystem    fcs_;
    GearModel              gear_;
    EquationsOfMotion      eom_;

    // Cached copy of the most recent PilotInput — needed by updateGear()
    // which runs before minorStep() but needs the brake handle state.
    // Round-2 audit: previously calcMuFric was hardcoded (false, false),
    // so PilotInput.wheelBrakes was a dead field. Now we plumb it through.
    PilotInput lastInput_{};

    int    minorPerMajor_{6};
    double minorFrameTime_{1.0 / 360.0}; // 6 sub-steps of 1/60 s = 1/10 s major
};

} // namespace f4flight
