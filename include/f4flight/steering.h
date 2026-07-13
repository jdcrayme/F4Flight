// f4flight - steering.h
//
// COMPATIBILITY SHIM — delegates to the digi/ subsystem.
//
// This header preserves the original SteeringController API so existing test
// code and host programs keep compiling without changes. New code should use
// f4flight/digi/digi_brain.h directly.
//
// Architecture:
//
//   SteeringController (this file, compatibility shim)
//       ↓ delegates to
//   digi::DigiBrain (the real brain)
//       ↓ dispatches to
//   digi::ManeuverPrimitives (nav primitives)
//   digi::GroundAvoid (ground avoidance)
//   digi::* (future: combat, formation, etc.)
//
// The digi/ subsystem lives under include/f4flight/digi/ and src/digi/.
// The flight model has NO dependency on the digi/ subsystem — the dependency
// flows one way only (digi reads AircraftState, writes PilotInput).

#pragma once

#include "f4flight/aircraft_state.h"
#include "f4flight/core/types.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"

#include <memory>
#include <vector>

namespace f4flight {

// ---------------------------------------------------------------------------
// DigiState — alias for digi::DigiState (backward compatibility).
// ---------------------------------------------------------------------------
using DigiState = digi::DigiState;

// ---------------------------------------------------------------------------
// DigiAI — alias for digi::ManeuverPrimitives (backward compatibility).
// The old steering.h exposed DigiAI as a class of static methods; the new
// digi/ subsystem exposes them as digi::ManeuverPrimitives. Existing test
// code that calls DigiAI::SetPstick etc. keeps working via this alias.
// ---------------------------------------------------------------------------
using DigiAI = digi::ManeuverPrimitives;

// Backward-compatibility constants for commandType (match old steering.h)
namespace DigiAICompat {
    constexpr int ErrorCommand = static_cast<int>(digi::CommandType::ErrorCommand);
    constexpr int GCommand     = static_cast<int>(digi::CommandType::GCommand);
    constexpr int AlphaCommand = static_cast<int>(digi::CommandType::AlphaCommand);
}

// ---------------------------------------------------------------------------
// SteeringController — manages the digi AI state and combines behaviors.
//
// This is now a thin facade over digi::DigiBrain. It preserves the original
// API (setMode, setHeading, setAltitude, setWaypoints, compute, reset) so
// existing test code keeps working. New code should use DigiBrain directly.
// ---------------------------------------------------------------------------
class SteeringController {
public:
    SteeringController();

    // Set the active steering mode. These configure the DigiAI functions
    // that will be called each frame.
    enum class Mode {
        HeadingAltitude,   // HeadingAndAltitudeHold
        Waypoint,          // GoToCurrentWaypoint (simplified)
        Loiter,            // Loiter orbit
        TerrainFollow,     // (not implemented yet)
        Formation,         // (not implemented yet)
        Combat,            // (not implemented yet)
        Manual,            // direct input passthrough
    };

    void setMode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }

    // Goal setters
    void setHeading(double heading_rad) { brain_.setHeading(heading_rad); }
    void setAltitude(double alt_ft) { brain_.setAltitude(alt_ft); }
    void setWaypoints(std::vector<Vec3> wps) { brain_.setWaypoints(std::move(wps)); }
    void setCaptureRadius(double r_ft) { brain_.setCaptureRadius(r_ft); }
    void setMaxGs(double g) { brain_.setMaxGs(g); }
    void setMaxBank(double bank_deg) { brain_.setMaxBank(bank_deg); }
    void setCornerSpeed(double kts) { brain_.setCornerSpeed(kts); }
    void setMaxGamma(double gamma_deg) { brain_.setMaxGamma(gamma_deg); }
    void setTurnG(double load_factor) { brain_.setTurnG(load_factor); }
    void setManualInput(const PilotInput& in) { manual_ = in; }

    // Accessors
    double heading() const { return brain_.state().holdPsi; }
    double altitude() const { return brain_.state().holdAlt; }
    std::size_t currentWaypoint() const { return brain_.currentWaypoint(); }
    bool allWaypointsCaptured() const { return brain_.allWaypointsCaptured(); }
    const DigiState& digiState() const { return brain_.state(); }
    DigiState&       digiState()       { return brain_.state(); }  // for tests / hosts

    // Main compute — produces PilotInput from the current state + mode.
    PilotInput compute(const AircraftState& state, double dt, double groundZ,
                       const FlightControlSystem& fcs, FcsState& fcsState);

    // Reset all digi state (call between independent test phases)
    void reset() noexcept { brain_.reset(); }

    // --- Access to the underlying brain (for new code) ---
    digi::DigiBrain& brain() { return brain_; }
    const digi::DigiBrain& brain() const { return brain_; }

private:
    digi::DigiBrain brain_;
    Mode mode_{Mode::HeadingAltitude};
    PilotInput manual_;
};

// ---------------------------------------------------------------------------
// Utility functions (kept for backward compatibility with test code)
// ---------------------------------------------------------------------------
double headingError(double setpoint, double current) noexcept;
double turnCompensatedG(const AircraftState& state) noexcept;

} // namespace f4flight
