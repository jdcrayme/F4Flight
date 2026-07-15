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

#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/core/types.h"
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

    // Configuration setters — read-modify-write through DigiConfig so we
    // don't touch the deprecated DigiBrain shims. Each call is O(1) and
    // doesn't disturb fields not named in the method.
    void setMaxGs(double g) {
        digi::DigiConfig cfg = brain_.config();
        cfg.maxGs = g;
        brain_.configure(cfg);
    }
    void setMaxBank(double bank_deg) {
        digi::DigiConfig cfg = brain_.config();
        cfg.maxBankDeg = bank_deg;
        brain_.configure(cfg);
    }
    void setCornerSpeed(double kts) {
        digi::DigiConfig cfg = brain_.config();
        cfg.cornerSpeedKts = kts;
        brain_.configure(cfg);
    }
    void setMaxGamma(double gamma_deg) {
        digi::DigiConfig cfg = brain_.config();
        cfg.maxGammaDeg = gamma_deg;
        brain_.configure(cfg);
    }
    void setTurnG(double load_factor) {
        digi::DigiConfig cfg = brain_.config();
        cfg.turnLoadFactor = load_factor;
        brain_.configure(cfg);
    }
    void setManualInput(const PilotInput& in) { manual_ = in; }

    // --- Threat/target setters (Tier 1-2) ---
    // Each call rebuilds the FrameInputs from the previous frame inputs so
    // only the named field changes.
    //
    // CONTRACT FIX: previously these methods ALSO wrote directly to
    // brain_.stateMutable() to "mirror the deprecated shim's commit
    // side-effect". This bypassed the setFrameInputs() contract and created
    // a subtle race: the brain's compute() would re-apply the injected
    // values anyway (lines 151-169 of digi_brain.cpp), making the bypass
    // redundant for the normal case — but the missile-state reset
    // (missileDefeatTtgo = -1.0) was only done in compute() on ID change,
    // and the bypass forced it immediately.
    //
    // Now we trust the brain's compute() to handle everything: it applies
    // injected threats, resets per-missile state on ID change, runs
    // SensorFusion, and dispatches to the right mode. The SteeringController
    // is a thin facade that only configures — it does NOT touch brain state
    // directly.
    void setIncomingMissile(const digi::DigiEntity* m) {
        digi::FrameInputs fi = brain_.frameInputs();
        fi.injectedMissile = m;
        brain_.setFrameInputs(fi);
    }
    void setGunsThreat(const digi::DigiEntity* t) {
        digi::FrameInputs fi = brain_.frameInputs();
        fi.injectedGunsThreat = t;
        brain_.setFrameInputs(fi);
    }
    void setTarget(const digi::DigiEntity* t) {
        digi::FrameInputs fi = brain_.frameInputs();
        fi.injectedTarget = t;
        brain_.setFrameInputs(fi);
    }

    // --- Wingman / formation setup ---
    // setLead: inject the flight lead entity. When non-null AND setWingman(true)
    // has been called, the brain enters Wingy mode and follows the lead.
    void setLead(const digi::DigiEntity* lead) {
        digi::FrameInputs fi = brain_.frameInputs();
        fi.injectedLead = lead;
        brain_.setFrameInputs(fi);
    }
    // setWingman: mark this aircraft as a wingman (isWing=true) and set the
    // flight lead's entity ID. The host must also call setLead() each frame
    // to provide the lead's current position/velocity.
    void setWingman(digi::EntityId leadId, int slot) {
        brain_.stateMutable().formation.isWing = true;
        brain_.stateMutable().formation.flightLeadId = leadId;
        brain_.stateMutable().formation.vehicleInUnit = slot;
    }
    // setFormation: set the formation type (FormationType enum value) and
    // optionally the side mirror (+1 = right, -1 = left).
    void setFormation(int formationId, int side = 1) {
        brain_.stateMutable().formation.formationId = formationId;
        brain_.stateMutable().formation.wingman.formSide = side;
    }
    // Kickout / Closeup: adjust lateral spacing.
    void kickout() {
        brain_.stateMutable().formation.wingman.formLateralSpaceFactor = 2.0;
    }
    void closeup() {
        brain_.stateMutable().formation.wingman.formLateralSpaceFactor = 0.5;
    }
    void toggleSide() {
        auto& ws = brain_.stateMutable().formation.wingman;
        ws.formSide = -ws.formSide;
    }

    // Accessors
    double heading() const { return brain_.state().nav.holdPsi; }
    double altitude() const { return brain_.state().nav.holdAlt; }
    std::size_t currentWaypoint() const { return brain_.currentWaypoint(); }
    bool allWaypointsCaptured() const { return brain_.allWaypointsCaptured(); }
    const DigiState& digiState() const { return brain_.state(); }
    DigiState&       digiState()       { return brain_.stateMutable(); }  // for tests / hosts

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
