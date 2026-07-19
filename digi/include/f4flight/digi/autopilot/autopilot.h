// f4flight - digi/autopilot/autopilot.h
//
// Structured autopilot modes.
//
// Port of FreeFalcon's autopilot.cpp (738 lines). FreeFalcon has 12
// autopilot functions (ThreeAxisAP, WaypointAP, LantirnAP, RealisticAP,
// AltHold, PitchRollHold, FollowWP, HDGSel, RollHold, PitchHold,
// CheckForTurn, AcceptManual). These are scattered across ManeuverPrimitives
// in F4Flight.
//
// This class structures them into a clean hierarchy with a single entry
// point (Autopilot::update) that dispatches on the selected mode. The
// modes use the existing ManeuverPrimitives under the hood — no new
// physics, just better organization.
//
// The autopilot is OPTIONAL — the brain's mode dispatch (runWaypoint,
// runRTB, etc.) still works directly. The autopilot provides a higher-
// level API for hosts that want to fly the aircraft without managing
// individual modes.

#pragma once

#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/fcs.h"

namespace f4flight {
namespace digi {

// ===========================================================================
// AutopilotMode — the available autopilot modes.
//
// Matches FreeFalcon's autopilot types:
//   Off          — no autopilot (manual or digi mode dispatch)
//   AltitudeHold — hold a target altitude + heading (FreeFalcon: AltHold)
//   HeadingSelect — turn to target heading, then hold (FreeFalcon: HDGSel)
//   AltitudeSelect — climb/descend to target altitude then hold
//   PitchRollHold — hold pitch + roll attitude (FreeFalcon: PitchRollHold)
// ===========================================================================
enum class AutopilotMode : int {
    Off            = 0,  // no autopilot
    AltitudeHold   = 1,  // hold target altitude + heading
    HeadingSelect  = 2,  // turn to target heading, then hold
    AltitudeSelect = 3,  // climb/descend to target altitude
    PitchRollHold  = 4,  // hold pitch + roll attitude
};

inline const char* autopilotModeName(AutopilotMode m) {
    switch (m) {
        case AutopilotMode::Off:            return "Off";
        case AutopilotMode::AltitudeHold:   return "AltitudeHold";
        case AutopilotMode::HeadingSelect:  return "HeadingSelect";
        case AutopilotMode::AltitudeSelect: return "AltitudeSelect";
        case AutopilotMode::PitchRollHold:  return "PitchRollHold";
    }
    return "Unknown";
}

// ===========================================================================
// Autopilot — structured autopilot controller.
//
// The host sets the mode + targets (altitude, heading) each frame, then
// calls update(). The autopilot writes stick/throttle commands to the
// DigiState, which the brain's compute() maps to PilotInput.
//
// The autopilot uses the flight-phase gain scheduling (PhaseGainSet) — it
// sets state_.nav.flightPhase to Cruise before calling ManeuverPrimitives.
//
// Usage:
//   Autopilot ap;
//   ap.setMode(AutopilotMode::AltitudeHold);
//   ap.setTargetAltitude(15000.0);
//   ap.setTargetHeading(radians(90.0));
//   ap.update(digi, as, fcs, fcsState, dt);
// ===========================================================================
class Autopilot {
public:
    Autopilot() = default;

    // --- Mode + targets ---
    void setMode(AutopilotMode mode) { mode_ = mode; }
    AutopilotMode mode() const { return mode_; }

    void setTargetAltitude(double altFt) { targetAlt_ = altFt; }
    double targetAltitude() const { return targetAlt_; }

    void setTargetHeading(double headingRad) { targetHeading_ = headingRad; }
    double targetHeading() const { return targetHeading_; }

    void setTargetSpeed(double speedKts) { targetSpeed_ = speedKts; }
    double targetSpeed() const { return targetSpeed_; }

    // --- Update — call each frame to generate stick/throttle commands ---
    //
    // Writes to digi.commands (pStick, rStick, yPedal, throttle).
    // The brain's compute() maps these to PilotInput.
    void update(DigiState& digi, const AircraftState& as,
                const FlightControlSystem& fcs, FcsState& fcsState,
                double dt);

private:
    AutopilotMode mode_{AutopilotMode::Off};
    double targetAlt_{0.0};      // ft, positive up
    double targetHeading_{0.0};  // rad
    double targetSpeed_{0.0};    // kts CAS (0 = use cornerSpeed)

    // Captured attitude for PitchRollHold mode
    double targetPitch_{0.0};    // deg
    double targetRoll_{0.0};     // deg
    bool attitudeCaptured_{false};

    // --- Per-mode implementations ---
    void altitudeHold(DigiState& digi, const AircraftState& as,
                       const FlightControlSystem& fcs, FcsState& fcsState,
                       double dt);
    void headingSelect(DigiState& digi, const AircraftState& as,
                        const FlightControlSystem& fcs, FcsState& fcsState,
                        double dt);
    void altitudeSelect(DigiState& digi, const AircraftState& as,
                         const FlightControlSystem& fcs, FcsState& fcsState,
                         double dt);
    void pitchRollHold(DigiState& digi, const AircraftState& as,
                        const FlightControlSystem& fcs, FcsState& fcsState,
                        double dt);
};

} // namespace digi
} // namespace f4flight
