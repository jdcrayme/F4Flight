// f4flight - steering.h
//
// AI steering ported from FreeFalcon's DigitalBrain (sim/digi/).
//
// Architecture:
//
//   The FreeFalcon digi AI uses GammaHold (flight path angle hold) as the
//   core pitch controller. GammaHold commands a desired flight path angle,
//   computes a G-command from the gamma error, then converts the G-command
//   to a stick deflection via a nonlinear sqrt mapping (SetPstick).
//
//   AltHold wraps GammaHold by scaling the altitude error into a gamma
//   command. MachHold controls throttle from speed error. HeadingHold /
//   LevelTurn control roll from heading error.
//
//   This is NOT a PID-based controller. It is a direct port of the FreeFalcon
//   digi AI steering functions, preserving the exact control laws, gain
//   scheduling, and smoothing behavior.
//
// Key differences from the old f4flight steering:
//   - GammaHold uses flight path angle (gamma), not altitude PID
//   - SetPstick converts G to stick via sqrt((G-1)/(maxGs-costhe))
//   - SetRstick converts roll error to stick via kr01*tr01 scaling
//   - Stick commands are smoothed (0.2*old + 0.8*new)
//   - Low-speed stick authority reduction below 300 kts
//   - MachHold uses linear throttle + integral, not PID

#pragma once

#include "f4flight/aircraft_state.h"
#include "f4flight/core/types.h"

#include <memory>
#include <vector>

namespace f4flight {

// ---------------------------------------------------------------------------
// DigiState — persistent state for the digi AI steering functions.
//
// In FreeFalcon, this state lives in the DigitalBrain class. Here we
// encapsulate it in a struct so the SteeringController can own it.
// ---------------------------------------------------------------------------
struct DigiState {
    // Stick smoothing state (SetPstick / SetRstick / SetYpedal)
    double pStick{0.0};
    double rStick{0.0};
    double yPedal{0.0};
    double throttle{0.5};  // current throttle command [0, 1.5]

    // GammaHold integral
    double gammaHoldIError{0.0};

    // MachHold integral
    double autoThrottle{0.0};

    // LevelTurn state (0 = leveling, 1 = banking, 2 = holding turn)
    int trackMode{0};

    // Waypoint state
    int onStation{0};  // 0=NotThereYet, 1=Arrived, 2=Stabalizing, 3=OnStation
    int waypointMode{1};
    double holdAlt{0.0};
    double holdPsi{0.0};

    // Configuration
    double cornerSpeed{330.0};  // kts, from aircraft config
    double maxGs{9.0};
    double maxRoll{30.0};       // deg
    double maxRollDelta{5.0};   // deg

    void reset() noexcept {
        pStick = rStick = yPedal = 0.0;
        gammaHoldIError = 0.0;
        autoThrottle = 0.0;
        trackMode = 0;
        onStation = 0;
        waypointMode = 1;
    }
};

// ---------------------------------------------------------------------------
// DigiAI — the core FreeFalcon digi AI steering functions.
//
// Each function is a direct port of the corresponding DigitalBrain method.
// They operate on DigiState + AircraftState + FcsState and produce stick
// commands (pStick, rStick, yPedal, throtl).
// ---------------------------------------------------------------------------
class DigiAI {
public:
    // SetPstick — convert a G-command (or error/alpha) to a stick deflection.
    // Direct port of DigitalBrain::SetPstick (mnvers.cpp:300-362).
    //
    //   pitchError: the commanded G (for GCommand), pitch error in degrees
    //               (for ErrorCommand), or alpha in degrees (for AlphaCommand)
    //   gLimit:     the G limit (typically maxGs)
    //   commandType: 0=ErrorCommand, 1=GCommand, 2=AlphaCommand
    //
    // Updates: digi.pStick
    static void SetPstick(double pitchError, double gLimit, int commandType,
                          DigiState& digi, const AircraftState& state);

    // SetRstick — convert a roll error (degrees) to a stick deflection.
    // Direct port of DigitalBrain::SetRstick (mnvers.cpp:364-390).
    //
    // Updates: digi.rStick
    static void SetRstick(double rollError, DigiState& digi,
                          const class FlightControlSystem& fcs,
                          const FcsState& fcsState);

    // SetYpedal — convert a yaw error to a pedal deflection.
    // Direct port of DigitalBrain::SetYpedal (mnvers.cpp:392-397).
    //
    // Updates: digi.yPedal
    static void SetYpedal(double yawError, DigiState& digi);

    // GammaHold — hold a desired flight path angle (gamma).
    // Direct port of DigitalBrain::GammaHold (mnvers.cpp:837-866).
    //
    //   desGamma: desired flight path angle in degrees
    //
    // Updates: digi.pStick, digi.gammaHoldIError
    static void GammaHold(double desGamma, DigiState& digi,
                          const AircraftState& state, double maxGs);

    // AltHold — hold a desired altitude.
    // Direct port of DigitalBrain::AltHold (autopilot.cpp:323-378).
    //
    //   desAlt: desired altitude in feet (positive up)
    //
    // Updates: digi.pStick (via GammaHold)
    static void AltHold(double desAlt, DigiState& digi, const AircraftState& state,
                        double maxGs);

    // AltitudeHold — full altitude hold with wings level.
    // Direct port of DigitalBrain::AltitudeHold (mnvers.cpp:759-784).
    //
    //   desAlt: desired altitude in feet
    //   Returns: true if within 25 ft of target
    //
    // Updates: digi.pStick, digi.rStick, digi.yPedal
    static bool AltitudeHold(double desAlt, DigiState& digi,
                             const AircraftState& state,
                             const FlightControlSystem& fcs,
                             const FcsState& fcsState, double maxGs);

    // HeadingAndAltitudeHold — hold heading + altitude.
    // Direct port of DigitalBrain::HeadingAndAltitudeHold (mnvers.cpp:786-835).
    //
    //   desPsi: desired heading (radians)
    //   desAlt: desired altitude (feet)
    //   Returns: true if within tolerance
    //
    // Updates: digi.pStick, digi.rStick, digi.yPedal
    static bool HeadingAndAltitudeHold(double desPsi, double desAlt,
                                       DigiState& digi, const AircraftState& state,
                                       const FlightControlSystem& fcs,
                                       const FcsState& fcsState, double maxGs);

    // LevelTurn — turn to a heading at a specified load factor.
    // Direct port of DigitalBrain::LevelTurn (mnvers.cpp:713-757).
    //
    //   loadFactor: target G (typically 2.0)
    //   turnDir: +1 (right) or -1 (left)
    //   newTurn: true if starting a new turn (resets gamma integral)
    //
    // Updates: digi.pStick, digi.rStick, digi.yPedal, digi.trackMode
    static void LevelTurn(double loadFactor, double turnDir, bool newTurn,
                          DigiState& digi, const AircraftState& state,
                          const FlightControlSystem& fcs,
                          const FcsState& fcsState, double maxGs);

    // MachHold — hold a target speed via throttle.
    // Direct port of DigitalBrain::MachHold (mnvers.cpp:414-665).
    //
    //   targetSpeed: desired speed in knots
    //   currentSpeed: current speed in knots
    //   adjustPitch: if true, add pStick/15 to throttle (pitch-throttle coupling)
    //
    // Returns: true if within 10% of target speed
    //
    // Updates: digi.autoThrottle
    // Output: throttle [0, 1.5]
    static bool MachHold(double targetSpeed, double currentSpeed, bool adjustPitch,
                         DigiState& digi, const AircraftState& state,
                         double minVcas, double maxVcas);

    // Loiter — orbit pattern.
    // Direct port of DigitalBrain::Loiter (mnvers.cpp:667+).
    static void Loiter(DigiState& digi, const AircraftState& state,
                       const FlightControlSystem& fcs,
                       const FcsState& fcsState, double maxGs);

    // Constants for commandType (match FreeFalcon AirframeClass flags)
    static constexpr int ErrorCommand = 0;
    static constexpr int GCommand = 1;
    static constexpr int AlphaCommand = 2;
};

// ---------------------------------------------------------------------------
// SteeringController — manages the digi AI state and combines behaviors.
//
// The controller owns a DigiState and provides a simplified API for the
// maneuver test framework. It delegates to DigiAI for the actual control
// laws.
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
    void setHeading(double heading_rad) { digi_.holdPsi = heading_rad; }
    void setAltitude(double alt_ft) { digi_.holdAlt = alt_ft; }
    void setWaypoints(std::vector<Vec3> wps) { wps_ = std::move(wps); curWp_ = 0; }
    void setCaptureRadius(double r_ft) { captureRadius_ = r_ft; }
    void setMaxGs(double g) { digi_.maxGs = g; }
    void setMaxBank(double bank_deg) { digi_.maxRoll = bank_deg; }
    void setCornerSpeed(double kts) { digi_.cornerSpeed = kts; }
    void setManualInput(const PilotInput& in) { manual_ = in; }

    // Accessors
    double heading() const { return digi_.holdPsi; }
    double altitude() const { return digi_.holdAlt; }
    std::size_t currentWaypoint() const { return curWp_; }
    bool allWaypointsCaptured() const { return curWp_ >= wps_.size(); }
    const DigiState& digiState() const { return digi_; }
    DigiState&       digiState()       { return digi_; }  // for tests / hosts

    // Main compute — produces PilotInput from the current state + mode.
    // Requires the FCS state (for kr01, tr01 used by SetRstick).
    PilotInput compute(const AircraftState& state, double dt, double groundZ,
                       const FlightControlSystem& fcs, const FcsState& fcsState);

    // Reset all digi state (call between independent test phases)
    void reset() noexcept { digi_.reset(); curWp_ = 0; }

private:
    DigiState digi_;
    Mode mode_{Mode::HeadingAltitude};
    std::vector<Vec3> wps_;
    std::size_t curWp_{0};
    double captureRadius_{5000.0};  // ft
    PilotInput manual_;

    // Waypoint following (simplified GoToCurrentWaypoint)
    void runWaypoint(const AircraftState& state, double dt,
                     const FlightControlSystem& fcs, const FcsState& fcsState,
                     PilotInput& out);
};

// ---------------------------------------------------------------------------
// Utility functions (kept for backward compatibility with test code)
// ---------------------------------------------------------------------------
double headingError(double setpoint, double current) noexcept;
double turnCompensatedG(const AircraftState& state) noexcept;

} // namespace f4flight
