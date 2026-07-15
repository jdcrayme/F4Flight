// f4flight - tools/trace.h
//
// Trace recording for maneuver visualization.
//
// A Trace is a per-frame snapshot of the aircraft state during a maneuver
// test, plus any threat/target entities, waypoints, and scene geometry
// (runway, taxiways). Traces can be:
//   1. Written to a JSON file via --trace-dir on the scenario runner
//   2. Read by trace2svg to produce 2D top-down + side-profile SVGs
//   3. Read by trace2html to produce a self-contained interactive HTML report
//   4. Embedded inline in the HTML report via traceToJson()
//
// The format is a single JSON document (not JSON lines) for simplicity — a
// 90-second trace at 60 Hz is ~5400 frames, well under 2 MB.
//
// Trace files are NEVER generated during normal test runs. They are opt-in
// via --trace-dir / --html on the scenario runner. This keeps ctest output
// clean.

#pragma once

#include "f4flight/flight/aircraft_state.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/ground/ground_ops.h"

#include <string>
#include <vector>

namespace f4flight {

// ThreatEntity — a per-frame snapshot of a threat/target for the trace.
// Used for missiles, guns threats, and offensive targets.
struct ThreatEntity {
    std::string type;   // "missile", "guns", "target"
    double x{0.0}, y{0.0}, z{0.0};
    double speed{0.0};  // ft/s
};

// Waypoint — a navigation waypoint for the trace (scenario-level, not
// per-frame). Captured once from the SteeringController after Init().
struct Waypoint {
    double x{0.0}, y{0.0}, z{0.0};
    std::string name;   // "WP1", "North", etc. — may be empty
};

// SceneLine — a static line in the world used for scene geometry overlays
// (runway centerline, runway edges, taxiway paths, approach corridor, etc.).
// All coordinates are NED (ft, altitude = -z).
struct SceneLine {
    std::string label;           // "RWY 27", "Taxiway A", etc.
    double x1{0.0}, y1{0.0}, z1{0.0};  // start point
    double x2{0.0}, y2{0.0}, z2{0.0};  // end point
    std::string color;           // hex "#FFD700", or empty for default
    double width{0.0};           // stroke width in ft (0 = default 2)
};

// TraceSample — a single key-value sample for a frame (e.g. "range"=5000ft).
// These let scenarios publish additional per-frame data (target range, heading
// error, fuel state, TTGO, etc.) that isn't in the standard TraceFrame.
// The HTML report renders these in the frame readout panel.
struct TraceSample {
    std::string key;
    double      value{0.0};
    std::string unit;   // "ft", "kts", "deg", "s", "" — for display
};

// TraceEvent — a discrete event at a point in time (mode transition, weapon
// fire, sensor detection, test milestone, assertion hit/miss, etc.).
// Events are rendered as markers on the time-series and in an event log.
struct TraceEvent {
    double      t{0.0};          // simulation time (s)
    std::string category;        // "mode", "weapon", "sensor", "test", "info"
    std::string message;         // human-readable description
    std::string severity;        // "info", "warn", "fail" — for color coding
};

// TraceFrame — one frame of the trace.
struct TraceFrame {
    double t{0.0};              // simulation time (s)

    // Aircraft position (NED, ft; altitude = -z)
    double x{0.0}, y{0.0}, z{0.0};

    // Aircraft orientation (radians)
    double psi{0.0}, theta{0.0}, phi{0.0};

    // Velocities
    double vt{0.0};             // true airspeed (ft/s)
    double vcas{0.0};           // calibrated airspeed (kts)

    // Loads
    double nzcgs{0.0};          // normal load factor (G)

    // Pilot input
    double throttle{0.0};
    double pstick{0.0};
    double rstick{0.0};

    // AI state
    std::string mode;           // digi mode name (e.g., "Landing", "MissileDefeat")
    std::string phase;          // ground ops phase name (e.g., "Approach", "TakeoffRoll")
                                // — empty if not in ground ops

    // Threats/targets active this frame
    std::vector<ThreatEntity> threats;

    // Additional per-frame samples (range, heading error, fuel, TTGO, etc.)
    std::vector<TraceSample> samples;
};

// PhaseResult — result of one phase in the scenario.
struct PhaseResult {
    std::string name;
    double start_s{0.0};
    double end_s{0.0};
    bool passed{false};
    bool skipped{false};
    bool reinitializes{false};  // true if the phase called fm.init() (flight
                                // model re-initialized → discontinuity)
    std::string criteria;       // human-readable pass/fail criteria (what's checked)
    std::string failureReason;  // human-readable explanation of WHY the phase
                                // failed (empty if passed). e.g. "Never entered
                                // GunsEngage mode (stayed in WVREngage)"
};

// Trace — a complete maneuver trace.
struct Trace {
    std::string aircraft;       // aircraft name (e.g., "f16bk50")
    std::string scenario;       // scenario name (e.g., "digi_groundops")
    double duration_s{0.0};
    std::vector<PhaseResult> phases;
    std::vector<TraceFrame> frames;
    std::vector<Waypoint> waypoints;    // navigation waypoints (scenario-level)
    std::vector<SceneLine> sceneLines;  // static geometry (runway, taxiways, etc.)
    std::vector<TraceEvent> events;     // discrete events (mode changes, fires, etc.)
};

// TraceRecorder — collects frames during a scenario run.
//
// Usage:
//   TraceRecorder rec;
//   rec.start("f16bk50", "digi_groundops");
//   // set scenario-level data (once, after Init):
//   rec.setWaypoints(wps);
//   rec.addSceneLine(runwayLine);
//   // each frame:
//   rec.record(t, state, input, modeName, phaseName, threats);
//   rec.addSample("range", rangeFt, "ft");  // additional per-frame data
//   // discrete events:
//   rec.addEvent(t, "mode", "Entered GunsEngage", "info");
//   rec.addEvent(t, "weapon", "Gun fired", "info");
//   // at phase boundaries:
//   rec.markPhase("Takeoff", startT, endT, passed, skipped, criteria);
//   rec.setPhaseFailureReason("Never reached rotation speed");
//   // at end:
//   rec.finish(duration);
//   rec.write("trace.json");
//
class TraceRecorder {
public:
    void start(const std::string& aircraft, const std::string& scenario);

    // Record a frame from the current aircraft state + pilot input.
    void record(double t, const AircraftState& as, const PilotInput& input,
                const std::string& modeName, const std::string& phaseName);

    // Record a frame with additional threat entities.
    void record(double t, const AircraftState& as, const PilotInput& input,
                const std::string& modeName, const std::string& phaseName,
                const std::vector<ThreatEntity>& threats);

    // Add a per-frame sample to the LAST recorded frame. Call AFTER record().
    // Used for scenario-specific data (target range, heading error, fuel, etc.).
    void addSample(const std::string& key, double value, const std::string& unit = "");

    // Add a discrete event at time t. Events are rendered as markers on the
    // time-series and in an event log in the HTML report.
    void addEvent(double t, const std::string& category,
                  const std::string& message, const std::string& severity = "info");

    // Set navigation waypoints (scenario-level, called once after Init).
    void setWaypoints(const std::vector<Waypoint>& wps);

    // Add a scene geometry line (runway, taxiway, etc.).
    void addSceneLine(const SceneLine& line);

    // Mark a phase boundary (called at the end of each phase).
    void markPhase(const std::string& name, double start_s, double end_s,
                   bool passed, bool skipped, bool reinitializes,
                   const std::string& criteria = "",
                   const std::string& failureReason = "");

    void finish(double duration_s);

    // Write the trace to a JSON file. Returns true on success.
    bool write(const std::string& path) const;

    const Trace& trace() const { return trace_; }

private:
    Trace trace_;
    double phaseStart_{0.0};
};

// Read a trace from a JSON file. Returns true on success.
// On failure, fills error_msg and returns false.
bool readTrace(const std::string& path, Trace& out, std::string& error_msg);

// Serialize a Trace to compact JSON (no whitespace). Appends to `out`.
// Used by TraceRecorder::write() and by the HTML report generator (to embed
// traces inline in a <script> tag).
void traceToJson(const Trace& trace, std::string& out);

} // namespace f4flight
