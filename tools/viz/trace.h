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

// ThreatEntity — a per-frame snapshot of a moving track for the trace.
// Generalizes over aircraft, missiles, bombs, slots, and ghost wingmen.
struct ThreatEntity {
    std::string type;   // "aircraft", "missile", "bomb", "guns", "target", "slot", "wingman", etc.
    double x{0.0}, y{0.0}, z{0.0};
    double speed{0.0};  // ft/s
    std::string name;   // Name of the track, e.g., "Lead", "Slot 1" (optional)
    double psi{0.0}, theta{0.0}, phi{0.0}; // Euler yaw, pitch, roll orientation (radians)
};

// TraceGeometry — static test geometry overlays to display in the 3D/2D views.
// Generalizes over waypoints, runways, taxiways, fixed targets, and airspace corridors.
struct TraceGeometry {
    std::string name;
    std::string type;   // "waypoint", "runway", "taxiway", "target", "corridor", etc.
    std::vector<double> coords; // x1,y1,z1, x2,y2,z2 or list of points for multi-point geometry
    std::string color;  // Hex code "#FFFFFF"
    double width{0.0};  // Stroke width/weight
};

// TestCondition — a single condition required for a test phase/scenario to pass.
struct TestCondition {
    std::string name;
    std::string description;
    bool passed{false};
};

// AdditionalResult — any additional per-test outcome, metric, or summary point.
struct AdditionalResult {
    std::string text;
    std::string color;  // "success", "warning", "danger", or empty
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

// Trace — a complete maneuver trace.
struct Trace {
    std::string aircraft;       // aircraft name (e.g., "f16bk50")
    std::string scenario;       // scenario name (e.g., "digi_groundops")
    std::string testGroup;      // test group, e.g. "Fighter Formation", "Ground Ops"
    std::string testLevel;      // test level, e.g. "Low Level", "High Level", "End-to-End"
    double duration_s{0.0};
    std::vector<TraceFrame> frames;
    std::vector<TraceGeometry> geometry; // generalized static test geometry
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

    // Set test metadata (test group and test level)
    void setTestMetadata(const std::string& testGroup, const std::string& testLevel);

    // Set generalized test geometry
    void setGeometry(const std::vector<TraceGeometry>& geom) { trace_.geometry = geom; }

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
