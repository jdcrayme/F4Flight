// f4flight - tools/trace.h
//
// Trace recording for maneuver visualization.
//
// A Trace is a per-frame snapshot of the aircraft state during a maneuver
// test, plus any threat/target entities. Traces can be:
//   1. Written to a JSON file via the --trace flag on maneuver_test
//   2. Read by trace2svg to produce 2D top-down + side-profile SVGs
//   3. Read by f4flight_viz for interactive 3D replay (RayLib)
//
// The format is a single JSON document (not JSON lines) for simplicity — a
// 90-second trace at 60 Hz is ~5400 frames, well under 2 MB.
//
// Trace files are NEVER generated during normal test runs. They are opt-in
// via --trace <file> on maneuver_test, or generated in-memory by the
// interactive viewer. This keeps ctest output clean.

#pragma once

#include "f4flight/aircraft_state.h"
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
};

// PhaseResult — result of one phase in the scenario.
struct PhaseResult {
    std::string name;
    double start_s{0.0};
    double end_s{0.0};
    bool passed{false};
    bool skipped{false};
};

// Trace — a complete maneuver trace.
struct Trace {
    std::string aircraft;       // aircraft name (e.g., "f16bk50")
    std::string scenario;       // scenario name (e.g., "digi_groundops")
    double duration_s{0.0};
    std::vector<PhaseResult> phases;
    std::vector<TraceFrame> frames;
};

// TraceRecorder — collects frames during a scenario run.
//
// Usage:
//   TraceRecorder rec;
//   rec.start("f16bk50", "digi_groundops");
//   // each frame:
//   rec.record(t, state, input, modeName, phaseName);
//   // at phase boundaries:
//   rec.markPhase("Takeoff", startT, endT, passed, skipped);
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

    // Mark a phase boundary (called at the end of each phase).
    void markPhase(const std::string& name, double start_s, double end_s,
                   bool passed, bool skipped);

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

} // namespace f4flight
