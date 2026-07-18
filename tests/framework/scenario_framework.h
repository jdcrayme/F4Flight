// f4flight - maneuver_test.h
//
// Maneuver test framework with a scenario registry.
//
// Design:
//
//   ManeuverTest      — one phase of a test (e.g. "Climb to 25000 ft").
//                       Subclasses configure the steering controller in Init(),
//                       evaluate each frame in Evaluate(), and report their
//                       own pass/fail criteria.
//
//   ManeuverScenario  — a full test sequence: an ordered list of ManeuverTest
//                       phases plus any per-scenario setup (waypoints, target
//                       geometry, etc.). Subclasses build the sequence in
//                       buildSequence().
//
//   ScenarioRegistry  — maps scenario names ("basic", "flightplan",
//                       "approach", ...) to factory functions. New scenarios
//                       self-register via the REGISTER_SCENARIO macro, so
//                       main() never needs editing when a scenario is added.
//
// This structure lets future work (ACM, weapons delivery, ILS approach, ...)
// land as new scenario files without touching the framework or the existing
// scenarios.

#pragma once

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/steering.h"  // SteeringController (AI compat shim)
#include "trace.h"                    // SceneLine (for sceneGeometry())

#include <cstdio>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace f4flight;

namespace f4flight_test {

// ---------------------------------------------------------------------------
// Abstract base class for maneuver test phases.
//
// A "phase" is one segment of a test sequence: "Climb to 25000 ft and hold
// for 60 s after level-off", "Turn to heading 090", "Orbit for 90 s", etc.
// Each subclass owns its own pass/fail criteria and timing.
// ---------------------------------------------------------------------------
class ManeuverTest {
public:

    ManeuverTest(const char* name, double maxTime)
        : testName_(name), maxTime_(maxTime) {}

    virtual ~ManeuverTest() = default;

    const char* name() const { return testName_.c_str(); }

    // Called once at the start of the phase to configure the steering
    // controller (set up behaviors, targets, etc.).
    virtual void Init(SteeringController& sc, FlightModel& fm) = 0;

    // Called each frame. Default implementation tracks min/max altitude,
    // speed, bank, and heading rate, and periodically prints a row.
    virtual void Evaluate(const AircraftState& as, const PilotInput& input, double dt);

    // Called when the phase is finished to print the summary.
    virtual void Finish() const = 0;

    virtual bool IsFinished() const = 0;
    virtual bool IsPassed() const = 0;

    // Optional bank-angle override. If a phase wants to command a specific
    // bank angle directly (e.g. Orbit), it overrides this to return a
    // non-negative value. The runner applies it as a roll-rate command.
    // Returns a negative value to indicate "no override, use the FCS output".
    virtual double bankOverride_rad() const { return -1.0; }

    // Optional full-input override. If a phase wants to bypass the steering
    // controller entirely and command the inputs directly (e.g. a G-step
    // test that sets pstick to achieve a specific G, or an engine test that
    // cycles the throttle), it overrides this to return true and fills
    // `out` with the desired input. The runner will use `out` instead of
    // calling sc.compute().
    //
    // The current AircraftState is passed so the phase can compute feedback
    // (e.g. throttle to hold speed).
    //
    // Default: return false (use the steering controller).
    virtual bool inputOverride(PilotInput& out, const AircraftState& state) const {
        (void)out; (void)state; return false;
    }

    // Human-readable pass/fail criteria for this phase. Shown in the HTML
    // report so reviewers can see what's being checked without reading the
    // source. Default: empty (phase has no explicit criteria text).
    virtual std::string criteria() const { return ""; }

    // Human-readable explanation of WHY the phase failed. Shown in the HTML
    // report alongside the criteria when the phase did not pass. Default:
    // empty (phase passed, or no specific reason).
    //
    // This is the key field for "Any test that fails needs to clearly
    // communicate what conditions were not met." Phases should override this
    // to return a concise, specific reason, e.g.:
    //   "Never entered GunsEngage mode (stayed in WVREngage for 30s)"
    //   "Min dist to slot was 1200ft (needed < 800ft)"
    //   "Max G was 1.08 (needed >= 2.0) — aircraft did not maneuver"
    virtual std::string failureReason() const { return ""; }

    // Checklist of explicit test conditions checked by this phase
    virtual std::vector<TestCondition> conditions() const { return {}; }

    // List of additional results, metrics, or summaries for this phase
    virtual std::vector<AdditionalResult> additionalResults() const { return {}; }

    // Custom trace entities for visualization. Called each frame (when trace
    // recording is enabled) to let the test provide additional entities to draw
    // in the HTML report — e.g. the flight lead, formation slot positions,
    // ghost wingmen, etc. The returned entities are appended to the trace
    // frame alongside the standard threat/target entities.
    //
    // Entity types recognized by the HTML report:
    //   "missile", "guns", "target"  — standard threats (bearing lines / trails)
    //   "lead"                       — green moving point with trail (flight lead)
    //   "slot"                       — blue diamond marker (desired formation slot)
    //   "wingman"                    — cyan moving point with trail (other wingmen)
    //   "airbase"                    — amber square marker (friendly airbase)
    //
    // Default: empty (no custom entities).
    virtual std::vector<ThreatEntity> traceEntities() const { return {}; }

    // Per-frame sample data for the trace. Called each frame (when trace
    // recording is enabled) to let the test publish additional numeric data
    // (target range, heading error, fuel state, TTGO, etc.) that isn't in
    // the standard TraceFrame. The HTML report renders these in the frame
    // readout panel and can plot them in the time-series.
    //
    // Default: empty (no extra samples).
    virtual std::vector<TraceSample> traceSamples() const { return {}; }

protected:
    std::string testName_;
    double phaseTime_{0.0};
    double maxTime_{60.0};


};

inline void ManeuverTest::Evaluate(const AircraftState& /*as*/, const PilotInput& /*input*/, double dt) {
    phaseTime_ += dt;
}

// ===========================================================================
// ManeuverScenario — a full test sequence
// ===========================================================================

// Context passed to a scenario when it builds its sequence. Gives the
// scenario access to the aircraft config, the chosen category profile, and
// the initial cruise altitude (so scenarios can derive climb/descent alts).
struct ScenarioContext {
    const AircraftConfig&    cfg;
};

// ---------------------------------------------------------------------------
// AircraftClass — coarse aerodynamic classification used by scenarios to
// scale test tolerances. Derived at runtime from the config so JSON files
// don't need a new field.
//
//   Fighter : maxGs > 5.0          (F-16, F-15, F-14, MiG-29, Su-27, EF2000,
//                                    Rafale, A-10 — A-10 is borderline but
//                                    its 7.3G / 30° AOA gives it fighter-like
//                                    pitch authority for the transient tests)
//   Heavy   : maxGs <= 4.0         (B-52H 2.3G, C-130 2.3G — low T/W, low G,
//                                    low roll authority, no afterburner)
//
// The gap (4.0 < maxGs <= 5.0) is intentionally empty in the current fleet;
// if an aircraft lands there it will be treated as Fighter. Add an `Attack`
// class here if a future aircraft (e.g. A-10 with revised data, Su-25) needs
// separate tolerances.
// ---------------------------------------------------------------------------
enum class AircraftClass { Fighter, Heavy };

inline AircraftClass aircraftClass(const AircraftConfig& cfg) {
    return (cfg.geometry.maxGs <= 4.0) ? AircraftClass::Heavy : AircraftClass::Fighter;
}

inline bool isHeavy(const AircraftConfig& cfg) {
    return aircraftClass(cfg) == AircraftClass::Heavy;
}

// A scenario is a named, self-contained test sequence. Subclasses build a
// list of ManeuverTest phases in buildSequence(); the runner executes them
// in order and reports pass/fail.
class ManeuverScenario {
public:
    explicit ManeuverScenario(std::string name) : scenarioName_(std::move(name)) {}
    virtual ~ManeuverScenario() = default;

    const std::string& name() const { return scenarioName_; }

    // Human-readable description shown by `maneuver_test --list`.
    virtual std::string GetDescription() const = 0;

    // Test group metadata, e.g. "Fighter Formation", "Ground Ops"
    virtual std::string GetTestGroup() const { return "General"; }

    // Test level metadata, e.g. "Low Level", "High Level", "End-to-End"
    virtual std::string GetTestLevel() const { return "Integration"; }

    // Build the ordered list of test phases. Called once at startup with
    // the scenario context (aircraft config, profile, cruise altitude).
    virtual std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) = 0;

    // Static scene geometry overlays for the visualization (runway
    // centerline, taxiway paths, approach corridor, etc.). Called once
    // after StartScenario. Default: empty (no scene geometry).
    virtual std::vector<SceneLine> sceneGeometry() const { return {}; }

protected:
    std::string scenarioName_;
};

// ===========================================================================
// ScenarioRegistry — name -> factory
// ===========================================================================

class ScenarioRegistry {
public:
    using Factory = std::function<std::unique_ptr<ManeuverScenario>()>;

    static ScenarioRegistry& instance() {
        static ScenarioRegistry r;
        return r;
    }

    void registerScenario(const std::string& name, Factory f) {
        scenarios_[name] = std::move(f);
    }

    std::unique_ptr<ManeuverScenario> create(const std::string& name) const {
        auto it = scenarios_.find(name);
        if (it == scenarios_.end()) return nullptr;
        return it->second();
    }

    std::vector<std::string> list() const {
        std::vector<std::string> names;
        names.reserve(scenarios_.size());
        for (const auto& kv : scenarios_) names.push_back(kv.first);
        return names;
    }

    bool has(const std::string& name) const {
        return scenarios_.find(name) != scenarios_.end();
    }

private:
    ScenarioRegistry() = default;
    std::map<std::string, Factory> scenarios_;
};

// Self-registration helper. Put this in a scenario's .cpp file at namespace
// scope to make the scenario available to `maneuver_test --scenario NAME`.
//   static RegisterScenario reg("basic", []() {
//       return std::make_unique<BasicScenario>();
//   });
struct RegisterScenario {
    RegisterScenario(const std::string& name, ScenarioRegistry::Factory f) {
        ScenarioRegistry::instance().registerScenario(name, std::move(f));
    }
};

} // namespace f4flight_test
