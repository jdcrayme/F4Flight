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

#include "f4flight/f4flight.h"
#include <cstdio>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace f4flight;

namespace manuver_test {

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

protected:
    std::string testName_;
    double phaseTime_{0.0};
    double maxTime_{60.0};


};

inline void ManeuverTest::Evaluate(const AircraftState& as, const PilotInput& input, double dt) {
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

    // Build the ordered list of test phases. Called once at startup with
    // the scenario context (aircraft config, profile, cruise altitude).
    virtual std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) = 0;

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

} // namespace f4flight
