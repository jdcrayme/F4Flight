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
#include "trace.h"

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

// ===========================================================================
// TestTier — three-tier classification for DIGI AI scenarios.
//
//   LowLevel   — one behavior per scenario (Takeoff, Climb, BVR Engage, ...).
//                Covers every entry in FreeFalcon's DigiMode enum.
//   HighLevel  — a chain of related behaviors (Taxi -> Takeoff -> Departure,
//                Tanker Intercept -> Pre-Contact -> Contact -> Disconnect).
//                The intermediate layer between Low and E2E.
//   EndToEnd   — a full mission analogous to one of FreeFalcon's AMIS_*
//                campaign mission types (BARCAP, INTERCEPT, ESCORT, ...).
//
// The cascade workflow:
//   1. Run all E2E tests on a commit.
//   2. For every E2E that fails, run the associated HighLevel tests.
//   3. For every HighLevel that fails, run the associated LowLevel tests.
//   4. The LowLevel failure points you at the exact behavior that broke.
//
// The mapping tables live in scenario_framework.cpp (g_e2eToHigh, g_highToLow).
// ===========================================================================
enum class TestTier {
    LowLevel,
    HighLevel,
    EndToEnd,
};

// String form used in the trace `testLevel` field and HTML report tabs.
inline const char* testTierName(TestTier t) {
    switch (t) {
        case TestTier::LowLevel:  return "Low Level";
        case TestTier::HighLevel: return "High Level";
        case TestTier::EndToEnd:  return "End-to-End";
    }
    return "Unknown";
}

// Parse a tier from a CLI --level argument. Returns true on success.
inline bool parseTestTier(const std::string& s, TestTier& out) {
    if (s == "low"  || s == "low-level"  || s == "lowlevel")  { out = TestTier::LowLevel;  return true; }
    if (s == "high" || s == "high-level" || s == "highlevel") { out = TestTier::HighLevel; return true; }
    if (s == "e2e"  || s == "end-to-end" || s == "endtoend")  { out = TestTier::EndToEnd;  return true; }
    return false;
}

// Get the cascade mapping (defined in scenario_framework.cpp).
//   highScenariosFor(e2eName)  -> list of high-level scenario names
//   lowScenariosFor(highName)  -> list of low-level scenario names
const std::vector<std::string>& highScenariosFor(const std::string& e2eName);
const std::vector<std::string>& lowScenariosFor(const std::string& highName);

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

// ===========================================================================
// Multi-Aircraft, Telemetry and Conditionals Infrastructure
// ===========================================================================

struct SimulatedAircraft {
    std::string name;
    FlightModel fm;
    SteeringController sc;
    PilotInput input;
    std::string activeModeName;

    explicit SimulatedAircraft(std::string n) : name(std::move(n)) {}
};

class Telemetry {
public:
    Telemetry(std::string name, std::function<double()> sampler)
        : name_(std::move(name)), sampler_(std::move(sampler)) {}

    Telemetry(std::string name, const double* ptr)
        : name_(std::move(name)), sampler_([ptr]() { return ptr ? *ptr : 0.0; }) {}

    const std::string& name() const { return name_; }

    double sample() {
        double val = sampler_ ? sampler_() : 0.0;
        history_.push_back(val);
        return val;
    }

    double lastValue() const {
        return history_.empty() ? 0.0 : history_.back();
    }

    const std::vector<double>& history() const { return history_; }

    void clear() {
        history_.clear();
    }

private:
    std::string name_;
    std::function<double()> sampler_;
    std::vector<double> history_;
};

class Conditional {
public:
    Conditional(std::shared_ptr<Telemetry> tel, bool isRequired = true)
        : tel_(std::move(tel)), isRequired_(isRequired) {}

    virtual ~Conditional() = default;

    virtual void Start() {
        if (!active_) {
            active_ = true;
            hasPassed_ = false;
            hasFailed_ = false;
            if (OnStarted) OnStarted();
        }
    }

    virtual void Stop() {
        if (active_) {
            active_ = false;
            if (OnFinished) OnFinished();
        }
    }

    bool isActive() const { return active_; }
    bool isRequired() const { return isRequired_; }
    bool hasPassed() const { return hasPassed_; }
    bool hasFailed() const { return hasFailed_; }
    std::shared_ptr<Telemetry> tel() const { return tel_; }

    virtual void Evaluate(double dt) = 0;

    std::function<void()> OnPassed;
    std::function<void()> OnFailed;
    std::function<void()> OnStarted;
    std::function<void()> OnFinished;

protected:
    std::shared_ptr<Telemetry> tel_;
    bool isRequired_{true};
    bool active_{false};
    bool hasPassed_{false};
    bool hasFailed_{false};
};

class ConditionalValueReachesRange : public Conditional {
public:
    ConditionalValueReachesRange(std::shared_ptr<Telemetry> tel, double target, double range, bool isRequired = true)
        : Conditional(std::move(tel), isRequired), target_(target), range_(range) {}

    void Evaluate(double /*dt*/) override {
        if (!active_ || hasPassed_ || hasFailed_) return;
        if (!tel_) return;
        double val = tel_->lastValue();
        if (std::abs(val - target_) <= range_) {
            hasPassed_ = true;
            active_ = false;
            if (OnPassed) OnPassed();
        }
    }
private:
    double target_;
    double range_;
};

class ConditionalValueRemainsInRange : public Conditional {
public:
    ConditionalValueRemainsInRange(std::shared_ptr<Telemetry> tel, double target, double range, bool isRequired = true)
        : Conditional(std::move(tel), isRequired), target_(target), range_(range) {}

    void Evaluate(double /*dt*/) override {
        if (!active_ || hasPassed_ || hasFailed_) return;
        if (!tel_) return;
        double val = tel_->lastValue();
        if (std::abs(val - target_) > range_) {
            hasFailed_ = true;
            active_ = false;
            if (OnFailed) OnFailed();
        }
    }
private:
    double target_;
    double range_;
};

class ConditionalNumFrames : public Conditional {
public:
    ConditionalNumFrames(int targetFrames, bool isRequired = true)
        : Conditional(nullptr, isRequired), targetFrames_(targetFrames) {}

    void Evaluate(double /*dt*/) override {
        if (!active_ || hasPassed_ || hasFailed_) return;
        frames_++;
        if (frames_ >= targetFrames_) {
            hasPassed_ = true;
            active_ = false;
            if (OnPassed) OnPassed();
        }
    }

    void Start() override {
        frames_ = 0;
        Conditional::Start();
    }
private:
    int targetFrames_;
    int frames_{0};
};

class ConditionalDuration : public Conditional {
public:
    ConditionalDuration(double targetDuration, bool isRequired = true)
        : Conditional(nullptr, isRequired), targetDuration_(targetDuration) {}

    void Evaluate(double dt) override {
        if (!active_ || hasPassed_ || hasFailed_) return;
        elapsed_ += dt;
        if (elapsed_ >= targetDuration_) {
            hasPassed_ = true;
            active_ = false;
            if (OnPassed) OnPassed();
        }
    }

    void Start() override {
        elapsed_ = 0.0;
        Conditional::Start();
    }
private:
    double targetDuration_;
    double elapsed_{0.0};
};

class ConditionalValueGreaterThan : public Conditional {
public:
    ConditionalValueGreaterThan(std::shared_ptr<Telemetry> tel, double threshold, bool isRequired = true)
        : Conditional(std::move(tel), isRequired), threshold_(threshold) {}

    void Evaluate(double /*dt*/) override {
        if (!active_ || hasPassed_ || hasFailed_) return;
        if (!tel_) return;
        double val = tel_->lastValue();
        if (val > threshold_) {
            hasPassed_ = true;
            active_ = false;
            if (OnPassed) OnPassed();
        }
    }
private:
    double threshold_;
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

    // Test group metadata, e.g. "Fighter Formation", "Ground Ops"
    virtual std::string GetTestGroup() const { return "General"; }

    // Test tier — LowLevel / HighLevel / EndToEnd. Subclasses MUST override
    // this to participate in the cascade workflow and HTML tab filtering.
    // The default returns LowLevel for backward compatibility with old
    // scenarios that haven't been classified yet; those scenarios also get
    // an "Unclassified" warning printed by --list so they're easy to find.
    virtual TestTier GetTestTier() const { return TestTier::LowLevel; }

    // String form for the trace `testLevel` field. Defaults to the tier name
    // so subclasses usually don't need to override this.
    virtual std::string GetTestLevel() const { return testTierName(GetTestTier()); }

    // Build the ordered list of test phases. Called once at startup with
    // the scenario context (aircraft config, profile, cruise altitude).
    virtual std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) = 0;

    // Static scene geometry overlays for the visualization (runway
    // centerline, taxiway paths, approach corridor, etc.). Called once
    // after StartScenario. Default: empty (no scene geometry).
    virtual std::vector<TraceGeometry> traceGeometry() const { return {}; }

    std::shared_ptr<SimulatedAircraft> CreateAircraft(const std::string& name, const AircraftConfig& cfg) {
        auto ac = std::make_shared<SimulatedAircraft>(name);
        // Pre-initialize config to match standard expectations
        ac->sc.setCornerSpeed(cfg.geometry.cornerVcas_kts > 0 ? cfg.geometry.cornerVcas_kts : 330.0);
        ac->sc.setMaxGs(cfg.geometry.maxGs);
        ac->sc.setMaxBank(45.0);
        ac->sc.setAltitude(10000.0);
        ac->sc.setHeading(0.0);
        ac->sc.setMaxGamma(15.0);
        aircraftList_.push_back(ac);
        return ac;
    }

    std::shared_ptr<Telemetry> CreateTelemetry(std::string name, const double* ptr) {
        auto tel = std::make_shared<Telemetry>(std::move(name), ptr);
        telemetryList_.push_back(tel);
        return tel;
    }

    std::shared_ptr<Telemetry> CreateTelemetry(std::string name, std::function<double()> sampler) {
        auto tel = std::make_shared<Telemetry>(std::move(name), std::move(sampler));
        telemetryList_.push_back(tel);
        return tel;
    }

    template <typename T, typename... Args>
    std::shared_ptr<T> CreateConditional(Args&&... args) {
        auto cond = std::make_shared<T>(std::forward<Args>(args)...);
        conditionalList_.push_back(cond);
        return cond;
    }

    const std::vector<std::shared_ptr<SimulatedAircraft>>& aircraftList() const { return aircraftList_; }
    const std::vector<std::shared_ptr<Telemetry>>& telemetries() const { return telemetryList_; }
    const std::vector<std::shared_ptr<Conditional>>& conditionals() const { return conditionalList_; }

    void ClearScenarioObjects() {
        aircraftList_.clear();
        telemetryList_.clear();
        conditionalList_.clear();
    }

protected:
    std::string scenarioName_;
    std::vector<std::shared_ptr<SimulatedAircraft>> aircraftList_;
    std::vector<std::shared_ptr<Telemetry>> telemetryList_;
    std::vector<std::shared_ptr<Conditional>> conditionalList_;
};

// ---------------------------------------------------------------------------
// ConditionalManeuverTest — Bridge to support flattened scenarios
// ---------------------------------------------------------------------------
class ConditionalManeuverTest : public ManeuverTest {
public:
    ConditionalManeuverTest(ManeuverScenario& scenario, const char* name, double maxTime)
        : ManeuverTest(name, maxTime), scenario_(scenario) {}

    void Init(SteeringController& /*sc*/, FlightModel& /*fm*/) override {
        // Conditionals are started/managed by scenario logic
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
    }

    void Finish() const override {
        std::printf("  --- Chained Conditionals Summary ---\n");
        for (const auto& cond : scenario_.conditionals()) {
            std::string name = cond->tel() ? cond->tel()->name() : "Unnamed";
            std::printf("  Conditional [%s]: active=%d, passed=%d, failed=%d\n",
                        name.c_str(), cond->isActive(), cond->hasPassed(), cond->hasFailed());
        }
    }

    bool IsFinished() const override {
        if (phaseTime_ >= maxTime_) return true;

        bool allRequiredFinished = true;
        bool hasRequired = false;
        for (const auto& cond : scenario_.conditionals()) {
            if (cond->isRequired()) {
                hasRequired = true;
                if (!cond->hasPassed() && !cond->hasFailed()) {
                    allRequiredFinished = false;
                    break;
                }
            }
        }
        return hasRequired && allRequiredFinished;
    }

    bool IsPassed() const override {
        bool hasRequired = false;
        for (const auto& cond : scenario_.conditionals()) {
            if (cond->isRequired()) {
                hasRequired = true;
                if (!cond->hasPassed() || cond->hasFailed()) return false;
            }
        }
        return hasRequired;
    }

    std::vector<TestCondition> conditions() const override {
        std::vector<TestCondition> conds;
        for (const auto& cond : scenario_.conditionals()) {
            if (cond->tel()) {
                conds.push_back({cond->tel()->name().c_str(),
                                 cond->hasPassed() ? "Passed" : (cond->hasFailed() ? "Failed" : "Pending"),
                                 cond->hasPassed()});
            } else {
                conds.push_back({"Conditional",
                                 cond->hasPassed() ? "Passed" : (cond->hasFailed() ? "Failed" : "Pending"),
                                 cond->hasPassed()});
            }
        }
        return conds;
    }

private:
    ManeuverScenario& scenario_;
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

    // List scenario names filtered by tier. Used by --level CLI flag and
    // by the cascade runner to enumerate the scenarios in a single tier.
    std::vector<std::string> listByTier(TestTier tier) const {
        std::vector<std::string> names;
        for (const auto& kv : scenarios_) {
            auto s = kv.second();
            if (s && s->GetTestTier() == tier) names.push_back(kv.first);
        }
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
