// f4flight - scenario_framework.h
//
// Maneuver test framework with a scenario registry.
// Highly simplified and cleaned to support the single baseline scenario architecture.

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

// TestTier — simplified classification for DIGI AI scenarios.
enum class TestTier {
    LowLevel,
    HighLevel,
    EndToEnd,
};

inline const char* testTierName(TestTier t) {
    switch (t) {
        case TestTier::LowLevel:  return "Low Level";
        case TestTier::HighLevel: return "High Level";
        case TestTier::EndToEnd:  return "End-to-End";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// Abstract base class for maneuver test phases.
// ---------------------------------------------------------------------------
class ManeuverTest {
public:
    ManeuverTest(const char* name, double maxTime)
        : testName_(name), maxTime_(maxTime) {}

    virtual ~ManeuverTest() = default;

    const char* name() const { return testName_.c_str(); }

    virtual void Init(SteeringController& sc, FlightModel& fm) = 0;
    virtual void Evaluate(const AircraftState& as, const PilotInput& input, double dt);
    virtual void Finish() const = 0;

    virtual bool IsFinished() const = 0;
    virtual bool IsPassed() const = 0;

    virtual double bankOverride_rad() const { return -1.0; }
    virtual bool inputOverride(PilotInput& out, const AircraftState& state) const {
        (void)out; (void)state; return false;
    }

    virtual std::string criteria() const { return ""; }
    virtual std::string failureReason() const { return ""; }

    virtual std::vector<TestCondition> conditions() const { return {}; }
    virtual std::vector<AdditionalResult> additionalResults() const { return {}; }

    virtual std::vector<ThreatEntity> traceEntities() const { return {}; }
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
struct ScenarioContext {
    const AircraftConfig&    cfg;
};

enum class AircraftClass { Fighter, Heavy };

inline AircraftClass aircraftClass(const AircraftConfig& cfg) {
    return (cfg.geometry.maxGs <= 4.0) ? AircraftClass::Heavy : AircraftClass::Fighter;
}

inline bool isHeavy(const AircraftConfig& cfg) {
    return aircraftClass(cfg) == AircraftClass::Heavy;
}

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

class ManeuverScenario {
public:
    explicit ManeuverScenario(std::string name) : scenarioName_(std::move(name)) {}
    virtual ~ManeuverScenario() = default;

    const std::string& name() const { return scenarioName_; }

    virtual std::string GetDescription() const = 0;
    virtual std::string GetTestGroup() const { return "General"; }
    virtual TestTier GetTestTier() const { return TestTier::LowLevel; }
    virtual std::string GetTestLevel() const { return testTierName(GetTestTier()); }

    virtual std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) = 0;

    virtual std::vector<TraceGeometry> traceGeometry() const { return {}; }

    std::shared_ptr<SimulatedAircraft> CreateAircraft(const std::string& name, const AircraftConfig& cfg) {
        auto ac = std::make_shared<SimulatedAircraft>(name);
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

class ConditionalManeuverTest : public ManeuverTest {
public:
    ConditionalManeuverTest(ManeuverScenario& scenario, const char* name, double maxTime)
        : ManeuverTest(name, maxTime), scenario_(scenario) {}

    void Init(SteeringController& /*sc*/, FlightModel& /*fm*/) override {}

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

struct RegisterScenario {
    RegisterScenario(const std::string& name, ScenarioRegistry::Factory f) {
        ScenarioRegistry::instance().registerScenario(name, std::move(f));
    }
};

} // namespace f4flight_test
