// f4flight - scenario_framework.h
//
// Simplified, declarative, flat conditional-driven scenario framework.
// Eliminates the ManeuverTest class. Scenarios define simulated aircraft,
// telemetries, and assertions (conditionals) directly.

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
    Conditional(std::shared_ptr<Telemetry> tel, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : tel_(std::move(tel)), isRequired_(isRequired), name_(std::move(name)), criteria_(std::move(criteriaText)) {}

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

    const std::string& name() const { return name_; }
    const std::string& criteria() const { return criteria_; }

    virtual std::string failureReason() const {
        if (hasFailed_) {
            return "Condition failed: " + criteria_;
        }
        if (!hasPassed_ && isRequired_) {
            return "Condition timed out: " + criteria_;
        }
        return "";
    }

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
    std::string name_;
    std::string criteria_;
};

class ConditionalValueReachesRange : public Conditional {
public:
    ConditionalValueReachesRange(std::shared_ptr<Telemetry> tel, double target, double range, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : Conditional(std::move(tel), isRequired, std::move(name), std::move(criteriaText)), target_(target), range_(range) {
        if (criteria_.empty()) {
            criteria_ = "Reach target " + std::to_string(target_) + " within +/- " + std::to_string(range_);
        }
    }

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

    std::string failureReason() const override {
        if (hasFailed_) return "Value diverged from target " + std::to_string(target_);
        if (!hasPassed_) {
            double lastVal = tel_ ? tel_->lastValue() : 0.0;
            return "Value " + std::to_string(lastVal) + " did not reach target " + std::to_string(target_) + " +/- " + std::to_string(range_);
        }
        return "";
    }
private:
    double target_;
    double range_;
};

class ConditionalValueRemainsInRange : public Conditional {
public:
    ConditionalValueRemainsInRange(std::shared_ptr<Telemetry> tel, double target, double range, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : Conditional(std::move(tel), isRequired, std::move(name), std::move(criteriaText)), target_(target), range_(range) {
        if (criteria_.empty()) {
            criteria_ = "Remain in range " + std::to_string(target_) + " +/- " + std::to_string(range_);
        }
    }

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

    void Stop() override {
        if (active_ && !hasFailed_) {
            hasPassed_ = true;
        }
        Conditional::Stop();
    }

    std::string failureReason() const override {
        if (hasFailed_) {
            double lastVal = tel_ ? tel_->lastValue() : 0.0;
            return "Value " + std::to_string(lastVal) + " exceeded limit of " + std::to_string(target_) + " +/- " + std::to_string(range_);
        }
        return "";
    }
private:
    double target_;
    double range_;
};

class ConditionalNumFrames : public Conditional {
public:
    ConditionalNumFrames(int targetFrames, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : Conditional(nullptr, isRequired, std::move(name), std::move(criteriaText)), targetFrames_(targetFrames) {
        if (criteria_.empty()) {
            criteria_ = "Hold for " + std::to_string(targetFrames_) + " frames";
        }
    }

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
    ConditionalDuration(double targetDuration, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : Conditional(nullptr, isRequired, std::move(name), std::move(criteriaText)), targetDuration_(targetDuration) {
        if (criteria_.empty()) {
            criteria_ = "Survive / run for " + std::to_string(targetDuration_) + " seconds";
        }
    }

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
    ConditionalValueGreaterThan(std::shared_ptr<Telemetry> tel, double threshold, bool isRequired = true, std::string name = "", std::string criteriaText = "")
        : Conditional(std::move(tel), isRequired, std::move(name), std::move(criteriaText)), threshold_(threshold) {
        if (criteria_.empty()) {
            criteria_ = "Value must be greater than " + std::to_string(threshold_);
        }
    }

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

    std::string failureReason() const override {
        if (!hasPassed_) {
            double lastVal = tel_ ? tel_->lastValue() : 0.0;
            return "Value " + std::to_string(lastVal) + " was not greater than " + std::to_string(threshold_);
        }
        return "";
    }
private:
    double threshold_;
};

// ===========================================================================
// ManeuverScenario — declarative, flat simulation
// ===========================================================================
class ManeuverScenario {
public:
    explicit ManeuverScenario(std::string name) : scenarioName_(std::move(name)) {}
    virtual ~ManeuverScenario() = default;

    const std::string& name() const { return scenarioName_; }

    virtual std::string GetDescription() const = 0;
    virtual std::string GetTestGroup() const { return "General"; }
    virtual TestTier GetTestTier() const { return TestTier::LowLevel; }
    virtual std::string GetTestLevel() const { return testTierName(GetTestTier()); }

    double maxTime() const { return maxTime_; }
    void setMaxTime(double t) { maxTime_ = t; }

    void SetDefaultAircraftPath(std::string path) { defaultAircraftPath_ = std::move(path); }
    const std::string& defaultAircraftPath() const { return defaultAircraftPath_; }

    // StartScenario replaces Init / buildSequence.
    virtual void StartScenario(const std::string& defaultAircraftPath) = 0;

    // Optional per-frame custom scenario update hook.
    virtual void UpdateScenario(double /*dt*/) {}

    virtual std::vector<TraceGeometry> traceGeometry() const { return {}; }

    std::shared_ptr<SimulatedAircraft> CreateAircraft(const std::string& name, const std::string& configPath = "") {
        auto ac = std::make_shared<SimulatedAircraft>(name);

        // Resolve JSON configuration file path. If empty, fallback to the defaultAircraftPath_.
        std::string path = configPath;
        if (path.empty()) {
            path = defaultAircraftPath_;
        }

        AircraftConfig cfg;
        auto result = json::readFile(path, cfg);
        if (!result.ok) {
            std::fprintf(stderr, "Error: CreateAircraft failed to load config from %s\n", path.c_str());
            return nullptr;
        }

        // Initialize flight model with standard values.
        const double initCs = cfg.geometry.cornerVcas_kts > 0 ? cfg.geometry.cornerVcas_kts : 330.0;
        ac->fm.init(cfg, 10000.0, initCs * KNOTS_TO_FTPSEC, 0.0, true);

        // Pre-initialize config to match standard expectations
        ac->sc.setCornerSpeed(initCs);
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
    double maxTime_{180.0};
    std::string defaultAircraftPath_;
    std::vector<std::shared_ptr<SimulatedAircraft>> aircraftList_;
    std::vector<std::shared_ptr<Telemetry>> telemetryList_;
    std::vector<std::shared_ptr<Conditional>> conditionalList_;
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
