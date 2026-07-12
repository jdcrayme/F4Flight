// f4flight - scenarios/scenario_engine.cpp
//
// Engine transient test. Directly commands throttle via inputOverride() —
// BYPASSES the AI steering controller entirely.
//
// Tests the engine model (RPM spool, MIL/AB branching, thrust tables), NOT
// the digi AI. Renamed from "engine" to "fcs_engine" for consistency with
// the other FCS-direct scenarios.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;

namespace manuver_test {

class EnginePhase : public ManeuverTest {
public:
    EnginePhase(const char* name, double throttle, double duration,
                double alt, double speed)
        : ManeuverTest(name, duration)
        , throttle_(throttle)
        , alt_(alt)
        , speed_(speed)
    {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        if (throttle_ <= 0.05) {
            fm.state().engine.rpm = 0.7;
        } else if (throttle_ <= 1.0) {
            fm.state().engine.rpm = 0.7 + 0.3 * throttle_;
        } else {
            fm.state().engine.rpm = 1.0;
        }
        fm.state().engine.rpmLag.y_prev = fm.state().engine.rpm;
        fm.state().engine.rpmLag.u_prev = fm.state().engine.rpm;
        fm.state().engine.rpmLagInitialized = true;
        fm.state().engine.engLit = true;
        (void)sc;
    }

    bool inputOverride(PilotInput& out, const AircraftState& state) const override {
        out.pstick = 0.0;
        out.rstick = 0.0;
        out.ypedal = 0.0;
        out.throttle = throttle_;
        out.refueling = false;
        (void)state;
        return true;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double rpm = as.engine.rpm;
        const double thrust = as.engine.thrust;
        const bool abLit = as.engine.aburnLit;

        minRpm_ = std::min(minRpm_, rpm);
        maxRpm_ = std::max(maxRpm_, rpm);
        minThrust_ = std::min(minThrust_, thrust);
        maxThrust_ = std::max(maxThrust_, thrust);
        abLitEver_ = abLitEver_ || abLit;

        if (std::isnan(rpm) || std::isnan(thrust) || std::isnan(as.kin.vt)) {
            hasNaN_ = true;
        }

        if (abLit && abLightTime_ < 0.0) {
            abLightTime_ = phaseTime_;
        }

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (throttle: %.2f)\n", testName_.c_str(), throttle_);
                std::printf("%6s %8s %8s %10s %4s %8s %8s\n",
                    "t(s)", "rpm", "thrust", "thrust(lbf)", "AB", "vt(kts)", "fuel(lb)");
            }
            const double thrust_lbf = thrust * as.fuel.mass_slugs;
            std::printf("%6.1f %8.3f %8.1f %10.0f %6s %8.1f %8.0f\n",
                phaseTime_, rpm, thrust, thrust_lbf,
                abLit ? "ON" : "",
                as.kin.vt * FTPSEC_TO_KNOTS, as.fuel.fuel_lbs);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        const bool isIdle = (throttle_ < 0.1);
        if (isIdle) {
            if (std::fabs(maxThrust_) < 0.1 && std::fabs(minThrust_) < 0.1)
                return false;
        } else {
            const double thrustMin = 5.0;
            if (maxThrust_ < thrustMin) return false;
        }
        if (minRpm_ < 0.0 || maxRpm_ > 1.6) return false;
        if (!isIdle && maxRpm_ < 0.9) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Throttle:   %.2f\n", throttle_);
        std::printf("  RPM range:  %.3f to %.3f\n", minRpm_, maxRpm_);
        std::printf("  Thrust:     %.1f to %.1f ft/s^2 (%.0f to %.0f lbf)\n",
            minThrust_, maxThrust_,
            minThrust_ * 841.0, maxThrust_ * 841.0);
        std::printf("  AB lit:     %s", abLitEver_ ? "yes" : "no");
        if (abLightTime_ >= 0.0) std::printf(" (first at T+%.1f)", abLightTime_);
        std::printf("\n");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double throttle_;
    double alt_;
    double speed_;

    double minRpm_{std::numeric_limits<double>::max()};
    double maxRpm_{std::numeric_limits<double>::lowest()};
    double minThrust_{std::numeric_limits<double>::max()};
    double maxThrust_{std::numeric_limits<double>::lowest()};
    double abLightTime_{-1.0};
    double nextPrint_{0.0};
    bool abLitEver_{false};
    bool hasNaN_{false};
};

class EngineScenario : public ManeuverScenario {
public:
    EngineScenario() : ManeuverScenario("fcs_engine") {}

    std::string GetDescription() const override {
        return "Engine transients: Idle, MIL, AB, Idle. Bypasses AI; tests "
               "RPM spool and thrust response directly.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = 350.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<EnginePhase>("Idle", 0.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("MIL", 1.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Afterburner", 1.5, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Back to MIL", 1.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Back to Idle", 0.0, 10.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerEngine("fcs_engine", []() {
    return std::make_unique<EngineScenario>();
});

extern "C" void f4flight_forceLink_scenario_fcs_engine() {}

} // namespace manuver_test
