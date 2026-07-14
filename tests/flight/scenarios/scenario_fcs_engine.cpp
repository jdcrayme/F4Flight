// f4flight - scenarios/scenario_engine.cpp
//
// Engine transient test. Directly commands throttle via inputOverride() —
// BYPASSES the AI steering controller entirely.
//
// Tests the engine model (RPM spool, MIL/AB branching, thrust tables), NOT
// the digi AI. Renamed from "engine" to "fcs_engine" for consistency with
// the other FCS-direct scenarios.

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;

namespace f4flight_test {

class EnginePhase : public ManeuverTest {
public:
    EnginePhase(const char* name, double throttle, double duration,
                double alt, double speed, bool heavy, bool hasAB)
        : ManeuverTest(name, duration)
        , throttle_(throttle)
        , alt_(alt)
        , speed_(speed)
        , isHeavy_(heavy)
        , hasAB_(hasAB)
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
        const bool isAB   = (throttle_ > 1.1);
        if (isIdle) {
            // Idle: thrust must be low AND rpm must drop to near idle (0.7).
            // The old test only checked thrust <= 1.0 — an engine stuck at
            // MIL rpm with low thrust would pass.
            if (maxThrust_ > 1.0) return false;
            if (maxRpm_ > 0.80) return false;  // idle rpm is ~0.7, allow spool drift
        } else {
            // Non-idle thrust threshold scales with aircraft class.
            //   Fighter: T/W ~ 0.6-1.2 → thrust accel >= 5.0 ft/s² at MIL
            //   Heavy  : T/W ~ 0.2-0.3 → thrust accel >= 1.0 ft/s² at MIL
            // Also: aircraft without afterburner (C-130, B-52H, A-10) will
            // produce the same thrust at throttle=1.5 as at 1.0 — that's
            // correct, not a failure.
            const double thrustMin = isHeavy_ ? 1.0 : 5.0;
            if (maxThrust_ < thrustMin) return false;
            // RPM must reach near 1.0 at MIL/AB. The old test accepted 0.9
            // (10% shortfall). Tighten to 0.95.
            if (maxRpm_ < 0.95) return false;
            // AB-specific: afterburner must actually light. The old test
            // tracked abLitEver_ but never asserted it — an AB phase passed
            // identically to a MIL phase. Aircraft without AB (C-130, B-52H,
            // A-10, G-4 Super Galeb) don't light AB; use the config's
            // hasAB flag to waive for them.
            if (isAB && hasAB_ && !abLitEver_) return false;
        }
        if (minRpm_ < 0.0 || maxRpm_ > 1.6) return false;
        return true;
    }

    std::string criteria() const override {
        return "No NaN; RPM stays in [0.0, 1.6]; "
               "Idle: max thrust <= 1.0, max RPM <= 0.80; "
               "MIL/AB: max thrust >= 5 ft/s² (1.0 heavy), max RPM >= 0.95; "
               "AB (if fitted): afterburner lit";
    }

    void Finish() const override {
        const bool isIdle = (throttle_ < 0.1);
        const bool isAB   = (throttle_ > 1.1);
        std::printf("  --- Summary ---\n");
        std::printf("  Throttle:   %.2f%s\n", throttle_,
            isAB ? " [AB]" : (isIdle ? " [IDLE]" : ""));
        std::printf("  RPM range:  %.3f to %.3f\n", minRpm_, maxRpm_);
        std::printf("  Thrust:     %.1f to %.1f ft/s^2\n", minThrust_, maxThrust_);
        if (isAB) {
            if (!hasAB_) {
                std::printf("  AB lit:     (waived — aircraft has no AB fitted)\n");
            } else {
                std::printf("  AB lit:     %s %s\n", abLitEver_ ? "yes" : "no",
                    abLitEver_ ? "[PASS]" : "[FAIL]");
            }
        } else if (isIdle) {
            std::printf("  Idle rpm:   %s (max rpm %.3f <= 0.80)\n",
                maxRpm_ <= 0.80 ? "[PASS]" : "[FAIL]", maxRpm_);
        }
        if (abLightTime_ >= 0.0) std::printf("  AB first lit at T+%.1f\n", abLightTime_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double throttle_;
    double alt_;
    double speed_;
    bool   isHeavy_;
    bool   hasAB_;

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
        const bool heavy = isHeavy(ctx.cfg);

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        const bool hasAB = ctx.cfg.engine.hasAB();
        tests.push_back(std::make_unique<EnginePhase>("Idle", 0.0, 10.0, alt, speed, heavy, hasAB));
        tests.push_back(std::make_unique<EnginePhase>("MIL", 1.0, 10.0, alt, speed, heavy, hasAB));
        tests.push_back(std::make_unique<EnginePhase>("Afterburner", 1.5, 10.0, alt, speed, heavy, hasAB));
        tests.push_back(std::make_unique<EnginePhase>("Back to MIL", 1.0, 10.0, alt, speed, heavy, hasAB));
        tests.push_back(std::make_unique<EnginePhase>("Back to Idle", 0.0, 10.0, alt, speed, heavy, hasAB));
        return tests;
    }
};

static RegisterScenario g_registerEngine("fcs_engine", []() {
    return std::make_unique<EngineScenario>();
});

extern "C" void f4flight_forceLink_scenario_fcs_engine() {}

} // namespace f4flight_test
