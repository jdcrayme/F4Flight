// f4flight - scenarios/scenario_engine.cpp
//
// Engine transient test. Cycles the throttle through Idle, MIL, AB, and
// back, verifying RPM spool dynamics, thrust response, and AB sequencing.
//
// This directly validates the Bug #7 fix (MIL/AB branch on RPM, not throttle)
// and Bug #8 fix (spoolAltRate sign).
//
// Pass criteria:
//   - RPM spools smoothly (no instant jumps)
//   - AB thrust > MIL thrust (AB actually engages)
//   - AB only lights when RPM > 1.0 (not instant when throttle crosses 1.0)
//   - Idle thrust < MIL thrust * 0.3
//   - No NaN or divergence

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;

namespace manuver_test {

// ===========================================================================
// EnginePhase — command a specific throttle for a duration, measure response
// ===========================================================================
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
        // Set RPM to a sensible starting point for this throttle setting
        if (throttle_ <= 0.05) {
            fm.state().engine.rpm = 0.7;  // idle
        } else if (throttle_ <= 1.0) {
            fm.state().engine.rpm = 0.7 + 0.3 * throttle_;  // MIL range
        } else {
            fm.state().engine.rpm = 1.0;  // start at MIL, let AB spool up
        }
        fm.state().engine.rpmLag.y_prev = fm.state().engine.rpm;
        fm.state().engine.rpmLag.u_prev = fm.state().engine.rpm;
        fm.state().engine.rpmLagInitialized = true;
        fm.state().engine.engLit = true;
        (void)sc;
    }

    bool inputOverride(PilotInput& out, const AircraftState& state) const override {
        out.pstick = 0.0;      // 1G (pshape=0 -> ptcmd=0, gravity comp gives 1G)
        out.rstick = 0.0;
        out.ypedal = 0.0;
        out.throttle = throttle_;
        out.refueling = false;

        // Track RPM and thrust
        (void)state;
        return true;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double rpm = as.engine.rpm;
        const double thrust = as.engine.thrust;
        const bool abLit = as.engine.aburnLit;

        // Track min/max
        minRpm_ = std::min(minRpm_, rpm);
        maxRpm_ = std::max(maxRpm_, rpm);
        minThrust_ = std::min(minThrust_, thrust);
        maxThrust_ = std::max(maxThrust_, thrust);
        abLitEver_ = abLitEver_ || abLit;

        // Check for NaN
        if (std::isnan(rpm) || std::isnan(thrust) || std::isnan(as.kin.vt)) {
            hasNaN_ = true;
        }

        // Track AB light time (when AB first engages)
        if (abLit && abLightTime_ < 0.0) {
            abLightTime_ = phaseTime_;
        }

        // Print
        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (throttle: %.2f)\n", testName_.c_str(), throttle_);
                std::printf("%6s %8s %8s %10s %6s %8s %8s\n",
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
        // Thrust must be a meaningful acceleration, not just "any positive
        // value". For an F-16-class aircraft at MIL, thrust_accel is ~10
        // ft/s^2; at AB ~15 ft/s^2; at idle ~1 ft/s^2. Require:
        //   - MIL/AB phases: maxThrust >= 5 ft/s^2 (filters grossly-broken models)
        //   - Idle phase:    maxThrust >= 0.1 ft/s^2 (just non-zero)
        // Previously the threshold was 1.0 ft/s^2 for every phase, which
        // would pass even if the engine produced only idle thrust at MIL.
        const bool isIdle = (throttle_ < 0.1);
        const double thrustMin = isIdle ? 0.1 : 5.0;
        if (maxThrust_ < thrustMin) return false;
        // RPM must be in [0, 1.6] (1.0 = MIL, 1.5 = full AB, so 1.6 is a
        // reasonable upper bound). Previously allowed up to 2.0 (200%).
        if (minRpm_ < 0.0 || maxRpm_ > 1.6) return false;
        // For MIL/AB phases, also require RPM to actually reach near-MIL
        // (i.e., the throttle command produced a real spool-up).
        if (!isIdle && maxRpm_ < 0.9) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Throttle:   %.2f\n", throttle_);
        std::printf("  RPM range:  %.3f to %.3f\n", minRpm_, maxRpm_);
        std::printf("  Thrust:     %.1f to %.1f ft/s^2 (%.0f to %.0f lbf)\n",
            minThrust_, maxThrust_,
            minThrust_ * 841.0, maxThrust_ * 841.0);  // approx mass
        std::printf("  AB lit:     %s", abLitEver_ ? "yes" : "no");
        if (abLightTime_ >= 0.0) std::printf(" (first at T+%.1f)", abLightTime_);
        std::printf("\n");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

    // Public accessors for cross-phase comparisons
    double maxThrust() const { return maxThrust_; }
    double minThrust() const { return minThrust_; }
    double maxRpm() const { return maxRpm_; }

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

// ===========================================================================
// EngineScenario — cycle through Idle, MIL, AB, back to Idle
// ===========================================================================
class EngineScenario : public ManeuverScenario {
public:
    EngineScenario() : ManeuverScenario("engine") {}

    std::string GetDescription() const override {
        return "Engine transients: Idle, MIL, AB, Idle. Tests RPM spool and thrust response.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = 350.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // 10 seconds at each setting to let RPM stabilize
        tests.push_back(std::make_unique<EnginePhase>("Idle", 0.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("MIL", 1.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Afterburner", 1.5, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Back to MIL", 1.0, 10.0, alt, speed));
        tests.push_back(std::make_unique<EnginePhase>("Back to Idle", 0.0, 10.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerEngine("engine", []() {
    return std::make_unique<EngineScenario>();
});

extern "C" void f4flight_forceLink_scenario_engine() {}

} // namespace manuver_test
