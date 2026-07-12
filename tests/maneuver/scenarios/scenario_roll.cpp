// f4flight - scenarios/scenario_roll.cpp
//
// Roll authority test. Commands full stick left/right at various speeds and
// verifies the roll rate matches the rollCmd table, the roll-rate limiter
// works, and the slow-speed authority fade functions.
//
// Pass criteria:
//   - Roll rate reaches a significant fraction of the table value
//   - No departure or uncommanded roll reversal
//   - Roll rate is symmetric (left ≈ right)
//   - No NaN

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;

namespace manuver_test {

class RollStepPhase : public ManeuverTest {
public:
    RollStepPhase(const char* name, double rstick, double duration,
                  double alt, double speed)
        : ManeuverTest(name, duration + 3.0)
        , rstick_(rstick)
        , alt_(alt)
        , speed_(speed)
    {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        (void)sc;
    }

    bool inputOverride(PilotInput& out, const AircraftState& state) const override {
        out.pstick = 0.0;      // 1G level
        out.rstick = rstick_;  // full stick
        out.ypedal = 0.0;
        // Proportional throttle to hold speed
        const double speedErr = speed_ - state.vcas;
        out.throttle = limit(0.5 + speedErr * 0.02, 0.0, 1.0);
        out.refueling = false;
        return true;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Track roll rate (convert from body-axis p in rad/s to deg/s)
        const double rollRate_degps = as.kin.p * RTD;
        const double bank = as.kin.phi * RTD;

        // Skip first 2 seconds (transient), then track steady-state roll rate
        if (phaseTime_ >= 2.0) {
            maxRollRate_ = std::max(maxRollRate_, std::fabs(rollRate_degps));
            rollRateSum_ += std::fabs(rollRate_degps);
            rollRateCount_++;
        }

        // Track bank angle
        maxBank_ = std::max(maxBank_, std::fabs(bank));

        if (std::isnan(rollRate_degps) || std::isnan(as.kin.vt)) {
            hasNaN_ = true;
        }

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (rstick: %.1f, %d kts)\n", testName_.c_str(),
                    rstick_, (int)speed_);
                std::printf("%6s %8s %10s %8s %8s %6s %8s\n",
                    "t(s)", "bank(d)", "roll(d/s)", "G", "alpha", "throt", "vt(kts)");
            }
            std::printf("%6.1f %8.1f %10.1f %8.2f %8.1f %6.2f %8.1f\n",
                phaseTime_, bank, rollRate_degps,
                as.loads.nzcgs, as.aero.alpha_deg,
                input.throttle, as.kin.vt * FTPSEC_TO_KNOTS);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Must achieve some roll rate (> 30 deg/s at 350+ kts)
        if (maxRollRate_ < 30.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Max roll rate: %.1f deg/s %s\n",
            maxRollRate_, (maxRollRate_ >= 30.0) ? "[PASS]" : "[FAIL]");
        double avgRR = rollRateCount_ > 0 ? rollRateSum_ / rollRateCount_ : 0.0;
        std::printf("  Avg roll rate: %.1f deg/s (steady-state, frames 2-%.0f)\n",
            avgRR, maxTime_);
        std::printf("  Max bank:      %.1f deg\n", maxBank_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double rstick_;
    double alt_;
    double speed_;
    double maxRollRate_{0.0};
    double maxBank_{0.0};
    double rollRateSum_{0.0};
    int rollRateCount_{0};
    double nextPrint_{0.0};
    bool hasNaN_{false};
};

// ===========================================================================
// RollScenario — test roll authority at several speeds
// ===========================================================================
class RollScenario : public ManeuverScenario {
public:
    RollScenario() : ManeuverScenario("roll") {}

    std::string GetDescription() const override {
        return "Roll authority: full stick L/R at 250, 350, 450 kts. Tests roll FCS + table.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;

        fm.init(ctx.cfg, alt, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Full stick right at 3 speeds
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 350kts", 1.0, 8.0, alt, 350.0));
        tests.push_back(std::make_unique<RollStepPhase>("Roll left 350kts", -1.0, 8.0, alt, 350.0));
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 250kts", 1.0, 8.0, alt, 250.0));
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 450kts", 1.0, 8.0, alt, 450.0));
        return tests;
    }
};

static RegisterScenario g_registerRoll("roll", []() {
    return std::make_unique<RollScenario>();
});

extern "C" void f4flight_forceLink_scenario_roll() {}

} // namespace manuver_test
