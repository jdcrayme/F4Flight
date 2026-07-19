// f4flight - scenarios/scenario_low_guns_jink.cpp
//
// LOW-LEVEL scenario: guns jink defensive behavior only.
//
// Split out of high_level/scenario_digi_defensive.cpp (GunsJinkPhase).
// Injects a guns threat 3000 ft ahead firing. The test verifies the AI
// enters GunsJink mode, rolls, and pulls G.
//
// Pass criteria is RELAXED vs the parent scenario: parent requires 25°
// of bank change; we require 15° (any direction). Parent requires maxG >=
// 1.8 (1.05 heavy); we keep that since it's the minimum to distinguish
// "actually jinked" from "drifted in level flight".
//
// Tier: LowLevel. Registered as "low_guns_jink" — referenced by the
// cascade mapping table g_highToLow["high_defensive_chain"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowGunsJinkPhase — single guns jink behavior
// ===========================================================================
class LowGunsJinkPhase : public ManeuverTest {
public:
    LowGunsJinkPhase(const char* name, double duration,
                     double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Guns threat 3000 ft ahead, pointing south at us, firing.
        threat_.x = 0.0;
        threat_.y = 3000.0;
        threat_.z = -alt_;
        threat_.yaw = -PI / 2.0;
        threat_.isFiring = true;
        threat_.isDead = false;

        FrameInputs fi = sc.brain().frameInputs();
        fi.injectedGunsThreat = &threat_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
        initialBank_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        threat_.y = 3000.0;  // static for simplicity

        const double bank = as.kin.phi;
        maxBankChange_ = std::max(maxBankChange_, std::fabs(bank - initialBank_));
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        if (sc_brain_->activeMode() == DigiMode::GunsJink) enteredGunsJink_ = true;

        curMode_ = sc_brain_->activeMode();
        curBank_ = bank * RTD;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (guns threat 3000ft, firing)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "bank(d)", "G", "pstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, bank * RTD,
                as.loads.nzcgs, input.pstick, digiModeName(curMode_));
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredGunsJink_) return false;
        // RELAXED: parent requires 25° bank change; we require 15°.
        if (maxBankChange_ < 15.0 * DTR) return false;
        const double gThresh = isHeavy_ ? 1.05 : 1.8;
        if (maxG_ < gThresh) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GunsJink mode; Max bank change >= 15deg; "
               "Max G >= 1.8 (1.05 heavy); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGunsJink_)
            return "Never entered GunsJink mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (maxBankChange_ < 15.0 * DTR)
            return "Max bank change was " + std::to_string(maxBankChange_ * RTD) +
                   "deg (needed >= 15deg).";
        const double gThresh = isHeavy_ ? 1.05 : 1.8;
        if (maxG_ < gThresh)
            return "Max G was " + std::to_string(maxG_) +
                   " (needed >= " + std::to_string(gThresh) + ").";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"bank",    curBank_, "deg"},
            {"G",       maxG_,    ""},
            {"in_jink", (enteredGunsJink_ && curMode_ == DigiMode::GunsJink) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double gThresh = isHeavy_ ? 1.05 : 1.8;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GunsJink:    %s\n", enteredGunsJink_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max bank change:     %.1f deg (need >= 15) %s\n",
            maxBankChange_ * RTD, maxBankChange_ >= 15.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:               %.2f (need >= %.2f%s) %s\n",
            maxG_, gThresh, isHeavy_ ? " [HEAVY]" : "",
            maxG_ >= gThresh ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity threat_;
    double nextPrint_{0.0};
    double initialBank_{0.0};
    double maxBankChange_{0.0};
    double maxG_{0.0};
    double minAlt_{1e9};
    bool isHeavy_{false};
    bool hasNaN_{false};
    bool enteredGunsJink_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curBank_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowGunsJinkScenario
// ===========================================================================
class LowGunsJinkScenario : public ManeuverScenario {
public:
    LowGunsJinkScenario() : ManeuverScenario("low_guns_jink") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: guns jink defensive behavior. Injects a guns threat 3000ft "
               "ahead firing. Verifies the AI enters GunsJink mode, rolls, "
               "pulls G, and stays above ground.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowGunsJinkPhase>(
            "Guns jink", 8.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowGunsJink("low_guns_jink", []() {
    return std::make_unique<LowGunsJinkScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_guns_jink() {}

} // namespace f4flight_test
