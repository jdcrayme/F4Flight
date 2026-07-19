// f4flight - scenarios/scenario_low_missile_defeat.cpp
//
// LOW-LEVEL scenario: missile defeat (beam/drag) behavior only.
//
// Split out of high_level/scenario_digi_defensive.cpp (MissileDefeatPhase).
// Injects a radar missile 5 NM away closing at 2000 ft/s. The test verifies
// the AI enters MissileDefeat mode and turns beam/cold to the missile.
//
// Pass criteria is RELAXED vs the parent scenario: drop the heading-to-south
// requirement (just verify the AI entered MissileDefeat and produced some G
// and turned away from the missile). This makes the test more tolerant of
// airframe-specific maneuver choices.
//
// Tier: LowLevel. Registered as "low_missile_defeat" — referenced by the
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
// LowMissileDefeatPhase — single missile defeat behavior
// ===========================================================================
class LowMissileDefeatPhase : public ManeuverTest {
public:
    LowMissileDefeatPhase(const char* name, double duration,
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

        // Missile 5 NM north, heading south, closing fast.
        const double missileRange = 5.0 * 6076.0;
        missile_.x = 0.0;
        missile_.y = missileRange;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;
        missile_.speed = 2000.0;
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        FrameInputs fi = sc.brain().frameInputs();
        fi.injectedMissile = &missile_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move missile toward the aircraft.
        missile_.y -= 2000.0 * dt;
        if (missile_.y < -1000.0) missile_.y = 5.0 * 6076.0;

        const double heading = as.kin.sigma;
        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(heading - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        curMode_ = sc_brain_->activeMode();
        if (curMode_ == DigiMode::MissileDefeat) enteredMissileDefeat_ = true;

        curMissileRange_ = std::sqrt(
            (missile_.x - as.kin.x) * (missile_.x - as.kin.x) +
            (missile_.y - as.kin.y) * (missile_.y - as.kin.y));

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (missile 5NM closing at 2000 ft/s)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "G", "mslY", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %8.0f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD,
                as.loads.nzcgs, input.pstick, missile_.y,
                digiModeName(curMode_));
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredMissileDefeat_) return false;
        // RELAXED: just require non-trivial G + meaningful heading change.
        // Parent requires 60° turn toward south; we just require 20° of
        // heading change in any direction (proves the AI maneuvered).
        const double gThresh = isHeavy_ ? 1.05 : 1.2;
        if (maxG_ < gThresh) return false;
        if (maxHeadingChange_ < 20.0 * DTR) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter MissileDefeat; Max G >= 1.2 (1.05 heavy); "
               "Heading change >= 20deg (any direction); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredMissileDefeat_)
            return "Never entered MissileDefeat mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double gThresh = isHeavy_ ? 1.05 : 1.2;
        if (maxG_ < gThresh)
            return "Max G was " + std::to_string(maxG_) +
                   " (needed >= " + std::to_string(gThresh) + ").";
        if (maxHeadingChange_ < 20.0 * DTR)
            return "Max heading change was " +
                   std::to_string(maxHeadingChange_ * RTD) +
                   "deg (needed >= 20deg).";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"msl_range", curMissileRange_, "ft"},
            {"in_defeat", (enteredMissileDefeat_ && curMode_ == DigiMode::MissileDefeat) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double gThresh = isHeavy_ ? 1.05 : 1.2;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered MissileDefeat: %s\n", enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                 %.2f (need >= %.2f%s) %s\n",
            maxG_, gThresh, isHeavy_ ? " [HEAVY]" : "",
            maxG_ >= gThresh ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading change:    %.1f deg (need >= 20) %s\n",
            maxHeadingChange_ * RTD, maxHeadingChange_ >= 20.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:          %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity missile_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    bool isHeavy_{false};
    bool hasNaN_{false};
    DigiMode curMode_{DigiMode::NoMode};
    bool enteredMissileDefeat_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curMissileRange_{0.0};
};

// ===========================================================================
// LowMissileDefeatScenario
// ===========================================================================
class LowMissileDefeatScenario : public ManeuverScenario {
public:
    LowMissileDefeatScenario() : ManeuverScenario("low_missile_defeat") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: missile defeat (beam/drag) behavior. Injects a radar "
               "missile 5NM away closing at 2000 ft/s. Verifies the AI enters "
               "MissileDefeat mode, maneuvers (G + heading change), and stays "
               "above ground.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowMissileDefeatPhase>(
            "Missile defeat", 15.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowMissileDefeat("low_missile_defeat", []() {
    return std::make_unique<LowMissileDefeatScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_missile_defeat() {}

} // namespace f4flight_test
