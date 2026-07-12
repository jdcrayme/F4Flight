// f4flight - scenarios/scenario_combat.cpp
//
// Previously a placeholder that trivially passed. Now a real level-flight
// test at the README's stated conditions (15,000 ft, 420 kts) — higher and
// faster than scenario_basic's 1,500 ft / 380 kts, so it exercises the
// Mach-dependent aero tables and the thrust/drag balance at a more demanding
// cruise point. Combat (ACM, weapons delivery) remains future work; in the
// meantime this scenario provides useful high-altitude coverage instead of
// just trivially passing.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;

namespace manuver_test {

class LevelHoldTest : public ManeuverTest {
public:
    LevelHoldTest(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double alt = -as.kin.z;
        const double spd = as.vcas;
        altSamples_.emplace_back(phaseTime_, alt);
        spdSamples_.emplace_back(phaseTime_, spd);
        if (std::fabs(alt - targetAlt_) < 200.0 && altCaptureTime_ == 0.0) {
            altCaptureTime_ = phaseTime_;
        }
        if (altCaptureTime_ > 0.0 && std::fabs(spd - targetSpd_) < 25.0 &&
            speedCaptureTime_ == 0.0) {
            speedCaptureTime_ = phaseTime_;
        }
        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("  %6s  %8s  %8s  %8s  %8s  %6s  %6s  %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "spdErr",
                    "bank(d)", "pitch(d)", "G");
            }
            std::printf("  %6.0f  %8.0f  %8.0f  %8.1f  %8.1f  %6.1f  %6.1f  %5.2f\n",
                phaseTime_, alt, targetAlt_ - alt, spd, targetSpd_ - spd,
                as.kin.phi * RTD, as.kin.theta * RTD, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ ||
               (altCaptureTime_ > 0.0 && speedCaptureTime_ > 0.0 &&
                phaseTime_ > std::max(altCaptureTime_, speedCaptureTime_) + 30.0);
    }

    bool IsPassed() const override {
        // Settling window: last 30 s of phase. Tolerances:
        //   ALT_TOL = 200 ft  (high-altitude cruise, Mach-dependent aero)
        //   SPD_TOL = 25 kts
        if (altCaptureTime_ == 0.0 || speedCaptureTime_ == 0.0) return false;
        const double tEnd = phaseTime_;
        const double tStart = tEnd - 30.0;
        double altMin = 1e9, altMax = -1e9, spdMin = 1e9, spdMax = -1e9;
        for (const auto& s : altSamples_) if (s.first >= tStart) { altMin = std::min(altMin, s.second); altMax = std::max(altMax, s.second); }
        for (const auto& s : spdSamples_) if (s.first >= tStart) { spdMin = std::min(spdMin, s.second); spdMax = std::max(spdMax, s.second); }
        return std::fabs(altMax - targetAlt_) < 200.0 && std::fabs(altMin - targetAlt_) < 200.0 &&
               std::fabs(spdMax - targetSpd_) < 25.0  && std::fabs(spdMin - targetSpd_) < 25.0;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        if (altCaptureTime_ > 0.0) {
            std::printf("  Altitude capture at T+%.2f\n", altCaptureTime_);
            if (speedCaptureTime_ > 0.0) {
                std::printf("  Speed capture at T+%.2f\n", speedCaptureTime_);
            } else {
                std::printf("  Speed capture NOT achieved.  [FAIL]\n");
            }
        } else {
            std::printf("  Altitude capture NOT achieved.  [FAIL]\n");
        }
        std::printf("  Result: %s\n", IsPassed() ? "[PASS]" : "[FAIL]");
    }

private:
    double targetAlt_, targetSpd_;
    double nextPrint_{0.0};
    double altCaptureTime_{0.0};
    double speedCaptureTime_{0.0};
    std::vector<std::pair<double,double>> altSamples_;
    std::vector<std::pair<double,double>> spdSamples_;
};

class CombatScenario : public ManeuverScenario {
public:
    CombatScenario() : ManeuverScenario("combat") {}

    std::string GetDescription() const override {
        return "High-altitude level flight (15000 ft, 420 kts) -- exercises "
               "Mach-dependent aero tables. ACM/weapon delivery is future work.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = 420.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LevelHoldTest>(
            "High-alt level hold 15000ft @ 420kts", 180.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerCombat("combat", []() {
    return std::make_unique<CombatScenario>();
});

extern "C" void f4flight_forceLink_scenario_combat() {}

} // namespace manuver_test
