// f4flight - scenarios/scenario_combat.cpp
//
// Combat scenario placeholder. ACM not yet ported.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <string>

using namespace f4flight;

namespace manuver_test {

class CombatTest : public ManeuverTest {
public:
    CombatTest(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        double cs = fm.config().geometry.cornerVcas_kts;
        sc.setCornerSpeed(cs > 0 ? cs : 330.0);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("%6s %8.0f %8.1f %6.1f %5.2f\n",
                    "t(s)", "alt(ft)", "vcas", "bank(d)", "G");
            }
            std::printf("%6.0f %8.0f %8.1f %6.1f %5.2f\n",
                phaseTime_, -as.kin.z, as.vcas,
                as.kin.phi * RTD, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override { return phaseTime_ >= maxTime_; }
    bool IsPassed() const override { return phaseTime_ >= maxTime_; }
    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  (Placeholder — manual inspection)\n");
    }

private:
    double alt_;
    double speed_;
    double nextPrint_{0.0};
};

class CombatScenario : public ManeuverScenario {
public:
    CombatScenario() : ManeuverScenario("combat") {}

    std::string GetDescription() const override {
        return "Combat placeholder — ACM not yet ported. Runs level flight.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = 420.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<CombatTest>("Combat placeholder (level hold)",
            60.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerCombat("combat", []() {
    return std::make_unique<CombatScenario>();
});

extern "C" void f4flight_forceLink_scenario_combat() {}

} // namespace manuver_test
