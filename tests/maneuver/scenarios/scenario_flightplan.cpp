// f4flight - scenarios/scenario_flightplan.cpp
//
// 4-waypoint square circuit using the FreeFalcon digi AI waypoint following.
// Rewritten for the digi AI steering port.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <string>

using namespace f4flight;

namespace manuver_test {

class FlightplanTest : public ManeuverTest {
public:
    FlightplanTest(const char* name, double duration,
                   std::vector<Vec3> wps, double captureRadius,
                   double alt, double speed)
        : ManeuverTest(name, duration)
        , wps_(std::move(wps)), captureRadius_(captureRadius)
        , alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setWaypoints(wps_);
        sc.setCaptureRadius(captureRadius_);
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
                std::printf("%6s %8.0f %8.0f %8.1f %6.1f %6.1f %5.2f\n",
                    "t(s)", "x(ft)", "y(ft)", "alt(ft)", "bank(d)", "pitch(d)", "G");
            }
            std::printf("%6.0f %8.0f %8.0f %8.1f %6.1f %6.1f %5.2f\n",
                phaseTime_, as.kin.x, as.kin.y, -as.kin.z,
                as.kin.phi * RTD, as.kin.theta * RTD, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override { return phaseTime_ >= maxTime_; }
    bool IsPassed() const override { return phaseTime_ >= maxTime_; }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  (Waypoint circuit - manual inspection)\n");
    }

private:
    std::vector<Vec3> wps_;
    double captureRadius_;
    double alt_;
    double speed_;
    double nextPrint_{0.0};
};

class FlightplanScenario : public ManeuverScenario {
public:
    FlightplanScenario() : ManeuverScenario("flightplan") {}

    std::string GetDescription() const override {
        return "4-waypoint square circuit using FreeFalcon digi AI waypoint following.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 10000.0;
        const double speed = 380.0;
        const double leg = 60000.0;  // 10 NM legs

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        // Square circuit: start at origin, go N -> E -> S -> W -> N
        std::vector<Vec3> wps;
        wps.push_back({0.0,   leg,  -alt});  // North
        wps.push_back({leg,   leg,  -alt});  // East
        wps.push_back({leg,   0.0,  -alt});  // South
        wps.push_back({0.0,   0.0,  -alt});  // West (back to start)

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<FlightplanTest>(
            "4-waypoint circuit", 600.0, std::move(wps), 5000.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerFlightplan("flightplan", []() {
    return std::make_unique<FlightplanScenario>();
});

extern "C" void f4flight_forceLink_scenario_flightplan() {}

} // namespace manuver_test
