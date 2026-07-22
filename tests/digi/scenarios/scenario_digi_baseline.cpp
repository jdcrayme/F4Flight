// f4flight - scenarios/scenario_digi_baseline.cpp
//
// Digi AI Baseline Integration Scenario: Waypoint Navigation and Cruise.
// Re-written as a clean, flat, declarative conditional-driven scenario.

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <string>
#include <vector>
#include <cmath>

using namespace f4flight;

namespace f4flight_test {

class DigiBaselineScenario : public ManeuverScenario {
public:
    DigiBaselineScenario() : ManeuverScenario("digi_baseline") {
        // Run for up to 350 seconds
        maxTime_ = 350.0;
    }

    std::string GetDescription() const override {
        return "Digi AI Baseline: Cruise, Waypoint Navigation, and Altitude Hold. "
               "Flies a 4-waypoint square flight plan, captures waypoints, and validates navigation stability.";
    }

    std::string GetTestGroup() const override { return "Baseline"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    void StartScenario(const std::string& defaultAircraftPath) override {
        const double targetAlt = 10000.0;
        const double targetSpd = 350.0;

        // 1. Spawn our primary simulated aircraft
        auto ac = CreateAircraft("F16", defaultAircraftPath);
        if (!ac) return;

        // Configure autopilot limits
        ac->sc.setCornerSpeed(targetSpd);
        ac->sc.setMaxGs(ac->fm.config().geometry.maxGs);
        ac->sc.setMaxBank(45.0);
        ac->sc.setMaxGamma(15.0);
        ac->sc.setTurnG(2.0);
        ac->sc.setAltitude(targetAlt);

        // Define a 10 NM leg square flight path (x = East, y = North)
        double leg = 60000.0;
        std::vector<Vec3> wps = {
            Vec3{leg, 0.0, -targetAlt},
            Vec3{leg, leg, -targetAlt},
            Vec3{0.0, leg, -targetAlt},
            Vec3{0.0, 0.0, -targetAlt}
        };
        ac->sc.setWaypoints(wps);
        ac->sc.setCaptureRadius(5000.0);
        ac->sc.setMode(SteeringController::Mode::Waypoint);

        // 2. Setup declarative telemetries
        auto t_alt = CreateTelemetry("Altitude", [ac]() { return -ac->fm.state().kin.z; });
        auto t_spd = CreateTelemetry("Speed", [ac]() { return ac->fm.state().vcas; });
        auto t_wp = CreateTelemetry("CurrentWaypoint", [ac]() { return static_cast<double>(ac->sc.currentWaypoint()); });
        auto t_nan = CreateTelemetry("StateNaN", [ac]() {
            const auto& as = ac->fm.state();
            return (std::isnan(as.kin.vt) || std::isnan(as.kin.z) || std::isnan(as.kin.sigma)) ? 1.0 : 0.0;
        });
        auto t_captured = CreateTelemetry("AllCaptured", [ac]() { return ac->sc.allWaypointsCaptured() ? 1.0 : 0.0; });

        // 3. Setup declarative assertions (conditionals)
        CreateConditional<ConditionalValueReachesRange>(t_captured, 1.0, 0.1, /*isRequired=*/true, "Waypoint Capture", "Capture all 4 route waypoints");
        CreateConditional<ConditionalValueRemainsInRange>(t_nan, 0.0, 0.1, /*isRequired=*/true, "Flight Stability", "No NaNs or state divergence occurred during flight");
        CreateConditional<ConditionalValueRemainsInRange>(t_alt, targetAlt, 500.0, /*isRequired=*/true, "Altitude Hold", "Altitude remains within ±500 ft of target");
        CreateConditional<ConditionalValueRemainsInRange>(t_spd, targetSpd, 100.0, /*isRequired=*/true, "Speed Hold", "Airspeed remains within ±100 kts of target");
    }
};

static RegisterScenario g_registerBaseline("digi_baseline", []() {
    return std::make_unique<DigiBaselineScenario>();
});

} // namespace f4flight_test
