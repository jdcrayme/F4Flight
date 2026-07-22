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
        
        const double targetAlt = 10000.0;
        const double targetSpd = 350.0;
        
        double leg = 10*NM_TO_FT;
        Vec3 wp1{ leg, 0.0, -targetAlt };
        Vec3 wp2{ leg, leg, -targetAlt };
        Vec3 wp3{ 0.0, leg, -targetAlt };
        Vec3 wp4{ 0.0, 0.0, -targetAlt };

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

        virtual std::vector<TraceGeometry> traceGeometry() const {
            return { { "", "waypoint", { wp1.x, wp1.y, wp1.z }, "#FFFFFF", 0.0 },
                     { "", "waypoint", { wp2.x, wp2.y, wp2.z }, "#FFFFFF", 0.0 },
                     { "", "waypoint", { wp3.x, wp3.y, wp3.z }, "#FFFFFF", 0.0 },
                     { "", "waypoint", { wp4.x, wp4.y, wp4.z }, "#FFFFFF", 0.0 },
                     { "", "corridor", { wp1.x, wp1.y, wp1.z,
                                         wp2.x, wp2.y, wp2.z,
                                         wp3.x, wp3.y, wp3.z,
										 wp4.x, wp4.y, wp4.z }, "#8a90a6", 1.0 } };
        }

        void StartScenario(const std::string& defaultAircraftPath) override {


            // 1. Spawn our primary simulated aircraft
            auto ac = CreateAircraft("F16", defaultAircraftPath);
            if (!ac) return;



            // Define a 10 NM leg square flight path (x = East, y = North) using FlightPlan
            auto flightPlan = digi::FlightPlan::fromWaypoints({ wp1,wp2,wp3,wp4 }, targetSpd);
            ac->sc.brain().setFlightPlan(flightPlan);

            // Insert a refuel task at waypoint 2 (between leg,leg and 0,leg)
            // The tanker will orbit at the midpoint between these two waypoints
            Vec3 refuelLocation{ leg * 0.5, leg, -targetAlt };
            digi::MissionTask refuelTask{ digi::TaskType::Refuel, refuelLocation, targetSpd, targetAlt };
            flightPlan->insertEmergencyTask(refuelTask);

            ac->sc.brain().setFlightPlan(flightPlan);

            // 2. Spawn a tanker aircraft for aerial refueling
            auto tanker = CreateAircraft("KC135", defaultAircraftPath);
            if (!tanker) return;

            // Position tanker at the refuel location (midpoint between waypoints 2 and 3)
            tanker->fm.state().kin.x = refuelLocation.x;
            tanker->fm.state().kin.y = refuelLocation.y;
            tanker->fm.state().kin.z = refuelLocation.z;
            tanker->fm.state().vcas = targetSpd;
            tanker->fm.state().kin.psi = 90.0 * PI/180.0;  // Flying east

            // Tanker flight plan: orbit/loiter at the refuel location
            auto tankerFlightPlan = std::make_shared<digi::FlightPlan>();
            // Loiter task at the refuel location for extended duration
            digi::MissionTask loiterTask{ digi::TaskType::Refuel, refuelLocation, targetSpd, targetAlt, 300.0 };  // 5 minute loiter
            tankerFlightPlan->pushTask(loiterTask);
            tanker->sc.brain().setFlightPlan(tankerFlightPlan);

            // 3. Setup declarative telemetries
            // 
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
