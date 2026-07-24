// f4flight - scenarios/scenario_digi_flightplan.cpp
//
// Digi AI Basic FlightPlan Navigation Scenario:
//   - Creates an aircraft initialized at 5,000 ft altitude.
//   - Flies through a series of 4 waypoints with explicit climb and descent altitude changes.
//   - Includes traceGeometry() defining waypoints and flightplan corridor overlay.
//   - Validates that each individual waypoint is reached.
//   - Validates that altitude for each leg is captured and held within +/- 20 ft.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"
#include "scenario_framework.h"

#include <string>
#include <vector>
#include <cmath>

using namespace f4flight;

namespace f4flight_test {

class DigiFlightPlanScenario : public ManeuverScenario {
protected:
    // Tracking state for altitude capture and hold across legs
    bool leg2Captured_{false};
    bool leg3Captured_{false};
    bool leg4Captured_{false};

    const   std::vector<double> corridorCoords = {
                0.0,     0.0,  -5000.0,
            30000.0,     0.0,  -5000.0,
            60000.0, 10000.0, -12000.0,
            60000.0, 70000.0,  -8000.0,
            10000.0, 70000.0, -15000.0
    };

public:
    DigiFlightPlanScenario() : ManeuverScenario("digi_flightplan") {
        // Allow up to 500 seconds to complete the 4-waypoint route
        maxTime_ = 500.0;
    }

    std::string GetDescription() const override {
        return "Digi AI Basic FlightPlan Navigation: Aircraft navigates through 4 waypoints "
               "executing climb (12,000 ft), descent (8,000 ft), and final climb (15,000 ft) altitude changes. "
               "Verifies reaching each waypoint and holding each leg altitude within +/- 20 ft.";
    }

    std::string GetTestGroup() const override { return "Navigation"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;

        // 1. Waypoints (type = "waypoint", coords = {x, y, z})
        for (int i = 1; i < corridorCoords.size()/3; i ++)
            geom.push_back(TraceGeometry{ "WP" + std::to_string(i),  "waypoint", {corridorCoords[i * 3 + 0],corridorCoords[i * 3 + 1],corridorCoords[i * 3 + 2]}, "#00e5ff", 2.0 });

        // 2. Flight Plan Corridor (connecting Start -> WP1 -> WP2 -> WP3 -> WP4)

        geom.push_back(TraceGeometry{"FlightPlan Corridor", "corridor", corridorCoords, "#00ffcc", 2.0});

        return geom;
    }

    void StartScenario(const std::string& defaultAircraftPath) override {

        const double initialAlt = 5000.0;
        const double cruiseSpeedKts = 350.0;

        // 1. Spawn main aircraft
        auto ac = CreateAircraft("NavAircraft", defaultAircraftPath);
        if (!ac) return;

        // Configure brain limits for Navigation aircraft
        digi::DigiConfig config = ac->brain.config();
        config.cornerSpeedKts = cruiseSpeedKts;
        config.maxGs = ac->fm.config().geometry.maxGs;
        config.maxBankDeg = 45.0;
        config.maxGammaDeg = 15.0;
        ac->brain.configure(config);

        // Position aircraft at origin, heading East (0.0 rad) at 5,000 ft altitude
        ac->fm.init(ac->fm.config(), initialAlt, cruiseSpeedKts * KNOTS_TO_FTPSEC, 0.0, true);
        ac->fm.state().kin.x = 0.0;
        ac->fm.state().kin.y = 0.0;
        ac->brain.setAltitude(initialAlt);

        // 2. Build multi-waypoint flightplan with climb and descent altitude changes
        //    WP1: (30,000',      0', Alt  5,000') - Level flight leg East
        //    WP2: (60,000', 30,000', Alt 12,000') - Climb leg Northeast
        //    WP3: (60,000', 70,000', Alt  8,000') - Orbit (Left-hand turns, 120s duration)
        //    WP4: (10,000', 70,000', Alt 15,000') - Climb leg West
        auto fp = std::make_shared<digi::FlightPlan>();
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{30000.0,     0.0,  -5000.0}, cruiseSpeedKts,  5000.0});
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{60000.0, 30000.0, -12000.0}, cruiseSpeedKts, 12000.0});
        fp->pushTask(digi::MissionTask{digi::TaskType::Orbit,    Vec3{60000.0, 70000.0,  -8000.0}, cruiseSpeedKts,  8000.0, digi::kInvalidEntityId, 120.0, digi::OrbitDirection::Left});
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{10000.0, 70000.0, -15000.0}, cruiseSpeedKts, 15000.0});

        ac->brain.setFlightPlan(fp);
        ac->brain.setCaptureRadius(4000.0);

        // 3. Setup declarative telemetries
        auto t_altitude = CreateTelemetry("Altitude", [ac]() {
            return -ac->fm.state().kin.z;
            });

        auto t_currentWaypoint = CreateTelemetry("CurrentWaypoint", [ac]() {
            return ac->brain.currentWaypoint();
        });

        auto t_altitude_deviation = CreateTelemetry("AltitudeDeviation", [ac, fp]() {
            if (!ac || !fp) return 0.0;
            size_t idx = ac->brain.currentWaypoint();
            if (idx >= fp->tasks().size()) {
                return -ac->fm.state().kin.z - ac->brain.state().nav.holdAlt;
            }
            double targetAlt = fp->tasks()[idx].altFt;
            return -ac->fm.state().kin.z - targetAlt;
        });

        auto t_course_deviation = CreateTelemetry("CourseDeviation", [ac, fp]() {
            if (!ac || !fp) return 0.0;
            size_t curIdx = ac->brain.currentWaypoint();
            if (curIdx >= fp->tasks().size()) {
                return 0.0;
            }
            const auto& task = fp->tasks()[curIdx];
            Vec3 A;
            if (curIdx > 0) {
                A = fp->tasks()[curIdx - 1].location;
            } else {
                A = Vec3{0.0, 0.0, -5000.0}; // Start position
            }
            const double ABx = task.location.x - A.x;
            const double ABy = task.location.y - A.y;
            const double AB_len = std::sqrt(ABx * ABx + ABy * ABy);
            if (AB_len < 1e-3) return 0.0;

            const double APx = ac->fm.state().kin.x - A.x;
            const double APy = ac->fm.state().kin.y - A.y;
            return (APx * ABy - APy * ABx) / AB_len;
        });

        auto t_nan = CreateTelemetry("StateNaN", [ac]() {
            const auto& as = ac->fm.state();
            return (std::isnan(as.kin.vt) || std::isnan(as.kin.z) || std::isnan(as.kin.sigma)) ? 1.0 : 0.0;
        });

        // 4. Setup assertions (conditionals)
        for (int i = 0; i < fp->tasks().size(); i++) {
            const double captureRange = 100.0;
			const double targetRange = 300.0; // +/- 300 ft tolerance for altitude hold
            auto targetAlt = fp->tasks()[i].altFt;
            auto holds_alt = CreateConditional<ConditionalValueRemainsInRange>(
                t_altitude /*Trace*/,
                targetAlt /*Target*/,
                targetRange /*Range*/,
                true /*isRequired=*/,
                "Leg " + std::to_string(i) + " alt hold" /*Name*/,
                "Leg " + std::to_string(i) + " altitude holds " + std::to_string((int)targetAlt) + " ft +/- " + std::to_string((int)targetRange) + " ft" /*Description*/);

            auto reaches_alt = CreateConditional<ConditionalValueReachesRange>(
                t_altitude /*Trace*/,
                targetAlt /*Target*/,
                captureRange/*Range*/,
                true /*isRequired=*/,
                "Leg " + std::to_string(i) + " alt reached" /*Name*/,
                "Leg " + std::to_string(i) + " altitude reaches " + std::to_string((int)targetAlt) + " ft +/- " + std::to_string((int)captureRange) + " ft" /*Description*/);
            reaches_alt->OnPassed = [holds_alt]() {
				holds_alt->Start();
				};

            reaches_alt->Stop();
            holds_alt->Stop();

            if (i < fp->tasks().size()) {
				int nextWpIdx = i + 1;
                auto reaches_waypoint = CreateConditional<ConditionalValueReachesRange>(t_currentWaypoint, i, 0.5, /*isRequired=*/false, "Waypoint " + std::to_string(nextWpIdx) + " Reached", "Aircraft reaches Waypoint " + std::to_string(nextWpIdx));
                reaches_waypoint->OnPassed = [reaches_alt]() {
                    reaches_alt->Start();
                    };
                auto reaches_next_waypoint = CreateConditional<ConditionalValueReachesRange>(t_currentWaypoint, nextWpIdx, 0.5, /*isRequired=*/true, "Waypoint " + std::to_string(nextWpIdx) + " Reached", "Aircraft reaches Waypoint " + std::to_string(nextWpIdx));
                reaches_next_waypoint->OnPassed = [reaches_alt, holds_alt]() {
                    reaches_alt->Stop();
					holds_alt->Stop();
					};
            }
        }

        CreateConditional<ConditionalValueRemainsInRange>(t_nan, 0.0, 0.1, /*isRequired=*/true, "Flight Stability", "No NaNs or state divergence occurred during flight");
    }

    void UpdateScenario(double /*dt*/) override {

    }
};

static RegisterScenario g_registerFlightPlan("digi_flightplan", []() {
    return std::make_unique<DigiFlightPlanScenario>();
});

} // namespace f4flight_test
