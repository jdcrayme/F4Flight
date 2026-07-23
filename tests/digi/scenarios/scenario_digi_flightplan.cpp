// f4flight - scenarios/scenario_digi_flightplan.cpp
//
// Digi AI Basic FlightPlan Navigation Scenario:
//   - Creates an aircraft initialized at 5,000 ft altitude.
//   - Flies through a series of 4 waypoints with explicit climb and descent altitude changes.
//   - Includes traceGeometry() defining waypoints and flightplan corridor overlay.
//   - Validates that each individual waypoint is reached.
//   - Validates that altitude for each leg is captured and held.

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
    bool leg1HoldPass_{true};

    bool leg2Captured_{false};
    bool leg2HoldPass_{false};

    bool leg3Captured_{false};
    bool leg3HoldPass_{false};

    bool leg4Captured_{false};
    bool leg4HoldPass_{false};

    double maxErrLeg1_{0.0};
    double maxErrLeg2_{0.0};
    double maxErrLeg3_{0.0};
    double maxErrLeg4_{0.0};

public:
    DigiFlightPlanScenario() : ManeuverScenario("digi_flightplan") {
        maxTime_ = 450.0;
    }

    std::string GetDescription() const override {
        return "Digi AI Basic FlightPlan Navigation: Aircraft navigates through 4 waypoints "
               "executing climb (12,000 ft), descent (8,000 ft), and final climb (15,000 ft) altitude changes. "
               "Verifies reaching each waypoint and holding each leg altitude.";
    }

    std::string GetTestGroup() const override { return "Navigation"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;

        // 1. Waypoints (type = "waypoint", coords = {x, y, z})
        geom.push_back(TraceGeometry{"WP1 (5k')",  "waypoint", {30000.0,     0.0,  -5000.0}, "#00e5ff", 2.0});
        geom.push_back(TraceGeometry{"WP2 (12k')", "waypoint", {60000.0, 30000.0, -12000.0}, "#00e5ff", 2.0});
        geom.push_back(TraceGeometry{"WP3 (8k')",  "waypoint", {60000.0, 70000.0,  -8000.0}, "#00e5ff", 2.0});
        geom.push_back(TraceGeometry{"WP4 (15k')", "waypoint", {10000.0, 70000.0, -15000.0}, "#00e5ff", 2.0});

        // 2. Flight Plan Corridor (connecting Start -> WP1 -> WP2 -> WP3 -> WP4)
        std::vector<double> corridorCoords = {
                0.0,     0.0,  -5000.0,
            30000.0,     0.0,  -5000.0,
            60000.0, 30000.0, -12000.0,
            60000.0, 70000.0,  -8000.0,
            10000.0, 70000.0, -15000.0
        };
        geom.push_back(TraceGeometry{"FlightPlan Corridor", "corridor", corridorCoords, "#00ffcc", 2.0});

        return geom;
    }

    void StartScenario(const std::string& defaultAircraftPath) override {
        leg1HoldPass_ = true;
        leg2Captured_ = false;
        leg2HoldPass_ = false;
        leg3Captured_ = false;
        leg3HoldPass_ = false;
        leg4Captured_ = false;
        leg4HoldPass_ = false;

        maxErrLeg1_ = 0.0;
        maxErrLeg2_ = 0.0;
        maxErrLeg3_ = 0.0;
        maxErrLeg4_ = 0.0;

        const double initialAlt = 5000.0;
        const double cruiseSpeedKts = 350.0;

        auto ac = CreateAircraft("NavAircraft", defaultAircraftPath);
        if (!ac) return;

        digi::DigiConfig config = ac->brain.config();
        config.cornerSpeedKts = cruiseSpeedKts;
        config.maxGs = ac->fm.config().geometry.maxGs;
        config.maxBankDeg = 45.0;
        config.maxGammaDeg = 15.0;
        ac->brain.configure(config);

        ac->fm.init(ac->fm.config(), initialAlt, cruiseSpeedKts * KNOTS_TO_FTPSEC, 0.0, true);
        ac->fm.state().kin.x = 0.0;
        ac->fm.state().kin.y = 0.0;
        ac->brain.setAltitude(initialAlt);

        auto fp = std::make_shared<digi::FlightPlan>();
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{30000.0,     0.0,  -5000.0}, cruiseSpeedKts,  5000.0});
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{60000.0, 30000.0, -12000.0}, cruiseSpeedKts, 12000.0});
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{60000.0, 70000.0,  -8000.0}, cruiseSpeedKts,  8000.0});
        fp->pushTask(digi::MissionTask{digi::TaskType::Navigate, Vec3{10000.0, 70000.0, -15000.0}, cruiseSpeedKts, 15000.0});

        ac->brain.setFlightPlan(fp);
        ac->brain.setCaptureRadius(4000.0);

        auto t_wp1_reached = CreateTelemetry("WP1_Reached", [ac]() {
            return (ac->brain.currentWaypoint() >= 1) ? 1.0 : 0.0;
        });

        auto t_wp2_reached = CreateTelemetry("WP2_Reached", [ac]() {
            return (ac->brain.currentWaypoint() >= 2) ? 1.0 : 0.0;
        });

        auto t_wp3_reached = CreateTelemetry("WP3_Reached", [ac]() {
            return (ac->brain.currentWaypoint() >= 3) ? 1.0 : 0.0;
        });

        auto t_wp4_reached = CreateTelemetry("WP4_Reached", [ac]() {
            return (ac->brain.allWaypointsCaptured() || ac->brain.currentWaypoint() >= 4) ? 1.0 : 0.0;
        });

        auto t_leg1_alt_ok = CreateTelemetry("Leg1_Alt_Hold_OK", [this]() { return leg1HoldPass_ ? 1.0 : 0.0; });
        auto t_leg2_alt_ok = CreateTelemetry("Leg2_Alt_Hold_OK", [this]() { return leg2HoldPass_ ? 1.0 : 0.0; });
        auto t_leg3_alt_ok = CreateTelemetry("Leg3_Alt_Hold_OK", [this]() { return leg3HoldPass_ ? 1.0 : 0.0; });
        auto t_leg4_alt_ok = CreateTelemetry("Leg4_Alt_Hold_OK", [this]() { return leg4HoldPass_ ? 1.0 : 0.0; });

        auto t_nan = CreateTelemetry("StateNaN", [ac]() {
            const auto& as = ac->fm.state();
            return (std::isnan(as.kin.vt) || std::isnan(as.kin.z) || std::isnan(as.kin.sigma)) ? 1.0 : 0.0;
        });

        CreateConditional<ConditionalValueGreaterThan>(t_wp1_reached, 0.5, true, "WP1 Reached", "Aircraft reaches Waypoint 1 (30k, 0, 5k')");
        CreateConditional<ConditionalValueGreaterThan>(t_wp2_reached, 0.5, true, "WP2 Reached", "Aircraft reaches Waypoint 2 (60k, 30k, 12k')");
        CreateConditional<ConditionalValueGreaterThan>(t_wp3_reached, 0.5, true, "WP3 Reached", "Aircraft reaches Waypoint 3 (60k, 70k, 8k')");
        CreateConditional<ConditionalValueGreaterThan>(t_wp4_reached, 0.5, true, "WP4 Reached", "Aircraft reaches Waypoint 4 (10k, 70k, 15k')");

        CreateConditional<ConditionalValueGreaterThan>(t_leg1_alt_ok, 0.5, true, "Leg 1 Alt Hold (+/-20')", "Leg 1 altitude holds within 5,000 ft +/- 20 ft");
        CreateConditional<ConditionalValueGreaterThan>(t_leg2_alt_ok, 0.5, true, "Leg 2 Alt Hold (+/-20')", "Leg 2 captures and holds 12,000 ft +/- 20 ft");
        CreateConditional<ConditionalValueGreaterThan>(t_leg3_alt_ok, 0.5, true, "Leg 3 Alt Hold (+/-20')", "Leg 3 captures and holds 8,000 ft +/- 20 ft");
        CreateConditional<ConditionalValueGreaterThan>(t_leg4_alt_ok, 0.5, true, "Leg 4 Alt Hold (+/-20')", "Leg 4 captures and holds 15,000 ft +/- 20 ft");

        CreateConditional<ConditionalValueRemainsInRange>(t_nan, 0.0, 0.1, true, "Flight Stability", "No NaNs or state divergence occurred during flight");
    }

    void UpdateScenario(double /*dt*/) override {
        if (aircraftList_.empty()) return;
        auto ac = aircraftList_[0];

        const double alt = -ac->fm.state().kin.z;
        const size_t wpIdx = ac->brain.currentWaypoint();

        if (wpIdx == 0) {
            double err = std::abs(alt - 5000.0);
            if (err > maxErrLeg1_) maxErrLeg1_ = err;
            if (err > 20.0) leg1HoldPass_ = false;
        }

        if (wpIdx == 1) {
            if (!leg2Captured_) {
                if (std::abs(alt - 12000.0) <= 20.0) {
                    leg2Captured_ = true;
                    leg2HoldPass_ = true;
                    std::printf("\n  --> [Leg 2 Alt Captured] alt = %.1f ft\n", alt);
                }
            } else {
                double err = std::abs(alt - 12000.0);
                if (err > maxErrLeg2_) maxErrLeg2_ = err;
                if (err > 20.0) {
                    leg2HoldPass_ = false;
                    std::printf("\n  --> [Leg 2 Alt Exceeded] alt = %.1f ft (err = %.1f ft)\n", alt, err);
                }
            }
        }

        if (wpIdx == 2) {
            if (!leg3Captured_) {
                if (std::abs(alt - 8000.0) <= 20.0) {
                    leg3Captured_ = true;
                    leg3HoldPass_ = true;
                    std::printf("\n  --> [Leg 3 Alt Captured] alt = %.1f ft\n", alt);
                }
            } else {
                double err = std::abs(alt - 8000.0);
                if (err > maxErrLeg3_) maxErrLeg3_ = err;
                if (err > 20.0) {
                    leg3HoldPass_ = false;
                    std::printf("\n  --> [Leg 3 Alt Exceeded] alt = %.1f ft (err = %.1f ft)\n", alt, err);
                }
            }
        }

        if (wpIdx == 3 || ac->brain.allWaypointsCaptured()) {
            if (!leg4Captured_) {
                if (std::abs(alt - 15000.0) <= 20.0) {
                    leg4Captured_ = true;
                    leg4HoldPass_ = true;
                    std::printf("\n  --> [Leg 4 Alt Captured] alt = %.1f ft\n", alt);
                }
            } else {
                double err = std::abs(alt - 15000.0);
                if (err > maxErrLeg4_) maxErrLeg4_ = err;
                if (err > 20.0) {
                    leg4HoldPass_ = false;
                    std::printf("\n  --> [Leg 4 Alt Exceeded] alt = %.1f ft (err = %.1f ft)\n", alt, err);
                }
            }
        }
    }
};

static RegisterScenario g_registerFlightPlan("digi_flightplan", []() {
    return std::make_unique<DigiFlightPlanScenario>();
});

} // namespace f4flight_test
