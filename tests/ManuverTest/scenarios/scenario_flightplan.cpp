// f4flight - scenarios/scenario_flightplan.cpp
//
// The "flightplan" scenario: fly a 4-waypoint square circuit using
// SteerToWaypoint. Demonstrates the AI's ability to navigate between
// waypoints, capture each in turn, and maintain altitude/speed throughout.
//
// This is the scenario that replaces the legacy `steering_demo` program.
// The demo program still exists as a forward-looking test harness for
// multi-behavior scenarios; this scenario is what gets run when you want
// the actual test results.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <string>

using namespace f4flight;

namespace manuver_test {

// ---------------------------------------------------------------------------
// TestWaypointLeg — fly toward the current waypoint, hold alt + speed.
// Passes if altitude and speed are maintained within tolerance for the
// duration of the leg. The phase ends when the waypoint is captured or
// maxTime expires.
// ---------------------------------------------------------------------------
class TestWaypointLeg : public ManeuverTest {
protected:
    double tgt_alt_ft;
    double tgt_speed_kts;

    double nextPrint_{ 0.0 };

    const double ALT_TOL{ 100.0 }; // ft
    const double SPD_TOL{ 10.0 };  // kts

    // Min/max tracking for altitude and speed
    double minAlt_{ std::numeric_limits<double>::max() };
    double maxAlt_{ std::numeric_limits<double>::lowest() };
    double minSpd_{ std::numeric_limits<double>::max() };
    double maxSpd_{ std::numeric_limits<double>::lowest() };

    double targetAlt_{ 0.0 };
    double targetSpd_{ 0.0 };

    bool checkAltPass() const {
        return std::fabs(maxAlt_ - targetAlt_) < ALT_TOL &&
            std::fabs(minAlt_ - targetAlt_) < ALT_TOL;
    }

    bool checkSpdPass() const {
        return std::fabs(maxSpd_ - targetSpd_) < SPD_TOL &&
            std::fabs(minSpd_ - targetSpd_) < SPD_TOL;
    }

    void printRow(const AircraftState& as, const PilotInput& input) const {
        double alt = -as.kin.z;
        double altErr = targetAlt_ - alt;
        double spdErr = targetSpd_ - as.vcas;
        std::printf("%6.0f %8.0f %8.0f %8.1f %8.1f %8.2f %8.2f %6.1f %6.1f %5.2f\n",
            phaseTime_, alt, altErr,
            as.vcas, spdErr,
            input.throttle, input.pstick,
            as.kin.phi * RTD, as.kin.theta * RTD,
            as.loads.nzcgs);
    }

public:
    TestWaypointLeg(double alt, double speed, Vec3 wp, double captureRadius_ft,
                    int wpIndex, int totalWps)
        : ManeuverTest("Waypoint leg", 600.0)
		, tgt_alt_ft(alt), tgt_speed_kts(speed)
        , wp_(wp), captureRadius_(captureRadius_ft)
        , wpIndex_(wpIndex), totalWps_(totalWps)
    {
        // Build a per-leg name like "Waypoint 1/4"
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Waypoint %d/%d", wpIndex_, totalWps_);
        testName_ = buf;
    }

    virtual bool IsPassed() const { return checkAltPass() && checkSpdPass(); }

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Single-waypoint SteerToWaypoint: the controller advances curWp_
        // when within captureRadius. We build a fresh SteerToWaypoint per
        // leg so the controller's curWp_ always points at OUR waypoint.
        std::vector<Vec3> wps = { wp_ };
        sc.setHorizontalBehavior(std::make_unique<SteerToWaypoint>(std::move(wps), captureRadius_));
        sc.setVerticalBehavior(std::make_unique<AltitudeHold>(tgt_alt_ft, tgt_speed_kts));
    }

    bool IsFinished() const override {
        return captured_ || phaseTime_ >= maxTime_;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        // Distance to waypoint
        const double dx = wp_.x - as.kin.x;
        const double dy = wp_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < captureRadius_) captured_ = true;


        double alt = -as.kin.z;
        minAlt_ = std::min(minAlt_, alt);
        maxAlt_ = std::max(maxAlt_, alt);
        minSpd_ = std::min(minSpd_, as.vcas);
        maxSpd_ = std::max(maxSpd_, as.vcas);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %8s %8s %8s %6s %6s %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "spdErr",
                    "throt", "pstick", "bank(d)", "pitch(d)", "G");
            }
            printRow(as, input);
            nextPrint_ += 10.0;
        }
    }

    void Finish() const {
        std::printf("  --- Summary ---\n");
        std::printf("  ALT: +%.0f ft, -%.0f ft %s\n",
            std::fabs(maxAlt_ - targetAlt_),
            std::fabs(minAlt_ - targetAlt_),
            checkAltPass() ? "[PASS]" : "[FAIL]");
        std::printf("  SPD: +%.1f kts, -%.1f kts %s\n",
            std::fabs(maxSpd_ - targetSpd_),
            std::fabs(minSpd_ - targetSpd_),
            checkSpdPass() ? "[PASS]" : "[FAIL]");
    }

private:
    Vec3   wp_;
    double captureRadius_;
    int    wpIndex_;
    int    totalWps_;
    bool   captured_{false};
};

// ===========================================================================
// FlightPlanScenario — fly a 4-waypoint square circuit
// ===========================================================================
class FlightPlanScenario : public ManeuverScenario {
public:
    FlightPlanScenario() : ManeuverScenario("flightplan") {}

    std::string GetDescription() const override {
        return "Fly a 4-waypoint square circuit using SteerToWaypoint. "
               "Each leg passes if altitude and speed are maintained within "
               "tolerance until the waypoint is captured.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000;
        const double speed = 350 * KNOTS_TO_FTPSEC;

        // Build a square circuit in the world (NED, Z-down). Side length
        // scales with cruise speed so the leg takes a reasonable time.
        // At 420 kts, ~120 NM sides => ~17 minutes per leg. We use a
        // shorter side (~30 NM) so the whole circuit fits in the 10-min
        // per-leg cap.
        const double side_ft = 30.0 * NM_TO_FT;  // 30 NM
        const double z = -alt;

        std::vector<Vec3> wps = {
            { side_ft,        0.0,       z },
            { side_ft,        side_ft,   z },
            { 0.0,            side_ft,   z },
            { 0.0,            0.0,       z },
        };
        const double captureRadius = 3000.0;  // ft

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        for (int i = 0; i < static_cast<int>(wps.size()); ++i) {
            tests.push_back(std::make_unique<TestWaypointLeg>(
                alt, speed, wps[i], captureRadius, i + 1, static_cast<int>(wps.size())));
        }
        return tests;
    }
};

static RegisterScenario g_registerFlightPlan("flightplan", []() {
    return std::make_unique<FlightPlanScenario>();
});

// Force-link symbol. See maneuver_test.h for the rationale.
extern "C" void f4flight_forceLink_scenario_flightplan() {}

} // namespace f4flight
