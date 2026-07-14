// f4flight - scenarios/scenario_ai_flightplan.cpp
//
// AI steering test: 4-waypoint square circuit using the digi AI waypoint
// following. Uses sc.compute() — exercises:
//   runWaypoint → HeadingAndAltitudeHold → AltitudeHold/LevelTurn → GammaHold
//   MachHold (throttle)
//   waypoint capture logic
//
// Per-aircraft adaptation:
//   - Target speed = cfg.geometry.cornerVcas_kts
//   - Altitude = 10000 ft (mid-altitude, not 1500)
//   - maxGamma = 15° (navigation envelope)

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>

using namespace f4flight;

namespace f4flight_test {

class AIFlightplanTest : public ManeuverTest {
public:
    AIFlightplanTest(const char* name, double duration,
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
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        // Heavy aircraft need a lower gamma envelope to avoid stalling during
        // turns (see scenario_ai_basic for the same rationale). The waypoint
        // circuit has 90° turns at each corner; heavy aircraft at 15° gamma
        // bleed speed and overshoot altitude. 10° keeps them sustainable.
        // Heavy aircraft also need a shallower turn load factor (1.3G vs 2.0G)
        // so the turn doesn't demand more G than they can sustain. A B-52H
        // (maxGs=2.3) in a 2.0G turn leaves only 0.3G for altitude recovery;
        // at 1.3G (35° bank) it has 1.0G of margin.
        const bool heavy = isHeavy(fm.config());
        sc.setMaxGamma(heavy ? 10.0 : 15.0);
        sc.setMaxBank(heavy ? 25.0 : 45.0);
        sc.setTurnG(heavy ? 1.3 : 2.0);
        inputTracker_ = &sc;
        waypointsVisited_ = 0;
        lastWpIndex_ = 0;
        minAlt_ = std::numeric_limits<double>::max();
        maxAlt_ = std::numeric_limits<double>::lowest();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const std::size_t curWp = inputTracker_ ? inputTracker_->currentWaypoint() : 0;
        if (curWp > lastWpIndex_) {
            waypointsVisited_ += (curWp - lastWpIndex_);
            lastWpIndex_ = curWp;
        }

        const double alt = -as.kin.z;
        minAlt_ = std::min(minAlt_, alt);
        maxAlt_ = std::max(maxAlt_, alt);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("  %6s  %8s  %8s  %8s  %6s  %6s  %5s  %5s\n",
                    "t(s)", "x(ft)", "y(ft)", "alt(ft)", "bank(d)", "pitch(d)", "G", "wp");
            }
            std::printf("  %6.0f  %8.0f  %8.0f  %8.1f  %6.1f  %6.1f  %5.2f  %4zu/%zu\n",
                phaseTime_, as.kin.x, as.kin.y, alt,
                as.kin.phi * RTD, as.kin.theta * RTD, as.loads.nzcgs,
                lastWpIndex_, wps_.size());
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ ||
               waypointsVisited_ >= wps_.size();
    }

    bool IsPassed() const override {
        // 1. Must visit all waypoints (was size-1, which accepted visiting
        //    only 3 of 4 corners — the 4th is "back to start").
        const bool wpOk = waypointsVisited_ >= wps_.size();
        // 2. Altitude band must stay within 500 ft of target.
        const bool altOk = std::fabs(maxAlt_ - alt_) < 500.0 &&
                           std::fabs(minAlt_ - alt_) < 500.0;
        return wpOk && altOk;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Waypoints captured: %zu / %zu  %s\n",
            waypointsVisited_, wps_.size(),
            waypointsVisited_ >= wps_.size() ? "[PASS]" : "[FAIL]");
        std::printf("  Altitude band: %.0f..%.0f ft (target %.0f, dev +%.0f/-%.0f)  %s\n",
            minAlt_, maxAlt_, alt_,
            std::fabs(maxAlt_ - alt_), std::fabs(minAlt_ - alt_),
            (std::fabs(maxAlt_ - alt_) < 500.0 && std::fabs(minAlt_ - alt_) < 500.0)
                ? "[PASS]" : "[FAIL]");
    }

private:
    std::vector<Vec3> wps_;
    double captureRadius_;
    double alt_;
    double speed_;
    double nextPrint_{0.0};
    std::size_t waypointsVisited_{0};
    std::size_t lastWpIndex_{0};
    double minAlt_{std::numeric_limits<double>::max()};
    double maxAlt_{std::numeric_limits<double>::lowest()};
    const SteeringController* inputTracker_{nullptr};
};

class AIFlightplanScenario : public ManeuverScenario {
public:
    AIFlightplanScenario() : ManeuverScenario("ai_flightplan") {}

    std::string GetDescription() const override {
        return "AI 4-waypoint square circuit. Per-aircraft corner speed. "
               "Exercises waypoint following + HeadingAndAltitudeHold.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double alt = 10000.0;
        const double leg = 60000.0;  // 10 NM legs

        fm.init(ctx.cfg, alt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<Vec3> wps;
        wps.push_back({0.0,   leg,  -alt});  // North
        wps.push_back({leg,   leg,  -alt});  // East
        wps.push_back({leg,   0.0,  -alt});  // South
        wps.push_back({0.0,   0.0,  -alt});  // West (back to start)

        auto test = std::make_unique<AIFlightplanTest>(
            "4-waypoint circuit", 600.0, std::move(wps), 5000.0, alt, cornerSpeed);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::move(test));
        return tests;
    }
};

static RegisterScenario g_registerAIFlightplan("ai_flightplan", []() {
    return std::make_unique<AIFlightplanScenario>();
});

extern "C" void f4flight_forceLink_scenario_ai_flightplan() {}

} // namespace f4flight_test
