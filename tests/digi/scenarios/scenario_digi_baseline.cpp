// f4flight - scenarios/scenario_digi_baseline.cpp
//
// Digi AI Baseline Integration Scenario: Waypoint Navigation and Cruise.
// Exercises the Behavior Tree / Flight Plan / Waypoint following logic:
//   Waypoint mode -> Waypoint Capture -> MachHold (throttle) -> HeadingAltitudeHold
//
// This is the single, clean, high-signal baseline scenario designed to verify
// that the core Digi AI is working properly.

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <string>
#include <vector>
#include <cmath>

using namespace f4flight;

namespace f4flight_test {

class BaselineNavigationPhase : public ManeuverTest {
protected:
    double nextPrint_{0.0};
    double targetAlt_{10000.0};
    double targetSpd_{350.0};
    bool hasNaN_{false};
    bool capturedAll_{false};
    SteeringController* sc_{nullptr};
    std::size_t waypointsVisited_{0};
    std::size_t lastWpIndex_{0};
    std::size_t totalWps_{0};

    // Tolerances for flight envelope during stable cruise.
    double ALT_TOL{500.0};  // ft
    double SPD_TOL{50.0};   // kts

    double minAlt_{std::numeric_limits<double>::max()};
    double maxAlt_{std::numeric_limits<double>::lowest()};

public:
    BaselineNavigationPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ = &sc;

        // Configure basic autopilot limits
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        sc.setTurnG(2.0);
        sc.setAltitude(targetAlt_);

        // Define a clean, high-signal square flight path (10 NM legs)
        // Waypoint 1: East
        // Waypoint 2: Northeast
        // Waypoint 3: North
        // Waypoint 4: Start
        double leg = 60000.0;
        std::vector<Vec3> wps = {
            Vec3{leg, 0.0, -targetAlt_},
            Vec3{leg, leg, -targetAlt_},
            Vec3{0.0, leg, -targetAlt_},
            Vec3{0.0, 0.0, -targetAlt_}
        };
        totalWps_ = wps.size();
        sc.setWaypoints(wps);
        sc.setCaptureRadius(5000.0);  // 5000 ft capture radius
        sc.setMode(SteeringController::Mode::Waypoint);

        waypointsVisited_ = 0;
        lastWpIndex_ = 0;
        minAlt_ = std::numeric_limits<double>::max();
        maxAlt_ = std::numeric_limits<double>::lowest();
    }

    bool IsFinished() const override {
        // Finish early if all waypoints are captured, or we run out of time.
        return phaseTime_ >= maxTime_ || capturedAll_;
    }

    bool IsPassed() const override {
        // Passed if no NaNs, we captured all waypoints successfully, and altitude held stable
        const bool altOk = std::fabs(maxAlt_ - targetAlt_) < ALT_TOL &&
                           std::fabs(minAlt_ - targetAlt_) < ALT_TOL;
        return !hasNaN_ && capturedAll_ && altOk;
    }

    void Finish() const override {
        std::printf("  --- Baseline Scenario Finish Summary ---\n");
        std::printf("  Waypoints captured: %zu / %zu  %s\n",
            waypointsVisited_, totalWps_,
            capturedAll_ ? "[PASS]" : "[FAIL]");
        std::printf("  Altitude band: %.0f..%.0f ft (target %.0f, dev +%.0f/-%.0f)  %s\n",
            minAlt_, maxAlt_, targetAlt_,
            std::fabs(maxAlt_ - targetAlt_), std::fabs(minAlt_ - targetAlt_),
            (std::fabs(maxAlt_ - targetAlt_) < ALT_TOL && std::fabs(minAlt_ - targetAlt_) < ALT_TOL)
                ? "[PASS]" : "[FAIL]");
    }

    std::string criteria() const override {
        return "Capture all 4 route waypoints within " + std::to_string(static_cast<int>(maxTime_)) +
               " seconds with altitude held stable within ±500 ft.";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft kinematic state (instability).";
        if (waypointsVisited_ < totalWps_) {
            return "Captured only " + std::to_string(waypointsVisited_) + " of " + std::to_string(totalWps_) + " waypoints.";
        }
        if (std::fabs(maxAlt_ - targetAlt_) >= ALT_TOL || std::fabs(minAlt_ - targetAlt_) >= ALT_TOL) {
            return "Altitude deviated too far: " + std::to_string(static_cast<int>(minAlt_)) + ".." + std::to_string(static_cast<int>(maxAlt_)) + " ft.";
        }
        return "";
    }

    std::vector<TestCondition> conditions() const override {
        return {
            {"Waypoint Capture", "Successfully captured all flight plan waypoints", capturedAll_},
            {"Flight Stability", "No NaNs or state divergence occurred during flight", !hasNaN_},
            {"Altitude Hold", "Altitude remained within ±500 ft of target",
                (std::fabs(maxAlt_ - targetAlt_) < ALT_TOL && std::fabs(minAlt_ - targetAlt_) < ALT_TOL)}
        };
    }

    std::vector<AdditionalResult> additionalResults() const override {
        return {
            {"Max Altitude: " + std::to_string(static_cast<int>(maxAlt_)) + " ft", "info"},
            {"Min Altitude: " + std::to_string(static_cast<int>(minAlt_)) + " ft", "info"}
        };
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z) ||
            std::isnan(as.kin.sigma) || std::isnan(input.pstick) ||
            std::isnan(input.throttle)) {
            hasNaN_ = true;
        }

        const std::size_t curWp = sc_ ? sc_->currentWaypoint() : 0;
        if (curWp > lastWpIndex_) {
            waypointsVisited_ += (curWp - lastWpIndex_);
            lastWpIndex_ = curWp;
        }

        if (sc_ && sc_->allWaypointsCaptured()) {
            capturedAll_ = true;
            // Ensure waypointsVisited is fully updated
            waypointsVisited_ = totalWps_;
        }

        double alt = -as.kin.z;
        double spd = as.vcas;

        minAlt_ = std::min(minAlt_, alt);
        maxAlt_ = std::max(maxAlt_, alt);

        // Print header and flight progress
        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %8s %8s %6s %6s %5s %5s\n",
                    "t(s)", "x(ft)", "y(ft)", "alt(ft)", "vcas(kt)", "pitch(d)", "roll(d)", "throt", "G", "curWp");
            }
            std::printf("%6.1f %8.0f %8.0f %8.0f %8.1f %8.1f %6.1f %6.2f %5.2f %5zu\n",
                phaseTime_, as.kin.x, as.kin.y, alt, spd, as.kin.theta * RTD, as.kin.phi * RTD, input.throttle, as.loads.nzcgs, curWp);
            nextPrint_ += 10.0;
        }
    }
};

class DigiBaselineScenario : public ManeuverScenario {
public:
    DigiBaselineScenario() : ManeuverScenario("digi_baseline") {}

    std::string GetDescription() const override {
        return "Digi AI Baseline: Cruise, Waypoint Navigation, and Altitude Hold. "
               "Flies a multi-waypoint triangular flight plan, captures waypoints, and validates navigation stability.";
    }

    std::string GetTestGroup() const override { return "Baseline"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 350.0;
        const double targetAlt = 10000.0;

        // Initialize Flight Model at target altitude and speed, heading East (0 rad)
        fm.init(ctx.cfg, targetAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Allocate 400s to complete the 4-waypoint circuit
        tests.push_back(std::make_unique<BaselineNavigationPhase>("Baseline 4-Waypoint Route", 400.0, targetAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerBaseline("digi_baseline", []() {
    return std::make_unique<DigiBaselineScenario>();
});

} // namespace f4flight_test
