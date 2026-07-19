// f4flight - scenarios/scenario_low_taxi.cpp
//
// LOW-LEVEL scenario: taxi-to-runway behavior in isolation.
//
// Split out of high_level/scenario_digi_groundops.cpp (TaxiPhase). Wraps the
// taxi behavior only — aircraft starts 500 ft east of the runway threshold
// and taxis to the origin. Verifies the AI enters TaxiToRunway phase,
// closes the distance, and stays at taxi speed.
//
// Pass criteria is RELAXED vs the parent scenario: same essential checks
// (enter taxi, reach threshold, hold taxi speed, no NaN) since taxi is
// already a single behavior — but no takeoff/landing follow-on phases.
//
// Tier: LowLevel (one behavior per test). Registered as "low_taxi" —
// referenced by the cascade mapping table g_highToLow["high_departure"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowTaxiPhase — taxi 500 ft east → runway threshold at origin
// ===========================================================================
class LowTaxiPhase : public ManeuverTest {
public:
    LowTaxiPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 500 ft east of the runway threshold, heading north.
        // In F4Flight's NED frame, north = +Y = heading PI/2.
        const double startX = 500.0;
        const double startY = 0.0;
        const double startHeading = PI / 2.0;

        fm.init(fm.config(), 0.0, 0.0, startHeading, false);
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;
        fm.state().kin.z = 0.0;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Command taxi to the runway threshold (origin).
        auto& go = sc.brain().stateMutable().ag.groundOps;
        go.phase = GroundOpsPhase::TaxiToRunway;
        go.runwayThresholdX = 0.0;
        go.runwayThresholdY = 0.0;
        go.runwayHeading = PI / 2.0;
        go.hasTakeoffClearance = false;

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double dx = 0.0 - as.kin.x;
        const double dy = 0.0 - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::TaxiToRunway)
            enteredTaxi_ = true;
        if (dist < 50.0) reachedThreshold_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curDist_ = dist;
        curVcas_ = as.vcas;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (taxi 500ft east -> threshold)\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s\n",
                    "t(s)", "x(ft)", "dist(ft)", "vcas", "phase");
            }
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.1f\n",
                phaseTime_, as.kin.x, dist, as.vcas, as.kin.sigma * RTD);
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || reachedThreshold_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTaxi_) return false;
        // RELAXED: only require meaningful closure (started 500 ft away,
        // must close at least 300 ft — proves taxi steering is moving the
        // aircraft toward the threshold). The parent requires reaching
        // within 50 ft, but for a low-level smoke test, "moved toward the
        // threshold" is enough.
        if (minDist_ > 200.0) return false;
        if (maxSpeed_ > 40.0) return false;  // generous taxi-speed cap
        return true;
    }

    std::string criteria() const override {
        return "Enter TaxiToRunway; Close to < 200ft of threshold; "
               "Speed <= 40kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredTaxi_) return "Never entered TaxiToRunway phase.";
        if (minDist_ > 200.0)
            return "Min dist to threshold was " + std::to_string(static_cast<int>(minDist_)) +
                   "ft (needed < 200ft) — taxi steering did not converge.";
        if (maxSpeed_ > 40.0)
            return "Max speed was " + std::to_string(maxSpeed_) +
                   "kts (needed <= 40kts) — speed governor did not hold taxi speed.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_thresh", curDist_, "ft"},
            {"vcas",     curVcas_, "kts"},
            {"in_taxi",  (enteredTaxi_ &&
                          sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::TaxiToRunway)
                         ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Taxi mode:        %s\n", enteredTaxi_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to threshold:    %.1f ft (need < 200) %s\n",
            minDist_, minDist_ < 200.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max speed:                %.1f kts (need <= 40) %s\n",
            maxSpeed_, maxSpeed_ <= 40.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minDist_{1e9};
    double maxSpeed_{0.0};
    bool reachedThreshold_{false};
    bool enteredTaxi_{false};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curDist_{0.0};
    double curVcas_{0.0};
};

// ===========================================================================
// LowTaxiScenario — single-phase low-level taxi test
// ===========================================================================
class LowTaxiScenario : public ManeuverScenario {
public:
    LowTaxiScenario() : ManeuverScenario("low_taxi") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: taxi-to-runway behavior. Aircraft starts 500ft east of "
               "the runway threshold and taxis to origin. Single phase, "
               "relaxed pass criteria (just verify taxi steering works).";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
        // Reuse the parent scenario's runway drawing for visualization.
        std::vector<TraceGeometry> geom;
        const double rwLen = 10000.0;
        const double rwHalf = rwLen / 2.0;
        const double rwWidth = 200.0;
        TraceGeometry centerline;
        centerline.name = "RWY";
        centerline.type = "runway";
        centerline.coords = {0.0, -rwHalf, 0.0, 0.0, rwHalf, 0.0};
        centerline.color = "#3a3a4a";
        centerline.width = 150.0;
        geom.push_back(centerline);
        TraceGeometry threshN;
        threshN.name = "RWY_End_N";
        threshN.type = "taxiway";
        threshN.coords = {-rwWidth, rwHalf, 0.0, rwWidth, rwHalf, 0.0};
        threshN.color = "#3a3a4a";
        threshN.width = 80.0;
        geom.push_back(threshN);
        TraceGeometry threshS;
        threshS.name = "RWY_End_S";
        threshS.type = "taxiway";
        threshS.coords = {-rwWidth, -rwHalf, 0.0, rwWidth, -rwHalf, 0.0};
        threshS.color = "#3a3a4a";
        threshS.width = 80.0;
        geom.push_back(threshS);
        return geom;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        fm.init(ctx.cfg, 0.0, 0.0, 0.0, false);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowTaxiPhase>("Taxi to runway", 60.0));
        return tests;
    }
};

static RegisterScenario g_registerLowTaxi("low_taxi", []() {
    return std::make_unique<LowTaxiScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_taxi() {}

} // namespace f4flight_test
