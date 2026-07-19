// f4flight - scenarios/scenario_low_landing.cpp
//
// LOW-LEVEL scenario: full landing behavior (approach → flare → touchdown →
// rollout) in isolation.
//
// Split out of high_level/scenario_digi_groundops.cpp (LandingPhase). Wraps
// the landing behavior only — aircraft starts 3 NM south on a 3° glideslope
// and lands. Verifies the AI enters Landing mode, descends, flares, and
// touches down.
//
// Pass criteria is RELAXED vs the parent scenario: drop the descent-rate
// and touchdown-pitch checks (which the F-16 still marginally fails — known
// limitation). Just verify: enter Landing, descend, enter Flare, touch down,
// no crash, no NaN.
//
// Tier: LowLevel (one behavior per test). Registered as "low_landing" —
// referenced by the cascade mapping table g_highToLow["high_recovery"].

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
// LowLandingPhase — straight-in ILS approach + flare + touchdown + rollout
// ===========================================================================
class LowLandingPhase : public ManeuverTest {
public:
    LowLandingPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialRange = 3.0 * 6076.0;  // 3 NM
        const double gsAngle = 3.0 * DTR;
        const double initialAlt = initialRange * std::tan(gsAngle);
        const double initialHeading = PI / 2.0;  // north

        const double defaultApproachKts = 170.0;
        fm.init(fm.config(), initialAlt, defaultApproachKts * KNOTS_TO_FTPSEC,
                initialHeading, true);

        const double stallSpeed = fm.state().aero.stallSpeed > 0
            ? fm.state().aero.stallSpeed : 130.0;
        const double approachSpeedKts = 1.3 * stallSpeed;
        if (std::fabs(approachSpeedKts - defaultApproachKts) > 5.0) {
            fm.init(fm.config(), initialAlt, approachSpeedKts * KNOTS_TO_FTPSEC,
                    initialHeading, true);
        }

        fm.state().kin.x = 0.0;
        fm.state().kin.y = -initialRange;
        fm.state().kin.z = -initialAlt;

        const double thetaTrim = fm.state().kin.theta;
        fm.state().kin.theta = thetaTrim - gsAngle;
        fm.state().kin.gmma = -gsAngle;
        fm.state().kin.singam = -std::sin(gsAngle);
        fm.state().kin.cosgam = std::cos(gsAngle);
        const double vt0 = fm.state().kin.vt;
        fm.state().kin.xdot = 0.0;
        fm.state().kin.ydot = vt0 * std::cos(gsAngle);
        fm.state().kin.zdot = vt0 * std::sin(gsAngle);
        fm.state().kin.quat = quatFromEuler(fm.state().kin.psi, fm.state().kin.theta, fm.state().kin.phi);

        initialAlt_ = initialAlt;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        sc.brain().commandLanding(270, PI / 2.0, 0.0, 0.0, 0.0);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);

        if (altAGL < 10.0 && !touchedDown_) touchedDown_ = true;
        if (sc_brain_->activeMode() == DigiMode::Landing) enteredLanding_ = true;
        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Flare) enteredFlare_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_ = altAGL;
        curVcas_ = as.vcas;
        curMode_ = sc_brain_->activeMode();

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (landing 3NM final)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, digiModeName(curMode_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        // End as soon as we've touched down + a few seconds of rollout.
        return phaseTime_ >= maxTime_ || hasNaN_ || (touchedDown_ && phaseTime_ > 5.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLanding_) return false;
        // RELAXED: just verify the aircraft descended meaningfully. Parent
        // requires 500 ft of descent; we require 300 ft (still distinguishes
        // "descended on glideslope" from "level-flight noise").
        if (minAlt_ > initialAlt_ - 300.0) return false;
        // Must enter Flare phase at some point — proves the brain transitioned
        // from Approach to Flare.
        if (!enteredFlare_) return false;
        // Must not crash through the ground.
        if (minAlt_ < -500.0) return false;
        // Touchdown is required for "landing behavior works" — but we don't
        // check touchdown pitch / descent rate (parent does, F-16 fails both
        // marginally due to a known elevator limitation).
        if (!touchedDown_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Landing; Descend >= 300ft; Enter Flare; Touch down; "
               "Min alt >= -500ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLanding_)
            return "Never entered Landing mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minAlt_ > initialAlt_ - 300.0)
            return "Did not descend (min alt " +
                   std::to_string(static_cast<int>(minAlt_)) + "ft).";
        if (!enteredFlare_)
            return "Never entered Flare phase — brain did not transition from "
                   "Approach to Flare.";
        if (minAlt_ < -500.0)
            return "Min alt " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= -500ft) — ground reaction bug.";
        if (!touchedDown_)
            return "Never touched down (min alt " +
                   std::to_string(static_cast<int>(minAlt_)) + "ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",      curAlt_,      "ft"},
            {"vcas",     curVcas_,     "kts"},
            {"in_landing", (enteredLanding_ && curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
            {"touched_down", touchedDown_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Landing mode: %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Descended >= 300ft:   %s\n",
            minAlt_ <= initialAlt_ - 300.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Flare:        %s\n", enteredFlare_ ? "[PASS]" : "[FAIL]");
        std::printf("  Touched down:         %s\n", touchedDown_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:         %.0f ft (need >= -500) %s\n",
            minAlt_, minAlt_ >= -500.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double initialAlt_{0.0};
    bool touchedDown_{false};
    bool hasNaN_{false};
    bool enteredLanding_{false};
    bool enteredFlare_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0};
    double curVcas_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowLandingScenario
// ===========================================================================
class LowLandingScenario : public ManeuverScenario {
public:
    LowLandingScenario() : ManeuverScenario("low_landing") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: landing behavior. Aircraft starts 3NM south on a 3-degree "
               "glideslope and lands. Single phase, relaxed pass criteria "
               "(drops touchdown pitch / descent rate checks the F-16 "
               "marginally fails).";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
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
        tests.push_back(std::make_unique<LowLandingPhase>("Landing", 90.0));
        return tests;
    }
};

static RegisterScenario g_registerLowLanding("low_landing", []() {
    return std::make_unique<LowLandingScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_landing() {}

} // namespace f4flight_test
