// f4flight - scenarios/scenario_low_approach.cpp
//
// LOW-LEVEL scenario: approach phase of landing only (no flare, no touchdown).
//
// Split out of high_level/scenario_digi_groundops.cpp (LandingPhase). Wraps
// the glideslope approach only — aircraft starts 3 NM south on a 3°
// glideslope and descends toward the runway. Verifies the AI enters Landing
// mode AND the Approach sub-phase, descends meaningfully, and stays within
// a wide altitude band (no go-around). The phase ends shortly before the
// flare would engage (so we don't accidentally test flare too).
//
// Pass criteria is RELAXED vs the parent scenario: just verify enter Landing
// mode, enter Approach sub-phase, descend >= 200 ft, no NaN.
//
// Tier: LowLevel (one behavior per test). Registered as "low_approach" —
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
// LowApproachPhase — descend on 3° glideslope toward the runway threshold
// ===========================================================================
class LowApproachPhase : public ManeuverTest {
public:
    LowApproachPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialRange = 3.0 * 6076.0;  // 3 NM
        const double gsAngle = 3.0 * DTR;
        const double initialAlt = initialRange * std::tan(gsAngle);
        const double initialHeading = PI / 2.0;

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

        if (sc_brain_->activeMode() == DigiMode::Landing) enteredLanding_ = true;
        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Approach)
            enteredApproach_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_ = altAGL;
        curVcas_ = as.vcas;
        curMode_ = sc_brain_->activeMode();
        curPhase_ = sc_brain_->state().ag.groundOps.phase;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (approach 3NM final)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "phase");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6d\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, static_cast<int>(curPhase_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        // End the phase as soon as we either leave Approach (entered Flare
        // or Touchdown) OR ran out of time. The whole point of this low-
        // level test is to verify the APPROACH behavior, not the full
        // landing — so we stop early when Flare begins.
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Flare ||
               sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Touchdown;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLanding_) return false;
        if (!enteredApproach_) return false;
        // RELAXED: just verify the aircraft descended at least 200 ft
        // (proves the glideslope tracker is working, not just cruising level).
        if (minAlt_ > initialAlt_ - 200.0) return false;
        // Must not have climbed excessively (no go-around).
        if (maxAlt_ > initialAlt_ + 400.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Landing; Enter Approach sub-phase; Descend >= 200ft; "
               "Max alt <= initial+400ft (no go-around); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLanding_)
            return "Never entered Landing mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!enteredApproach_)
            return "Never entered Approach sub-phase.";
        if (minAlt_ > initialAlt_ - 200.0)
            return "Did not descend (min alt " +
                   std::to_string(static_cast<int>(minAlt_)) + "ft, need <= " +
                   std::to_string(static_cast<int>(initialAlt_ - 200.0)) + "ft).";
        if (maxAlt_ > initialAlt_ + 400.0)
            return "Climbed too high (max alt " +
                   std::to_string(static_cast<int>(maxAlt_)) + "ft, need <= " +
                   std::to_string(static_cast<int>(initialAlt_ + 400.0)) +
                   "ft) — go-around.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",      curAlt_,      "ft"},
            {"vcas",     curVcas_,     "kts"},
            {"in_landing", (enteredLanding_ && curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
            {"in_approach", (enteredApproach_ &&
                             curPhase_ == GroundOpsPhase::Approach) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Landing mode:  %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Approach:      %s\n", enteredApproach_ ? "[PASS]" : "[FAIL]");
        std::printf("  Descended >= 200ft:    %s (min %.0fft, started %.0fft)\n",
            minAlt_ <= initialAlt_ - 200.0 ? "[PASS]" : "[FAIL]", minAlt_, initialAlt_);
        std::printf("  Max alt (no go-around):%s (max %.0fft, limit %.0fft)\n",
            maxAlt_ <= initialAlt_ + 400.0 ? "[PASS]" : "[FAIL]", maxAlt_, initialAlt_ + 400.0);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double initialAlt_{0.0};
    bool hasNaN_{false};
    bool enteredLanding_{false};
    bool enteredApproach_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0};
    double curVcas_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    GroundOpsPhase curPhase_{GroundOpsPhase::Parking};
};

// ===========================================================================
// LowApproachScenario
// ===========================================================================
class LowApproachScenario : public ManeuverScenario {
public:
    LowApproachScenario() : ManeuverScenario("low_approach") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: glideslope approach phase of landing only. Aircraft "
               "starts 3NM south on a 3-degree glideslope. Phase ends when "
               "Flare begins (does not test flare/touchdown).";
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
        tests.push_back(std::make_unique<LowApproachPhase>("Approach", 60.0));
        return tests;
    }
};

static RegisterScenario g_registerLowApproach("low_approach", []() {
    return std::make_unique<LowApproachScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_approach() {}

} // namespace f4flight_test
