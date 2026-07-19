// f4flight - scenarios/scenario_low_takeoff.cpp
//
// LOW-LEVEL scenario: takeoff behavior in isolation.
//
// Split out of high_level/scenario_digi_groundops.cpp (TakeoffPhase). Wraps
// the takeoff behavior only — aircraft starts on the runway at the threshold
// and takes off (accelerate, rotate, lift-off, climb out). Verifies the AI
// enters Takeoff mode, applies takeoff throttle, and becomes airborne.
//
// Pass criteria is RELAXED vs the parent scenario: just require airborne +
// minimum altitude (300 ft vs 500 ft parent) + minimum speed (150 kts vs
// 200 kts parent). Heavy aircraft get the same relaxation as the parent
// (only require 80 kts of acceleration).
//
// Tier: LowLevel (one behavior per test). Registered as "low_takeoff" —
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
// LowTakeoffPhase — accelerate, rotate, lift off, climb out
// ===========================================================================
class LowTakeoffPhase : public ManeuverTest {
public:
    LowTakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double rwyHeading = PI / 2.0;  // north
        fm.init(fm.config(), 0.0, 0.0, rwyHeading, false);  // on ground, 0 kts
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        isHeavy_ = isHeavy(fm.config());

        // Command takeoff. Runway heading matches the init heading.
        sc.brain().commandTakeoff(270, rwyHeading, 0.0, 0.0, 0.0);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (altAGL > 10.0) becameAirborne_ = true;
        if (input.throttle > 0.9) appliedTakeoffThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_ = altAGL;
        curVcas_ = as.vcas;
        curThrottle_ = input.throttle;
        curMode_ = sc_brain_->activeMode();

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (takeoff from runway 27)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, digiModeName(curMode_));
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || (maxAlt_ > 800.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTakeoff_) return false;
        if (!appliedTakeoffThrottle_) return false;
        // RELAXED criteria:
        //   Heavy aircraft: require >= 80 kts (parent requires same).
        //   Fighter/attack: require airborne + alt >= 300ft (vs 500 parent)
        //   and speed >= 150 kts (vs 200 parent). The point of the low-level
        //   test is "did the takeoff behavior engage and produce liftoff",
        //   not "did the aircraft complete a full climb-out".
        if (isHeavy_) return maxSpeed_ >= 80.0;
        if (!becameAirborne_) return false;
        if (maxAlt_ < 300.0) return false;
        if (maxSpeed_ < 150.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Takeoff mode; Apply takeoff throttle; "
               "Fighter: airborne + alt >= 300ft + speed >= 150kts; "
               "Heavy: speed >= 80kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredTakeoff_)
            return "Never entered Takeoff mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!appliedTakeoffThrottle_)
            return "Takeoff throttle never advanced (max " +
                   std::to_string(curThrottle_) + ").";
        if (isHeavy_) {
            if (maxSpeed_ < 80.0)
                return "Heavy max speed " + std::to_string(maxSpeed_) +
                       "kts (needed >= 80).";
            return "";
        }
        if (!becameAirborne_)
            return "Never became airborne (max alt " +
                   std::to_string(static_cast<int>(maxAlt_)) + "ft).";
        if (maxAlt_ < 300.0)
            return "Max altitude " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (needed >= 300ft).";
        if (maxSpeed_ < 150.0)
            return "Max speed " + std::to_string(maxSpeed_) +
                   "kts (needed >= 150kts).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",      curAlt_,      "ft"},
            {"vcas",     curVcas_,     "kts"},
            {"throttle", curThrottle_, ""},
            {"in_takeoff", (enteredTakeoff_ && curMode_ == DigiMode::Takeoff) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Takeoff mode:     %s\n", enteredTakeoff_ ? "[PASS]" : "[FAIL]");
        std::printf("  Applied takeoff throttle: %s\n", appliedTakeoffThrottle_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Heavy: max speed %.1f kts (need >= 80) %s\n",
                maxSpeed_, maxSpeed_ >= 80.0 ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Became airborne:          %s\n", becameAirborne_ ? "[PASS]" : "[FAIL]");
            std::printf("  Max altitude:             %.0f ft (need >= 300) %s\n",
                maxAlt_, maxAlt_ >= 300.0 ? "[PASS]" : "[FAIL]");
            std::printf("  Max speed:                %.1f kts (need >= 150) %s\n",
                maxSpeed_, maxSpeed_ >= 150.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    bool becameAirborne_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    bool enteredTakeoff_{false};
    bool appliedTakeoffThrottle_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0};
    double curVcas_{0.0};
    double curThrottle_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowTakeoffScenario
// ===========================================================================
class LowTakeoffScenario : public ManeuverScenario {
public:
    LowTakeoffScenario() : ManeuverScenario("low_takeoff") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: takeoff behavior. Aircraft starts on the runway threshold "
               "and takes off (accelerate, rotate, lift off, climb out). "
               "Single phase, relaxed pass criteria.";
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
        tests.push_back(std::make_unique<LowTakeoffPhase>("Takeoff", 90.0));
        return tests;
    }
};

static RegisterScenario g_registerLowTakeoff("low_takeoff", []() {
    return std::make_unique<LowTakeoffScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_takeoff() {}

} // namespace f4flight_test
