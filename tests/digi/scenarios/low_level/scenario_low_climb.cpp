// f4flight - scenarios/scenario_low_climb.cpp
//
// LOW-LEVEL scenario: single staged climb from 5000ft to 15000ft.
//
// Split out of high_level/scenario_ai_basic.cpp (AILevelPhase chain). Wraps
// a single climb phase — start at 5000ft, command altitude 15000ft, verify
// the AI climbs and gets most of the way there.
//
// Pass criteria is RELAXED vs the parent scenario: parent requires ±150ft
// altitude band over a 30s settling window; we only require the aircraft
// to climb >= 8000ft (out of 10000ft target) at some point — proves the
// GammaHold + MachHold climbed the aircraft without stalling or leveling
// off early.
//
// Heavy aircraft (B-52H, C-130) cannot reach 15000ft in the test duration;
// for them we require climb >= 4000ft (any meaningful climb).
//
// Tier: LowLevel. Registered as "low_climb" — referenced by the cascade
// mapping table g_highToLow["high_departure"].

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;

namespace f4flight_test {

// ===========================================================================
// LowClimbPhase — climb from startAlt to targetAlt
// ===========================================================================
class LowClimbPhase : public ManeuverTest {
public:
    LowClimbPhase(const char* name, double duration, double startAlt,
                  double targetAlt, double speed)
        : ManeuverTest(name, duration), startAlt_(startAlt), targetAlt_(targetAlt),
          targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        const bool heavy = isHeavy(fm.config());
        isHeavy_ = heavy;
        sc.setMaxGamma(heavy ? 10.0 : 15.0);
        sc.setTurnG(heavy ? 1.3 : 2.0);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z) ||
            std::isnan(input.pstick) || std::isnan(input.throttle)) {
            hasNaN_ = true;
        }

        const double alt = -as.kin.z;
        const double spd = as.vcas;
        maxAlt_ = std::max(maxAlt_, alt);
        minAlt_ = std::min(minAlt_, alt);
        minSpd_ = std::min(minSpd_, spd);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (climb %.0f -> %.0fft)\n",
                    testName_.c_str(), startAlt_, targetAlt_);
                std::printf("%6s %8s %8s %8s %6s %6s %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "thrt", "pstk", "G");
            }
            std::printf("%6.0f %8.0f %8.0f %8.1f %6.2f %6.2f %5.2f\n",
                phaseTime_, alt, targetAlt_ - alt, spd,
                input.throttle, input.pstick, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // RELAXED: require the aircraft to have climbed by at least 8000ft
        // (heavy: 4000ft). This proves GammaHold + MachHold worked together
        // to climb without stalling. Don't require exact altitude capture
        // (the parent scenario handles that).
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        if (climbAmount < minClimb) return false;
        // Must not have stalled (speed stayed above 0.5 * target).
        if (minSpd_ < 0.5 * targetSpd_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Climb >= 8000ft (4000 heavy) from start; Min speed >= 50% target "
               "(no stall); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        if (climbAmount < minClimb)
            return "Only climbed " + std::to_string(static_cast<int>(climbAmount)) +
                   "ft (needed >= " + std::to_string(static_cast<int>(minClimb)) +
                   "ft) — GammaHold did not climb the aircraft.";
        if (minSpd_ < 0.5 * targetSpd_)
            return "Min speed was " + std::to_string(static_cast<int>(minSpd_)) +
                   "kts (needed >= " + std::to_string(static_cast<int>(0.5 * targetSpd_)) +
                   "kts) — climb stalled the aircraft.";
        return "";
    }

    void Finish() const override {
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Start altitude:    %.0f ft\n", startAlt_);
        std::printf("  Target altitude:   %.0f ft\n", targetAlt_);
        std::printf("  Max altitude:      %.0f ft (climbed %.0fft, need >= %.0f) %s\n",
            maxAlt_, climbAmount, minClimb,
            climbAmount >= minClimb ? "[PASS]" : "[FAIL]");
        std::printf("  Min speed:         %.1f kts (need >= %.0f) %s\n",
            minSpd_, 0.5 * targetSpd_,
            minSpd_ >= 0.5 * targetSpd_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double startAlt_{0.0};
    double targetAlt_{0.0};
    double targetSpd_{0.0};
    bool isHeavy_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double maxAlt_{-1e9};
    double minAlt_{1e9};
    double minSpd_{1e9};
};

// ===========================================================================
// LowClimbScenario
// ===========================================================================
class LowClimbScenario : public ManeuverScenario {
public:
    LowClimbScenario() : ManeuverScenario("low_climb") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: single staged climb from 5000ft to 15000ft. Verifies the "
               "AI's GammaHold + MachHold can climb the aircraft without "
               "stalling. Relaxed pass criteria (climb >= 8000ft fighter / "
               "4000ft heavy).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double startAlt = 5000.0;
        const double targetAlt = 15000.0;
        fm.init(ctx.cfg, startAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowClimbPhase>(
            "Climb 5000 -> 15000ft", 240.0, startAlt, targetAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerLowClimb("low_climb", []() {
    return std::make_unique<LowClimbScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_climb() {}

} // namespace f4flight_test
