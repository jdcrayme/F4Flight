// f4flight - scenarios/scenario_low_level_hold.cpp
//
// LOW-LEVEL scenario: single level-altitude hold phase.
//
// Split out of high_level/scenario_ai_basic.cpp (AILevelPhase). Wraps a
// single level-hold phase — start at 10000ft, command altitude 10000ft,
// verify the AI holds altitude and speed within reasonable tolerances.
//
// Pass criteria is RELAXED vs the parent scenario: parent requires ±150ft
// altitude band over a 30s settling window AND altitude+speed capture;
// we only require the aircraft to stay within ±500ft of target altitude
// for the entire phase (proves GammaHold is working, not necessarily
// perfectly tuned).
//
// Tier: LowLevel. Registered as "low_level_hold" — referenced by the
// cascade mapping tables g_highToLow["high_departure"] and
// g_highToLow["high_loiter_station"].

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;

namespace f4flight_test {

// ===========================================================================
// LowLevelHoldPhase — hold altitude + heading at cruise
// ===========================================================================
class LowLevelHoldPhase : public ManeuverTest {
public:
    LowLevelHoldPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

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
        // Track the WORST deviation over the whole phase.
        maxAltErr_ = std::max(maxAltErr_, std::fabs(alt - targetAlt_));
        maxSpdErr_ = std::max(maxSpdErr_, std::fabs(spd - targetSpd_));

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (level hold at %.0fft)\n", testName_.c_str(), targetAlt_);
                std::printf("%6s %8s %8s %8s %8s %6s %6s %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "spdErr", "thrt", "pstk", "G");
            }
            std::printf("%6.0f %8.0f %8.0f %8.1f %8.1f %6.2f %6.2f %5.2f\n",
                phaseTime_, alt, targetAlt_ - alt, spd, targetSpd_ - spd,
                input.throttle, input.pstick, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // RELAXED: parent requires ±150ft over a 30s window after capture;
        // we just require the WORST deviation over the entire phase to be
        // <= 500ft (heavy: 800ft). This proves GammaHold held the aircraft
        // near the target altitude, even if there was a capture transient.
        const double altTol = isHeavy_ ? 800.0 : 500.0;
        if (maxAltErr_ > altTol) return false;
        // Speed tolerance: parent uses 25kts (50 heavy). We use 60kts (100
        // heavy) — just verify MachHold is roughly tracking.
        const double spdTol = isHeavy_ ? 100.0 : 60.0;
        if (maxSpdErr_ > spdTol) return false;
        return true;
    }

    std::string criteria() const override {
        return "Altitude stays within ±500ft (±800 heavy) of target for whole phase; "
               "Speed within ±60kts (±100 heavy); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        const double altTol = isHeavy_ ? 800.0 : 500.0;
        if (maxAltErr_ > altTol)
            return "Max altitude error was " + std::to_string(static_cast<int>(maxAltErr_)) +
                   "ft (needed <= " + std::to_string(static_cast<int>(altTol)) +
                   "ft) — GammaHold did not hold altitude.";
        const double spdTol = isHeavy_ ? 100.0 : 60.0;
        if (maxSpdErr_ > spdTol)
            return "Max speed error was " + std::to_string(static_cast<int>(maxSpdErr_)) +
                   "kts (needed <= " + std::to_string(static_cast<int>(spdTol)) +
                   "kts) — MachHold did not hold speed.";
        return "";
    }

    void Finish() const override {
        const double altTol = isHeavy_ ? 800.0 : 500.0;
        const double spdTol = isHeavy_ ? 100.0 : 60.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Target altitude:  %.0f ft\n", targetAlt_);
        std::printf("  Max alt error:    %.0f ft (need <= %.0f) %s\n",
            maxAltErr_, altTol, maxAltErr_ <= altTol ? "[PASS]" : "[FAIL]");
        std::printf("  Target speed:     %.0f kts\n", targetSpd_);
        std::printf("  Max spd error:    %.0f kts (need <= %.0f) %s\n",
            maxSpdErr_, spdTol, maxSpdErr_ <= spdTol ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double targetAlt_{0.0};
    double targetSpd_{0.0};
    bool isHeavy_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double maxAltErr_{0.0};
    double maxSpdErr_{0.0};
};

// ===========================================================================
// LowLevelHoldScenario
// ===========================================================================
class LowLevelHoldScenario : public ManeuverScenario {
public:
    LowLevelHoldScenario() : ManeuverScenario("low_level_hold") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: single level-altitude hold at 10000ft. Verifies GammaHold "
               "+ MachHold can maintain altitude and speed. Relaxed pass "
               "criteria (±500ft altitude / ±60kts speed over the whole phase).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double alt = 10000.0;
        fm.init(ctx.cfg, alt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowLevelHoldPhase>(
            "Level hold 10000ft", 120.0, alt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerLowLevelHold("low_level_hold", []() {
    return std::make_unique<LowLevelHoldScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_level_hold() {}

} // namespace f4flight_test
