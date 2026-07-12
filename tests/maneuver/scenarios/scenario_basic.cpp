// f4flight - scenarios/scenario_basic.cpp
//
// The "basic" scenario: level flight, climb, descent.
// Rewritten for the FreeFalcon digi AI steering port.
//
// The FreeFalcon digi AI uses GammaHold (flight path angle hold) for pitch
// and MachHold for throttle. There are no separate climb/descent speed
// schedules — the aircraft holds cornerSpeed throughout, and GammaHold
// adjusts the flight path angle to reach the target altitude.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <string>

using namespace f4flight;

namespace manuver_test {

// ===========================================================================
// TestPhase — simplified test phase for the digi AI steering
// ===========================================================================
class TestPhase : public ManeuverTest {
protected:
    double nextPrint_{0.0};
    const double ALT_TOL{100.0};
    const double SPD_TOL{10.0};

    double minAlt_{std::numeric_limits<double>::max()};
    double maxAlt_{std::numeric_limits<double>::lowest()};
    double minSpd_{std::numeric_limits<double>::max()};
    double maxSpd_{std::numeric_limits<double>::lowest()};

    double targetAlt_{0.0};
    double targetSpd_{0.0};
    double startAlt_{0.0};
    double altCaptureTime_{0.0};
    double speedCaptureTime_{0.0};
    bool isFirstFrame_{true};

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
    TestPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(0.0);
        // Use the scenario's target speed as the MachHold target, NOT the
        // aircraft's corner Vcas (which may be different from cruise speed).
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
    }

    virtual bool IsFinished() const {
        return phaseTime_ >= maxTime_ ||
            (altCaptureTime_ > 0 && speedCaptureTime_ > 0 &&
             phaseTime_ > altCaptureTime_ + 60.0 && phaseTime_ > speedCaptureTime_ + 60.0);
    }

    virtual bool IsPassed() const { return checkAltPass() && checkSpdPass(); }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        double alt = -as.kin.z;
        double spd = as.vcas;

        if (isFirstFrame_) {
            startAlt_ = alt;
            isFirstFrame_ = false;
        }

        // Altitude capture
        if (altCaptureTime_ == 0.0) {
            if (std::fabs(startAlt_ - targetAlt_) < ALT_TOL ||
                (startAlt_ < targetAlt_ && alt >= targetAlt_) ||
                (startAlt_ > targetAlt_ && alt <= targetAlt_)) {
                altCaptureTime_ = phaseTime_;
                minAlt_ = alt;
                maxAlt_ = alt;
            }
        } else {
            minAlt_ = std::min(minAlt_, alt);
            maxAlt_ = std::max(maxAlt_, alt);
        }

        // Speed capture (only after alt capture)
        if (speedCaptureTime_ == 0.0 && altCaptureTime_ > 0) {
            if (std::fabs(spd - targetSpd_) < SPD_TOL) {
                speedCaptureTime_ = phaseTime_;
                minSpd_ = spd;
                maxSpd_ = spd;
            }
        } else if (speedCaptureTime_ > 0.0) {
            minSpd_ = std::min(minSpd_, spd);
            maxSpd_ = std::max(maxSpd_, spd);
        }

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

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        if (altCaptureTime_ > 0.0) {
            std::printf("  Altitude capture at T+%.2f \t", altCaptureTime_);
            std::printf("  ALT: +%.0f ft, -%.0f ft %s\n",
                std::fabs(maxAlt_ - targetAlt_),
                std::fabs(minAlt_ - targetAlt_),
                checkAltPass() ? "[PASS]" : "[FAIL]");
            if (speedCaptureTime_ > 0.0) {
                std::printf("  Speed capture at T+%.2f \t", speedCaptureTime_);
                std::printf("  SPD: +%.1f kts, -%.1f kts %s\n",
                    std::fabs(maxSpd_ - targetSpd_),
                    std::fabs(minSpd_ - targetSpd_),
                    checkSpdPass() ? "[PASS]" : "[FAIL]");
            } else {
                std::printf("  Speed capture NOT achieved.  [FAIL]\n");
            }
        } else {
            std::printf("  Altitude capture NOT achieved.  [FAIL]\n");
        }
    }
};

// ===========================================================================
// BasicScenario — level flight, climb, level, descent
// ===========================================================================
class BasicScenario : public ManeuverScenario {
public:
    BasicScenario() : ManeuverScenario("basic") {}

    std::string GetDescription() const override {
        return "Level flight, climb, descent. Uses FreeFalcon digi AI steering.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cruiseAlt = 1500;
        const double climbAlt  = cruiseAlt + 10000.0;
        const double descentAlt = std::max(3000.0, cruiseAlt - 10000.0);
        const double cruiseSpeed = 380;
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, cruiseAlt, cruiseSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<TestPhase>("Level hold 1500ft", 120.0, cruiseAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestPhase>("Climb to 11500ft", 300.0, climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestPhase>("Level hold 11500ft", 120.0, climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestPhase>("Descend to 3000ft", 300.0, descentAlt, cruiseSpeed));
        return tests;
    }
};

static RegisterScenario g_registerBasic("basic", []() {
    return std::make_unique<BasicScenario>();
});

extern "C" void f4flight_forceLink_scenario_basic() {}

} // namespace manuver_test
