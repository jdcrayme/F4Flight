// f4flight - scenarios/scenario_basic.cpp
//
// The "basic" scenario: the original maneuver test sequence that has shipped
// with the library since v1.0. Exercises level flight, climb, descent,
// turn, orbit, accelerate, decelerate. This is the default scenario when
// `maneuver_test` is invoked without `--scenario`.
//
// The category profile (cruise speed, climb speed, etc.) is selected from
// the aircraft name in the same way the legacy maneuver_test did it.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <string>

using namespace f4flight;

namespace manuver_test {

    // ===========================================================================
    // Built-in test phases
    // ===========================================================================
    class TestAltitudeControl : public ManeuverTest {
    protected:
        double nextPrint_{ 0.0 };

        const double ALT_TOL{ 100.0 }; // ft
        const double SPD_TOL{ 10.0 };  // kts

        // Min/max tracking for altitude and speed
        double minAlt_{ std::numeric_limits<double>::max() };
        double maxAlt_{ std::numeric_limits<double>::lowest() };
        double minSpd_{ std::numeric_limits<double>::max() };
        double maxSpd_{ std::numeric_limits<double>::lowest() };

        double targetAlt_{ 0.0 };
        double startAlt_{ 0.0 };
        double altCaptureTime_{ 0.0 };

        double targetSpd_{ 0.0 };
        double startSpd_{ 0.0 };
        double speedCaptureTime_{ 0.0 };

        bool isFirstFrame_{ true }; // Added to initialize starting values

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
        TestAltitudeControl(double alt, double speed, double heading_rad = 0.0)
            : ManeuverTest("Cruise stable", 360.0), heading_(heading_rad), targetAlt_(alt), targetSpd_(speed) {
        }

        virtual bool IsFinished() const {
            // Finished on timeout, or when both captured and held for 60 seconds
            return phaseTime_ >= maxTime_ ||
                (altCaptureTime_ > 0 && speedCaptureTime_ > 0 &&
                    phaseTime_ > altCaptureTime_ + 60.0 && phaseTime_ > speedCaptureTime_ + 60.0);
        }

        virtual bool IsPassed() const { return checkAltPass() && checkSpdPass(); }

        void Init(SteeringController& sc, FlightModel& fm) override {
            sc.setVerticalBehavior(std::make_unique<AltitudeHold>(targetAlt_, targetSpd_));
            sc.setHorizontalBehavior(std::make_unique<HeadingHold>(heading_));
            sc.setThrottleBehavior(std::make_unique<SpeedHold>(targetSpd_));
        }

        void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
            ManeuverTest::Evaluate(as, input, dt);

            double alt = -as.kin.z;
            double spd = as.vcas;

            // Initialize starting conditions on the very first frame evaluated
            if (isFirstFrame_) {
                startAlt_ = alt;
                startSpd_ = spd;
                isFirstFrame_ = false;
            }

            // --- 1. ALTITUDE CAPTURE LOGIC ---
            if (altCaptureTime_ == 0.0) {
                if ((startAlt_ < targetAlt_ && alt >= targetAlt_) ||      // Climbing capture
                    (startAlt_ > targetAlt_ && alt <= targetAlt_) ||      // Descending capture
                    (startAlt_ == targetAlt_)) {                         // Already at target

                    altCaptureTime_ = phaseTime_;
                    minAlt_ = alt;
                    maxAlt_ = alt;
                }
            }
            else {
                // Only track min/max post-capture to accurately measure overshoot/undershoot
                minAlt_ = std::min(minAlt_, alt);
                maxAlt_ = std::max(maxAlt_, alt);
            }

            // --- 2. SPEED CAPTURE LOGIC ---
            if (speedCaptureTime_ == 0.0 && altCaptureTime_>0) {
                // Note: Ensuring we check capture condition only, without early else-pollution
                if ((startSpd_ < targetSpd_ && spd >= targetSpd_) ||      // Accelerating capture
                    (startSpd_ > targetSpd_ && spd <= targetSpd_) ||      // Decelerating capture
                    (startSpd_ == targetSpd_)) {                         // Already at target speed

                    speedCaptureTime_ = phaseTime_;
                    minSpd_ = spd;
                    maxSpd_ = spd;
                }
            }
            else {
                // Only track min/max post-capture
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

        void Finish() const {
            std::printf("  --- Summary ---\n");

            if (altCaptureTime_ > 0.0) {
                std::printf("  Altitude capture achieved at T+%.2f \t", altCaptureTime_);
                std::printf("  ALT: +%.0f ft, -%.0f ft %s\n",
                    std::fabs(maxAlt_ - targetAlt_),
                    std::fabs(minAlt_ - targetAlt_),
                    checkAltPass() ? "[PASS]" : "[FAIL]");

                if (speedCaptureTime_ > 0.0) {
                    std::printf("  Speed capture achieved at T+%.2f \t", speedCaptureTime_);
                    std::printf("  SPD: +%.1f kts, -%.1f kts %s\n",
                        std::fabs(maxSpd_ - targetSpd_),
                        std::fabs(minSpd_ - targetSpd_),
                        checkSpdPass() ? "[PASS]" : "[FAIL]");
                }
                else {
                    std::printf("  Speed capture NOT achieved.  [FAIL]\n");
                }
            }
            else {
                std::printf("  Altitude capture NOT achieved.  [FAIL]\n");
            }
        }

    private:
        double heading_;
    };

// ===========================================================================
// BasicScenario — the original maneuver test sequence
// ===========================================================================
class BasicScenario : public ManeuverScenario {
public:
    BasicScenario() : ManeuverScenario("basic") {}

    std::string GetDescription() const override {
        return "Level flight, climb, cruse, descent. "
               "The default maneuver test sequence.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cruiseAlt = 1500;
        const double climbAlt  = cruiseAlt + 10000.0;
        const double descentAlt = std::max(3000.0, cruiseAlt - 10000.0);
        const double cruiseSpeed = 380 ;
        const double climbSpeed = 320;
        const double climbMach = 0.8;
        const double descendSpeed = 420;
		const double descendMach = 0.8;
		const double climbPower = 0.99;
		const double descendPower = 0.01;

        // Reset the flight model to a clean state for this scenario. Each test phase
        fm.init(ctx.cfg, cruiseAlt, cruiseSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<TestAltitudeControl>(cruiseAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(descentAlt, cruiseSpeed));
        return tests;
    }
};

// Self-register the scenario. The static initializer runs before main()
// starts, so `maneuver_test --scenario basic` (and the default `--all`)
// will find it without any explicit wiring in main().
static RegisterScenario g_registerBasic("basic", []() {
    return std::make_unique<BasicScenario>();
});

// Force-link symbol. See maneuver_test.h for the rationale.
extern "C" void f4flight_forceLink_scenario_basic() {}

} // namespace f4flight
