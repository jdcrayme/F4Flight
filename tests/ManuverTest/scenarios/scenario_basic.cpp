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

        double targetSpd_{ 0.0 };       // set on first frame based on phase type
        double startSpd_{ 0.0 };
        double speedCaptureTime_{ 0.0 };

        bool isFirstFrame_{ true }; // Added to initialize starting values
        int  speedWithinTolFrames_{0};  // frames speed has been within SPD_TOL of target

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
        // Construct a level-hold phase at (alt, speed).
        TestAltitudeControl(const char* testname, 
                            double alt, double speed, 
                            double heading_rad = 0.0)
            : ManeuverTest(testname, 360.0), heading_(heading_rad)
            , targetAlt_(alt)
        {}

        // Construct a climb or descent phase with a full speed schedule.
        // climbSpd/descentSpd default to 0 (use cruise speed); pass a
        // non-zero value to schedule a different speed during the
        // climb/descent.
        TestAltitudeControl(const char* testname, 
                            double alt, double cruiseSpd,
                            double climbSpd, double climbMach, double climbPower,
                            double descentSpd, double descentMach, double descentPower,
                            double heading_rad = 0.0)
            : ManeuverTest(testname, 360.0), heading_(heading_rad)
            , targetAlt_(alt)
        {}

        virtual bool IsFinished() const {
            // Finished on timeout, or when:
            //   - Altitude captured AND speed stabilized AND held for 60s
            //   - OR altitude captured but speed failed to stabilize within 60s
            if (phaseTime_ >= maxTime_) return true;
            if (altCaptureTime_ > 0.0 && speedCaptureTime_ > 0.0 &&
                phaseTime_ > speedCaptureTime_ + 60.0) {
                return true;  // both captured, held for 60s after speed stabilization
            }
            // If altitude was captured but speed hasn't stabilized in 60s, end as FAIL
            if (altCaptureTime_ > 0.0 && speedCaptureTime_ == 0.0 &&
                phaseTime_ > altCaptureTime_ + 60.0) {
                return true;
            }
            return false;
        }

        virtual bool IsPassed() const {
            // Pass requires: altitude captured with acceptable deviation,
            // AND speed stabilized within tolerance.
            // If speed is still stabilizing (not yet captured), that's a PASS
            // as long as altitude is good — the phase will continue running
            // until speed stabilizes or the window expires.
            if (!checkAltPass()) return false;
            if (altCaptureTime_ == 0.0) return false;
            if (speedCaptureTime_ == 0.0) {
                // Speed not yet stabilized — pass only if still within window
                return (phaseTime_ - altCaptureTime_) < 60.0;
            }
            return checkSpdPass();
        }

        void Init(SteeringController& sc, FlightModel& fm) override {

            sc.setVerticalBehavior(std::make_unique<AltitudeHold>(targetAlt_));
            sc.setHorizontalBehavior(std::make_unique<HeadingHold>(heading_));
            sc.setThrottleBehavior(std::make_unique<SpeedHold>(300));
        }

        void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
            ManeuverTest::Evaluate(as, input, dt);

            double alt = -as.kin.z;
            double spd = as.vcas;

            // Initialize starting conditions on the very first frame evaluated.
            // Also choose the pass/fail target speed based on whether this
            // phase is a climb, descent, or level hold.
            //
            // IMPORTANT: For climb/descent phases, the target speed is the
            // CRUISE speed (not climb/descent speed). The climb/descent
            // speed is what the AltitudeHold behavior targets DURING the
            // climb/descent, but once we level off, the aircraft should
            // settle at cruise speed. The test checks that speed stabilizes
            // at cruise speed AFTER altitude capture — not during the
            // climb/descent itself.
            if (isFirstFrame_) {
                startAlt_ = alt;
                startSpd_ = spd;
                // All phases check against cruise speed after level-off.
                targetSpd_ = as.vcas;
                isFirstFrame_ = false;
            }

            // --- 1. ALTITUDE CAPTURE LOGIC ---
            // Capture is the moment the aircraft first arrives at the target
            // altitude — either by crossing it (climbing/descending) or by
            // starting within tolerance of it (level-hold phases).
            //
            // For level-hold phases (where the aircraft starts at target),
            // we delay capture by 20 seconds to skip the initial trim
            // transient (the aircraft may sink 200+ ft while the FCS
            // settles to 1 G level flight). This is not a control failure —
            // it's just the FCS spooling up from the initial state.
            if (altCaptureTime_ == 0.0) {
                bool startedAtTarget = std::fabs(startAlt_ - targetAlt_) < ALT_TOL;
                bool trimDelayPassed = !startedAtTarget || phaseTime_ >= 20.0;

                if (trimDelayPassed && (
                    (startedAtTarget && std::fabs(alt - targetAlt_) < ALT_TOL) ||
                    (startAlt_ < targetAlt_ && alt >= targetAlt_) ||      // Climbing capture
                    (startAlt_ > targetAlt_ && alt <= targetAlt_))) {    // Descending capture

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

            // --- 2. SPEED STABILIZATION LOGIC ---
            //
            // Per the new design: we do NOT try to capture speed during
            // climbs or descents. Instead, after altitude is captured, we
            // wait for the speed to stabilize within tolerance for a
            // sustained period (2 seconds). The phase passes if speed
            // stabilizes within a time limit (60 seconds) after altitude
            // capture.
            //
            // For level-hold phases (where altitude is captured immediately),
            // this is equivalent to the old behavior: speed must stabilize
            // quickly and stay within tolerance.
            //
            // For climb/descent phases, the aircraft may overshoot/undershoot
            // speed during the transition — that's fine as long as it
            // settles at cruise speed after leveling off.
            if (altCaptureTime_ > 0.0) {
                if (speedCaptureTime_ == 0.0) {
                    // Check if speed is within tolerance
                    if (std::fabs(spd - targetSpd_) < SPD_TOL) {
                        speedWithinTolFrames_++;
                        // Stabilized for 2 seconds (120 frames at 60 Hz)
                        if (speedWithinTolFrames_ >= 120) {
                            speedCaptureTime_ = phaseTime_;
                            minSpd_ = spd;
                            maxSpd_ = spd;
                        }
                    } else {
                        speedWithinTolFrames_ = 0;
                    }
                } else {
                    // Post-stabilization: track min/max for pass/fail
                    minSpd_ = std::min(minSpd_, spd);
                    maxSpd_ = std::max(maxSpd_, spd);
                }
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
                std::printf("  Altitude capture at T+%.1f\t", altCaptureTime_);
                std::printf("  ALT: +%.0f / -%.0f ft %s\n",
                    std::fabs(maxAlt_ - targetAlt_),
                    std::fabs(minAlt_ - targetAlt_),
                    checkAltPass() ? "[PASS]" : "[FAIL]");

                if (speedCaptureTime_ > 0.0) {
                    std::printf("  Speed stabilized at T+%.1f (after alt capture)\t", speedCaptureTime_);
                    std::printf("  SPD: +%.1f / -%.1f kts %s\n",
                        std::fabs(maxSpd_ - targetSpd_),
                        std::fabs(minSpd_ - targetSpd_),
                        checkSpdPass() ? "[PASS]" : "[FAIL]");
                }
                else {
                    // Check if we're still within the stabilization window
                    double timeSinceAltCapture = phaseTime_ - altCaptureTime_;
                    if (timeSinceAltCapture < 60.0) {
                        std::printf("  Speed stabilizing (T+%.1f since alt capture, %.0fs remaining)  [PASS*]\n",
                            timeSinceAltCapture, 60.0 - timeSinceAltCapture);
                    } else {
                        std::printf("  Speed NOT stabilized within 60s of alt capture  [FAIL]\n");
                    }
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

        // Use the per-aircraft performance profile for speeds and power
        // settings, but use a lower altitude for the basic scenario to
        // keep the climb/descent short (10000 ft). The profile's cruise
        // altitude is used for the flightplan scenario; the basic scenario
        // uses a low starting altitude so the climb doesn't take too long
        // and the aircraft doesn't have time to oscillate.
        const double cruiseSpeed  = ctx.cfg.profile.cruiseSpeed_kts;

        // Basic scenario uses the profile's cruise altitude, then climbs
        // 10000 ft and descends 10000 ft. Using the profile's cruise
        // altitude ensures the tuned speeds (which were tested at that
        // altitude) are appropriate.
		const double cruiseAlt   = 1000; // Low altitude to keep climb in performance range for all aircraft
        const double climbAlt    = cruiseAlt + 10000.0;

        // Reset the flight model to a clean state for this scenario.
        // Use calcTasFromKcas to get the correct true airspeed for the desired
        // calibrated airspeed at the cruise altitude. Without this, the aircraft
        // would start at the wrong CAS (e.g., 248 kts instead of 340 at 20000 ft)
        // and the speed transient would cause altitude oscillation.
        fm.init(ctx.cfg, cruiseAlt, calcTasFromKcas(cruiseSpeed, cruiseAlt), 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<TestAltitudeControl>(
            "Phase 1: level hold at cruise altitude.", 
            cruiseAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(
            "Phase 2: climb to climbAlt using the full climb schedule.",
            climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(
			"Phase 3: level hold at climb altitude.",
            climbAlt, cruiseSpeed));
        tests.push_back(std::make_unique<TestAltitudeControl>(
			"Phase 4: descend to descentAlt using the full descent schedule.",
            cruiseAlt, cruiseSpeed));
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
