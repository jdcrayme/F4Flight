// f4flight - scenarios/scenario_digi_groundops.cpp
//
// Maneuver tests for digi AI ground operations (Phase 2).
//
// These are END-TO-END integration tests that drive the full FlightModel +
// DigiBrain through takeoff and landing scenarios:
//
//   1. Set up a real aircraft on the ground at a runway threshold
//   2. Command the brain to start takeoff (or landing approach)
//   3. Run the simulation for ~60 seconds
//   4. Verify the AI executes the correct sequence:
//      - Takeoff: accelerate, rotate, lift off, climb out
//      - Landing: descend on glideslope, flare, touchdown, decelerate
//
// The tests use a flat-earth ground model (groundZ = 0) and a simple runway
// (heading 0°, threshold at origin). ATC clearance is auto-granted (the
// brain's startTakeoff/startLanding methods set hasTakeoffClearance/hasLandingClearance).

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace manuver_test {

// ===========================================================================
// Phase: Takeoff
//
// Aircraft starts on the ground at the runway threshold, brakes on. The AI
// should:
//   - Enter Takeoff mode
//   - Accelerate down the runway (throttle = 1.0)
//   - Rotate at V_R (~140 kts)
//   - Lift off (altitude > 10 ft AGL)
//   - Climb out to 1500 ft AGL
//   - Not NaN, not crash
// ===========================================================================
class TakeoffPhase : public ManeuverTest {
public:
    TakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start on the ground at the runway threshold
        fm.init(fm.config(), 0.0, 0.0, 0.0, false);  // on ground, 0 kts
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Check if this is a heavy aircraft (low T/W may not take off in time)
        isHeavy_ = isHeavy(fm.config());

        // Command takeoff via the new async-command API.
        // Runway 27 (heading 270° = west, but we use 0° = north for simplicity)
        // Runway threshold at origin, heading 0 (north)
        sc.brain().commandTakeoff(270, 0.0, 0.0, 0.0, 0.0);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;  // groundZ = 0
        maxAlt_ = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (altAGL > 10.0) becameAirborne_ = true;
        if (altAGL > 1000.0) reachedClimbout_ = true;
        if (input.throttle > 0.9) appliedTakeoffThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (takeoff from runway 27)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "rstk", "phase");
            }
            const std::size_t bufSize = 24;
            char phaseBuf[bufSize];
            switch (sc_brain_->state().groundOps.phase) {
                case GroundOpsPhase::TakeoffRoll: std::snprintf(phaseBuf, bufSize, "Roll"); break;
                case GroundOpsPhase::Rotation:    std::snprintf(phaseBuf, bufSize, "Rotation"); break;
                case GroundOpsPhase::AfterTakeoff: std::snprintf(phaseBuf, bufSize, "Climbout"); break;
                default: std::snprintf(phaseBuf, bufSize, "Other"); break;
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, input.rstick, phaseBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || reachedClimbout_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. All aircraft must enter Takeoff mode and apply takeoff throttle.
        if (!enteredTakeoff_) return false;
        if (!appliedTakeoffThrottle_) return false;
        // 2. Heavy aircraft (low T/W) may not have enough thrust to take off
        //    in the test time. For these, require at least 80 kts of
        //    acceleration (proves the throttle actually advanced and the
        //    aircraft is moving) — the old predicate accepted a stationary
        //    aircraft as long as the brain latched Takeoff mode.
        if (isHeavy_) {
            return maxSpeed_ >= 80.0;
        }
        // 3. Fighter/attack: must become airborne AND reach a meaningful
        //    altitude. The old test only required 100 ft (7% of the 1500 ft
        //    climbout target) — a hop-and-stall would pass. Require 500 ft
        //    (33% of climbout) and at least 200 kts (rotation + accel).
        if (!becameAirborne_) return false;
        if (maxAlt_ < 500.0) return false;
        if (maxSpeed_ < 200.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Takeoff mode:    %s\n", enteredTakeoff_ ? "[PASS]" : "[FAIL]");
        std::printf("  Applied takeoff throttle:%s\n", appliedTakeoffThrottle_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Heavy: max speed %.1f kts (need >= 80) %s\n",
                maxSpeed_, maxSpeed_ >= 80.0 ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Became airborne: %s\n", becameAirborne_ ? "[PASS]" : "[FAIL]");
            std::printf("  Max altitude:    %.0f ft (need >= 500) %s\n",
                maxAlt_, maxAlt_ >= 500.0 ? "[PASS]" : "[FAIL]");
            std::printf("  Max speed:       %.1f kts (need >= 200) %s\n",
                maxSpeed_, maxSpeed_ >= 200.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    bool becameAirborne_{false};
    bool reachedClimbout_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    bool enteredTakeoff_{false};
    bool appliedTakeoffThrottle_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: Landing
//
// Aircraft starts 3 NM from the runway threshold at 2000 ft AGL, heading
// toward the threshold. The AI should:
//   - Enter Landing mode
//   - Descend on ~3° glideslope
//   - Flare near the ground
//   - Touch down
//   - Decelerate
//   - Not crash, not NaN
// ===========================================================================
class LandingPhase : public ManeuverTest {
public:
    LandingPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 3 NM south of threshold, 2000 ft AGL, heading north toward threshold
        const double initialRange = 3.0 * 6076.0;  // 3 NM
        const double initialAlt = 2000.0;
        const double initialHeading = PI / 2.0;  // north (toward +Y, toward threshold)
        const double initialSpeedFtps = 250.0 * KNOTS_TO_FTPSEC;

        // Initialize with the correct heading so velocity vector is consistent
        // with body heading. (Previously init was called with heading=0 then
        // sigma was overwritten, leaving xdot/ydot pointing east while sigma
        // said north — the EOM then fought itself for several seconds.)
        fm.init(fm.config(), initialAlt, initialSpeedFtps, initialHeading, true);

        // Position aircraft 3 NM south of origin (threshold at origin)
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -initialRange;
        fm.state().kin.z = -initialAlt;
        // sigma and psi are already set by init; velocity components are
        // also set by init (xdot = vt*cos(sigma), ydot = vt*sin(sigma)).

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Command landing via the new async-command API.
        sc.brain().commandLanding(270, 0.0, 0.0, 0.0, 0.0);

        // The brain now clears pullupTimer/groundAvoidNeeded itself when
        // it detects a landing phase in compute() (matching FreeFalcon's
        // dlogic.cpp:49-52 which disables GroundCheck for LandingMode).

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;  // groundZ = 0
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);  // <-- was missing
        maxSpeed_ = std::max(maxSpeed_, as.vcas);
        minSpeed_ = std::min(minSpeed_, as.vcas);

        if (altAGL < 10.0 && !touchedDown_) {
            touchedDown_ = true;
            touchdownSpeed_ = as.vcas;  // capture speed at moment of touchdown
        }
        if (touchedDown_ && as.vcas < 30.0) stopped_ = true;
        if (sc_brain_->activeMode() == DigiMode::Landing) enteredLanding_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (landing on runway 27, 3NM final)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "rstk", "mode", "phase");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            char phaseBuf[bufSize];
            switch (sc_brain_->state().groundOps.phase) {
                case GroundOpsPhase::Approach:  std::snprintf(phaseBuf, bufSize, "Approach"); break;
                case GroundOpsPhase::Flare:     std::snprintf(phaseBuf, bufSize, "Flare"); break;
                case GroundOpsPhase::Touchdown: std::snprintf(phaseBuf, bufSize, "Touchdown"); break;
                case GroundOpsPhase::Rollout:   std::snprintf(phaseBuf, bufSize, "Rollout"); break;
                case GroundOpsPhase::VacatingRunway: std::snprintf(phaseBuf, bufSize, "Vacating"); break;
                default: std::snprintf(phaseBuf, bufSize, "Other"); break;
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f %6s %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, input.rstick, modeBuf, phaseBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || stopped_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered Landing mode.
        if (!enteredLanding_) return false;
        // 2. Must have descended (not just cruised). The old threshold of
        //    initialAlt-100 was too generous (a 100-ft descent over 90 s
        //    is level flight noise). Require descent of at least 500 ft.
        if (minAlt_ > initialAlt_ - 500.0) return false;
        // 3. Must not have climbed excessively. The TrackPointLanding
        //    primitive (ported from FreeFalcon mnvers.cpp:33) is a pure
        //    proportional tracker — no integral, no gamma feedback. In
        //    F4Flight's flight model this still produces a ~200-300 ft
        //    Phugoid transient at capture (the aircraft pitches up first,
        //    then descends — see DIGI_AUDIT.md "Known library gaps").
        //    The old test allowed +500 ft (a go-around would pass); we
        //    tighten to +400 ft (accepts the Phugoid, still catches a
        //    true go-around).
        if (maxAlt_ > initialAlt_ + 400.0) return false;
        // 4. Must touch down (altAGL <= 10 at some point).
        if (!touchedDown_) return false;
        // 5. Must not go excessively underground (no ground reaction bug).
        if (minAlt_ < -500.0) return false;
        // 6. Must have decelerated after touchdown. The current rollout
        //    logic sets throttle=0 but doesn't command wheel brakes
        //    (PilotInput has no brake field — see structural recommendation
        //    in DIGI_AUDIT.md). The aircraft decelerates only via drag +
        //    rolling resistance, so requiring a full stop is unrealistic.
        //    Instead require at least 30 kts of deceleration after touchdown
        //    (proves the rollout phase engaged and throttle went to idle).
        if (touchedDown_ && minSpeed_ > touchdownSpeed_ - 30.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Landing mode: %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.1f ft (need >= -500) %s\n",
            minAlt_, minAlt_ >= -500.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:        %.1f ft (need <= %.0f) %s\n",
            maxAlt_, initialAlt_ + 400.0,
            maxAlt_ <= initialAlt_ + 400.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Touched down:        %s %s\n", touchedDown_ ? "yes" : "no",
            touchedDown_ ? "[PASS]" : "[FAIL]");
        std::printf("  Descended >= 500 ft: %s\n",
            minAlt_ <= initialAlt_ - 500.0 ? "[PASS]" : "[FAIL]");
        if (touchedDown_) {
            std::printf("  Decel after TD:      %.0f -> %.0f kts (need >= 30 kts decel) %s\n",
                touchdownSpeed_, minSpeed_,
                minSpeed_ <= touchdownSpeed_ - 30.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    double minSpeed_{1e9};
    double touchdownSpeed_{0.0};
    double initialAlt_{2000.0};  // start altitude (ft AGL)
    bool touchedDown_{false};
    bool stopped_{false};
    bool hasNaN_{false};
    bool enteredLanding_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// DigiGroundOpsScenario
// ===========================================================================
class DigiGroundOpsScenario : public ManeuverScenario {
public:
    DigiGroundOpsScenario() : ManeuverScenario("digi_groundops") {}

    std::string GetDescription() const override {
        return "Digi AI ground ops: takeoff (accelerate, rotate, climb out) and "
               "landing (approach, flare, touchdown, rollout). End-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        fm.init(ctx.cfg, 0.0, 0.0, 0.0, false);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Takeoff: 90s for heavy aircraft (low T/W needs more runway)
        tests.push_back(std::make_unique<TakeoffPhase>("Takeoff", 90.0));
        // Landing: 90s for approach + flare + rollout
        tests.push_back(std::make_unique<LandingPhase>("Landing", 90.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiGroundOps("digi_groundops", []() {
    return std::make_unique<DigiGroundOpsScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_groundops() {}

} // namespace manuver_test
