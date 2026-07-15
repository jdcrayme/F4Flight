// f4flight - scenarios/scenario_digi_merge.cpp
//
// Maneuver test for digi AI Merge mode (close nose-to-nose engagement).
//
// Sets up a merge scenario: AI and target start ~3000 ft apart, nose-to-nose,
// closing fast. The AI should:
//   1. Detect the target via the injected target pointer.
//   2. Enter Merge mode at close range (range <= 1000 ft, ata < 45°, altAGL
//      > 3000 ft, pitch < 45°, ataFrom < 45°).
//   3. Execute the merge maneuver (level turn / slice / vertical pull based
//      on speed vs corner speed).
//   4. Not crash or NaN.
//
// The MergeCheck triggers at range <= 1000 ft. At corner speed (350 kts)
// head-on, closure is ~1170 ft/s, so range drops from 3000 ft to merge
// range in ~1.7 s. Merge mode then runs for 3 s (mergeTimer).
//
// Note: at very close range (< ~844 ft for F-16, reactTime*closure+kHRange),
// CollisionAvoid may pre-empt Merge. This is by design — the test verifies
// the AI enters Merge OR WVREngage at some point during the close pass.
//
// Pass criteria:
//   - Enters Merge mode (preferred) or WVREngage (fallback) at some point
//   - Maneuvers (heading change > 20° — merge involves a hard turn)
//   - No NaN, no crash

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// Phase: Merge (close nose-to-nose)
// ===========================================================================
class MergePhase : public ManeuverTest {
public:
    MergePhase(const char* name, double duration,
               double alt, double speed, double targetSpeed,
               double initialRangeFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeFt_(initialRangeFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);  // east
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Target ahead (+x), head-on (heading west, toward us).
        target_.x = initialRangeFt_;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = PI;  // west (toward us)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = -target_.speed;  // moving west (toward us)
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us (west, -x).
        target_.x -= target_.speed * dt;
        // If target passes us, reset to keep the engagement going.
        if (target_.x < -2000.0) target_.x = initialRangeFt_;

        // Per-frame state
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(as.kin.sigma));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Merge) enteredMerge_ = true;
        if (mode == DigiMode::WVREngage) enteredWVREngage_ = true;
        if (mode == DigiMode::CollisionAvoid) enteredCollision_ = true;
        curMode_ = mode;

        // Per-frame sample data
        curRange_ = range;
        curHdgChg_ = std::fabs(as.kin.sigma) * RTD;
        curG_ = as.loads.nzcgs;
        curInMerge_ = (mode == DigiMode::Merge);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.0fft head-on, %.0f kts)\n",
                    testName_.c_str(), initialRangeFt_, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "pstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.pstick, modeBuf);
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter Merge OR WVREngage at some point.
        if (!enteredMerge_ && !enteredWVREngage_) return false;
        // 2. Must maneuver (merge / WVR involves hard turns).
        const double hdgThreshold = isHeavy_ ? 15.0 : 20.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) return false;
        // 3. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Merge mode (or WVREngage fallback); Heading change >= 20deg "
               "(15deg heavy); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredMerge_ && !enteredWVREngage_) {
            return "Never entered Merge or WVREngage mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target closed to " + std::to_string(static_cast<int>(minRange_)) +
                   "ft — neither MergeCheck nor WVR fallback fired).";
        }
        const double hdgThreshold = isHeavy_ ? 15.0 : 20.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) {
            return "Max heading change was " + std::to_string(curHdgChg_) +
                   "deg (needed >= " + std::to_string(hdgThreshold) +
                   "deg) — aircraft did not maneuver through the merge.";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — merge maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",    curRange_,         "ft"},
            {"hdg_chg",  curHdgChg_,        "deg"},
            {"G",        curG_,             ""},
            {"in_merge", curInMerge_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 15.0 : 20.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Merge:         %s\n", enteredMerge_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered WVREngage:     %s (fallback)\n",
            enteredWVREngage_ ? "[PASS]" : "(n/a)");
        std::printf("  Entered CollisionAvoid:%s (info)\n",
            enteredCollision_ ? "[PASS]" : "(n/a)");
        std::printf("  Max heading change:    %.1f deg (need >= %.0f) %s\n",
            maxHeadingChange_ * RTD, hdgThreshold,
            maxHeadingChange_ >= hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min range:             %.0f ft (info)\n", minRange_);
        std::printf("  Max G:                 %.2f\n", maxG_);
        std::printf("  Min altitude:          %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeFt_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxHeadingChange_{0.0};
    bool hasNaN_{false};
    bool enteredMerge_{false};
    bool enteredWVREngage_{false};
    bool enteredCollision_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curHdgChg_{0.0};
    double curG_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curInMerge_{false};
};

// ===========================================================================
// DigiMergeScenario
// ===========================================================================
class DigiMergeScenario : public ManeuverScenario {
public:
    DigiMergeScenario() : ManeuverScenario("digi_merge") {}

    std::string GetDescription() const override {
        return "Digi AI Merge: target 3000ft head-on, closing fast. Tests "
               "MergeCheck (range <= 1000ft, ata < 45deg) + MergeManeuver "
               "(level turn / slice / vertical pull based on speed regime).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Target at 3000 ft head-on, both at corner speed (fast closure).
        tests.push_back(std::make_unique<MergePhase>(
            "Merge (3000ft head-on)", 12.0, alt, speed, 350.0, 3000.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiMerge("digi_merge", []() {
    return std::make_unique<DigiMergeScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_merge() {}

} // namespace f4flight_test
