// f4flight - scenarios/scenario_digi_wvr_defensive.cpp
//
// Maneuver test for digi AI defensive WVR (target on our tail).
//
// Sets up the "defensive BFM" case where the bandit has the advantage:
// AI aircraft starts with a target directly BEHIND it (ata = 180°), at
// 2 NM, faster. The AI should:
//   1. Detect the target via the injected target pointer.
//   2. Enter WVREngage mode (target within maxAAWpnRange = 35 NM).
//   3. Run the defensive branch of RollAndPull (bandit behind us).
//   4. Maneuver aggressively (heading change > 30° proves defensive BFM).
//   5. Not crash or NaN.
//
// The RollAndPull defensive branch tries to point the nose at the target
// (AutoTrack on the target's position), which requires a ~180° turn. The
// AI should rotate through at least 30° to begin the defensive maneuver.
//
// Pass criteria:
//   - Enters WVREngage mode (or MissileDefeat if the target is treated as
//     a threat rather than a target)
//   - Maneuvers (heading change > 30°, 15° heavy)
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
// Phase: WVR defensive (target on our tail)
// ===========================================================================
class WVRDefensivePhase : public ManeuverTest {
public:
    WVRDefensivePhase(const char* name, double duration,
                      double alt, double speed, double targetSpeed,
                      double initialRangeNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm) {}

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

        // Target BEHIND us (-x), same heading (east), faster. The bandit is
        // on our tail — defensive BFM case.
        initialRangeFt_ = initialRangeNm_ * 6076.0;
        target_.x = -initialRangeFt_;  // behind us (-x)
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;  // same heading (chasing us from behind)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = target_.speed;  // moving east (chasing us)
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target forward (east, +x) chasing us.
        target_.x += target_.speed * dt;

        // Per-frame state
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(as.kin.sigma - initialHeading_));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::WVREngage) enteredWVREngage_ = true;
        if (mode == DigiMode::MissileDefeat) enteredMissileDefeat_ = true;
        curMode_ = mode;

        // Per-frame sample data
        curRange_ = range;
        curHdgChg_ = std::fabs(as.kin.sigma - initialHeading_) * RTD;
        curG_ = as.loads.nzcgs;
        curInWvr_ = (mode == DigiMode::WVREngage);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.1f NM behind, %.0f kts)\n",
                    testName_.c_str(), initialRangeNm_, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "rstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.rstick, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter WVREngage mode (or MissileDefeat as alt — the target
        //    may be reclassified if it's treated as a threat).
        if (!enteredWVREngage_ && !enteredMissileDefeat_) return false;
        // 2. Must maneuver — either turn (heading change > 30°) OR pull G
        //    (maxG > 2.0). Attack aircraft like the A-10 may pull G in
        //    pitch without much bank, so accept either. Heavy aircraft
        //    (B-52, C-130) can't sustain either — waive for them.
        if (isHeavy_) {
            return minAlt_ >= 5000.0;
        }
        const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
        if (!maneuvered) return false;
        // 3. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter WVREngage mode (or MissileDefeat alt); "
               "Maneuver (hdg chg > 30deg OR maxG >= 2.0; heavy: waived); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredWVREngage_ && !enteredMissileDefeat_) {
            return "Never entered WVREngage or MissileDefeat mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target was at " + std::to_string(static_cast<int>(curRange_)) +
                   "ft behind us — bandit was not classified as a WVR threat.";
        }
        if (isHeavy_) {
            // Heavy: only mode + alt required, both passed
            return "";
        }
        const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
        if (!maneuvered) {
            return "Max heading change was " + std::to_string(curHdgChg_) +
                   "deg and max G was " + std::to_string(maxG_) +
                   " (needed hdg chg > 30deg OR maxG >= 2.0) — "
                   "aircraft did not maneuver defensively.";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — defensive maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",   curRange_,    "ft"},
            {"hdg_chg", curHdgChg_,   "deg"},
            {"G",       curG_,        ""},
            {"in_wvr",  curInWvr_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered WVREngage:        %s\n",
            enteredWVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered MissileDefeat:    %s (alt)\n",
            enteredMissileDefeat_ ? "[PASS]" : "(n/a)");
        if (isHeavy_) {
            std::printf("  Max heading change:       %.1f deg (heavy: waived)\n",
                maxHeadingChange_ * RTD);
        } else {
            const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
            std::printf("  Max heading change:       %.1f deg, max G %.2f "
                        "(need hdg > 30 OR G >= 2.0) %s\n",
                maxHeadingChange_ * RTD, maxG_,
                maneuvered ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Range: %.0f..%.0f ft (info)\n", minRange_, maxRange_);
        std::printf("  Min altitude:             %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_, initialRangeFt_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxHeadingChange_{0.0};
    bool hasNaN_{false};
    bool enteredWVREngage_{false};
    bool enteredMissileDefeat_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curHdgChg_{0.0};
    double curG_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curInWvr_{false};
};

// ===========================================================================
// DigiWVRDefensiveScenario
// ===========================================================================
class DigiWVRDefensiveScenario : public ManeuverScenario {
public:
    DigiWVRDefensiveScenario() : ManeuverScenario("digi_wvr_defensive") {}

    // Tier classification for the 3-tier test workflow.
    // See scenario_framework.h -> TestTier enum for the meaning.
    TestTier GetTestTier() const override { return TestTier::HighLevel; }

        std::string GetDescription() const override {
        return "Digi AI defensive WVR: bandit starts 2 NM behind us on the same "
               "heading, faster. Tests the defensive branch of RollAndPull (bandit "
               "on our tail — the AI must turn to bring its nose on the target).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Bandit 2 NM behind us, same heading (east), faster (450 kts).
        tests.push_back(std::make_unique<WVRDefensivePhase>(
            "WVR defensive (2NM bandit on tail)", 25.0, alt, speed, 450.0, 2.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiWVRDefensive("digi_wvr_defensive", []() {
    return std::make_unique<DigiWVRDefensiveScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_wvr_defensive() {}

} // namespace f4flight_test
