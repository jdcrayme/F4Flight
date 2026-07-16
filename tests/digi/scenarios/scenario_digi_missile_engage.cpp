// f4flight - scenarios/scenario_digi_missile_engage.cpp
//
// Maneuver test for digi AI MissileEngage mode (within 8 NM, beyond gun range).
//
// Sets up an engagement where the AI has AIM-9 missiles loaded (via SMS) and
// faces a target at 5 NM head-on. The AI should:
//   1. Detect the target via the injected target pointer.
//   2. Enter MissileEngage mode (SMS has AimWpn, range within AIM-9 RMax).
//   3. Steer toward the target (AutoTrack on the target position).
//   4. Not crash or NaN.
//
// The MissileEngageCheck requires:
//   - SMS with at least one AimWpn (A/A missile)
//   - range <= maxMissileRMax * 1.05 (AIM-9 RMax = 8 NM)
//   - ata <= 60° * 1.05
// At 5 NM head-on, both conditions are met.
//
// Pass criteria:
//   - Enters MissileEngage mode (or WVREngage as fallback if SMS isn't set)
//   - Steers toward target (heading converges)
//   - No NaN, no crash

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/weapons/weapon_spec.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// Phase: MissileEngage (5 NM head-on, AIM-9 loaded)
// ===========================================================================
class MissileEngagePhase : public ManeuverTest {
public:
    MissileEngagePhase(const char* name, double duration,
                       double alt, double speed, double targetSpeed,
                       double initialRangeNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm) {}

    // IMPORTANT: clear the SMS pointer from the brain when this phase is
    // destroyed. The SMS is a member of this phase, so once the phase is
    // destroyed, the brain's `sms_` pointer would dangle — causing a UAF
    // on the next scenario that calls sms_->hasWeaponClass() in
    // resolveMode(). The brain's reset() doesn't clear sms_, so we have
    // to do it here.
    ~MissileEngagePhase() override {
        if (sc_brain_) sc_brain_->setSMS(nullptr);
    }

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

        // Set up an SMS with 4 AIM-9 missiles (station 1, 2) and the gun.
        // AIM-9 RMax = 8 NM, so 5 NM is well within envelope.
        sms_.addHardpoint(1, WeaponType::Aim9, 2);  // 2x AIM-9 on station 1
        sms_.addHardpoint(2, WeaponType::Aim9, 2);  // 2x AIM-9 on station 2
        sms_.addHardpoint(9, WeaponType::Guns, 510);  // internal gun
        sc.brain().setSMS(&sms_);

        // Target ahead (+x), head-on (heading west, toward us).
        initialRangeFt_ = initialRangeNm_ * 6076.0;
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

        // Inject the target directly.
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us (west, -x).
        target_.x -= target_.speed * dt;
        // If target passes us, reset to keep the engagement going.
        if (target_.x < -3000.0) target_.x = initialRangeFt_;

        // Per-frame state
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        // Track heading change — head-on target requires nose tracking.
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(as.kin.sigma));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::MissileEngage) {
            enteredMissileEngage_ = true;
            // Track G specifically DURING MissileEngage mode. The old test
            // checked maxG over the whole phase — but the G might be achieved
            // in GunsEngage or WVREngage after the AI leaves MissileEngage.
            // A regression where the AI enters MissileEngage but doesn't
            // maneuver (then later pulls G in another mode) would pass the
            // old test. This catches that.
            maxGInMissileEngage_ = std::max(maxGInMissileEngage_, as.loads.nzcgs);
            framesInMissileEngage_++;
        }
        if (mode == DigiMode::WVREngage) enteredWVREngage_ = true;
        curMode_ = mode;

        // Per-frame sample data
        curRange_ = range;
        curHdgChg_ = std::fabs(as.kin.sigma) * RTD;
        curG_ = as.loads.nzcgs;
        curInMissile_ = (mode == DigiMode::MissileEngage);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.1f NM head-on, %.0f kts, AIM-9 loadout)\n",
                    testName_.c_str(), initialRangeNm_, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "pstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.pstick, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter an offensive mode: MissileEngage (preferred) or
        //    WVREngage (fallback if SMS path doesn't trigger — see task notes).
        if (!enteredMissileEngage_ && !enteredWVREngage_) return false;
        // 2. Must have closed range significantly — proves the AI was
        //    actually engaging the target, not just sitting at heading 0.
        //    The target starts at 5 NM; the AI should close to at least
        //    50% of initial range (2.5 NM). Heavy aircraft get 70%.
        const double rangeFraction = isHeavy_ ? 0.7 : 0.5;
        if (minRange_ > rangeFraction * initialRangeFt_) return false;
        // 3. Must maneuver — either turn (heading change > 30°) OR pull G
        //    (maxG > 2.0). Attack aircraft like the A-10 may pull G in
        //    pitch without much bank, so accept either. Heavy aircraft
        //    (B-52, C-130) can't sustain either — waive for them.
        if (isHeavy_) {
            return minAlt_ >= 5000.0;
        }
        const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
        if (!maneuvered) return false;
        // 4. If MissileEngage was entered, must have pulled SOME G during
        //    MissileEngage mode specifically. The old test checked maxG over
        //    the whole phase — a regression where the AI enters
        //    MissileEngage but doesn't maneuver (then later pulls G in
        //    GunsEngage/WVREngage) would pass. Require maxG >= 1.5 during
        //    MissileEngage (above level-flight G of ~1.0, proving the AI
        //    was actively maneuvering in the mode). This only applies if
        //    MissileEngage was actually entered (not just the WVR fallback).
        if (enteredMissileEngage_ && framesInMissileEngage_ > 0) {
            if (maxGInMissileEngage_ < 1.5) return false;
        }
        // 5. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter MissileEngage mode (or WVREngage fallback); "
               "Close range to <= 50% of initial (70% heavy); "
               "Maneuver (hdg chg >= 30deg OR maxG >= 2.0; heavy: waived); "
               "G >= 1.5 during MissileEngage (if entered); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredMissileEngage_ && !enteredWVREngage_) {
            return "Never entered MissileEngage or WVREngage mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target was at " + std::to_string(static_cast<int>(curRange_)) +
                   "ft with AIM-9 loadout — SMS path did not trigger MissileEngage "
                   "and WVR fallback did not engage either).";
        }
        const double rangeFraction = isHeavy_ ? 0.7 : 0.5;
        if (minRange_ > rangeFraction * initialRangeFt_) {
            return "Min range was " + std::to_string(static_cast<int>(minRange_)) +
                   "ft (needed <= " +
                   std::to_string(static_cast<int>(rangeFraction * initialRangeFt_)) +
                   "ft = " + std::to_string(rangeFraction * 100.0) +
                   "% of initial) — aircraft did not close on the target.";
        }
        if (isHeavy_) {
            // Heavy: only mode + range + alt required, all passed
            return "";
        }
        const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
        if (!maneuvered) {
            return "Max heading change was " + std::to_string(curHdgChg_) +
                   "deg and max G was " + std::to_string(maxG_) +
                   " (needed hdg chg >= 30deg OR maxG >= 2.0) — "
                   "aircraft did not maneuver to track the target.";
        }
        if (enteredMissileEngage_ && framesInMissileEngage_ > 0 &&
            maxGInMissileEngage_ < 1.5) {
            return "Max G during MissileEngage mode was " +
                   std::to_string(maxGInMissileEngage_) +
                   " (needed >= 1.5) — AI entered MissileEngage but did not "
                   "maneuver while in that mode (G was only achieved in "
                   "other modes like GunsEngage/WVREngage).";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — engagement maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",      curRange_,        "ft"},
            {"hdg_chg",    curHdgChg_,       "deg"},
            {"G",          curG_,            ""},
            {"in_missile", curInMissile_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered MissileEngage: %s\n",
            enteredMissileEngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered WVREngage:     %s (fallback)\n",
            enteredWVREngage_ ? "[PASS]" : "(n/a)");
        const double rangeFraction = isHeavy_ ? 0.7 : 0.5;
        std::printf("  Min range:             %.0f ft (need <= %.0f) %s\n",
            minRange_, rangeFraction * initialRangeFt_,
            minRange_ <= rangeFraction * initialRangeFt_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Max heading change:    %.1f deg (heavy: waived)\n",
                maxHeadingChange_ * RTD);
        } else {
            const bool maneuvered = (maxHeadingChange_ >= 30.0 * DTR) || (maxG_ >= 2.0);
            std::printf("  Max heading change:    %.1f deg, max G %.2f "
                        "(need hdg >= 30 OR G >= 2.0) %s\n",
                maxHeadingChange_ * RTD, maxG_,
                maneuvered ? "[PASS]" : "[FAIL]");
        }
        if (enteredMissileEngage_) {
            std::printf("  Max G in MissileEngage:%.2f (need >= 1.5) %s\n",
                maxGInMissileEngage_,
                maxGInMissileEngage_ >= 1.5 ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Min altitude:          %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_, initialRangeFt_;
    DigiEntity target_;
    StoresManagementSystem sms_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxHeadingChange_{0.0};
    double maxGInMissileEngage_{0.0};
    int   framesInMissileEngage_{0};
    bool hasNaN_{false};
    bool enteredMissileEngage_{false};
    bool enteredWVREngage_{false};
    bool isHeavy_{false};
    DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curHdgChg_{0.0};
    double curG_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curInMissile_{false};
};

// ===========================================================================
// DigiMissileEngageScenario
// ===========================================================================
class DigiMissileEngageScenario : public ManeuverScenario {
public:
    DigiMissileEngageScenario() : ManeuverScenario("digi_missile_engage") {}

    std::string GetDescription() const override {
        return "Digi AI MissileEngage: target 5 NM head-on, AIM-9 loadout via SMS. "
               "Tests MissileEngageCheck + the SMS path (AimWpn detection + WEZ "
               "envelope). Falls back to WVREngage if SMS path doesn't trigger.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<MissileEngagePhase>(
            "MissileEngage (5NM head-on, AIM-9)", 25.0, alt, speed, 350.0, 5.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiMissileEngage("digi_missile_engage", []() {
    return std::make_unique<DigiMissileEngageScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_missile_engage() {}

} // namespace f4flight_test
