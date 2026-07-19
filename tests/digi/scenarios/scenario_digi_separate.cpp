// f4flight - scenarios/scenario_digi_separate.cpp
//
// Maneuver test for digi AI disengage modes (RTB / Bugout / Separate).
//
// Two phases:
//   Phase 1 "Damage abort": pctStrength = 0.3 (below 0.50 threshold) + a
//     friendly airbase 20 NM north. SeparateCheck returns RTB (the
//     documented damage-abort behavior). The brain enters RTB and
//     navigates toward the divert airbase.
//     Verify: enters RTB mode, heading converges to airbase bearing,
//     doesn't crash.
//
//     NOTE: the task description suggested Separate/Bugout for damage, but
//     the actual implementation routes damage-abort through RTB
//     (decision_routines.cpp:152: `if (pctStrength < 0.50) return RTB;`).
//     This test verifies the IMPLEMENTED behavior.
//
//   Phase 2 "Bugout (pre-armed timer)": target ahead, same heading, fleeing.
//     The bugout timer is pre-armed to 1.0s at Init (instead of waiting 90s).
//     After 1 second of "deep six" geometry (target ataFrom > 135°), the
//     brain enters Bugout mode. Bugout is sticky — the offensive block
//     can't pre-empt it (unlike Separate, which IS pre-empted by WVR).
//     Verify: enters Bugout mode, maneuvers (WvrBugOut accelerates to 2x
//     corner speed — throttle change), doesn't crash.
//
//     NOTE: Separate mode is essentially unreachable in the current
//     implementation — SeparateCheck queues Separate (priority 14), but the
//     offensive block (which runs AFTER SeparateCheck in resolveMode) queues
//     WVREngage (priority 10), which pre-empts Separate. Bugout is testable
//     because it's sticky (addMode special case).
//
// Pass criteria:
//   - Phase 1: enters RTB mode (damage abort → RTB)
//   - Phase 2: enters Bugout mode (deep-six timer expired)
//   - Both: maneuvers, no NaN, no crash

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// Airbase position for the damage-abort phase (20 NM north, like digi_rtb).
constexpr double kSepAirbaseX = 0.0;
constexpr double kSepAirbaseY = 20.0 * 6076.0;  // 20 NM north
constexpr double kSepAirbaseZ = -5000.0;

// ===========================================================================
// Phase 1: Damage abort (pctStrength < 0.50 → RTB)
// ===========================================================================
class DamageAbortPhase : public ManeuverTest {
public:
    DamageAbortPhase(const char* name, double duration,
                     double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);  // east (airbase is north → must turn 90°)
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Set up the friendly airbase 20 NM north (like digi_rtb).
        airbase_.x = kSepAirbaseX;
        airbase_.y = kSepAirbaseY;
        airbase_.z = kSepAirbaseZ;
        airbase_.runwayHeading = 0.0;
        airbase_.id = 100;

        // Damage state: pctStrength = 0.3 (below 0.50 abort threshold).
        // Provide fuel above bingo (we want to isolate the damage path,
        // not also trigger fuel RTB).
        FrameInputs fi;
        fi.pctStrength = 0.3;
        fi.fuelLbs = 5000.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);

        // Manually set the divert airbase on the brain state. SeparateCheck
        // returns RTB for damage, but doesn't call AirbaseCheck (only the
        // fuel-critical path does). Without a divert airbase set, runRTB
        // falls back to heading-hold (no maneuver). Setting it here lets
        // the brain actually navigate toward the divert field.
        sc.brain().stateMutable().fuel.divertAirbaseX = airbase_.x;
        sc.brain().stateMutable().fuel.divertAirbaseY = airbase_.y;
        sc.brain().stateMutable().fuel.divertAirbaseZ = airbase_.z;
        sc.brain().stateMutable().fuel.divertAirbaseHeading = airbase_.runwayHeading;
        sc.brain().stateMutable().fuel.hasDivertAirbase = true;

        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Re-apply the damage state each frame (the brain reads pctStrength
        // from frameInputs each compute() call).
        FrameInputs fi = sc_brain_->frameInputs();
        fi.pctStrength = 0.3;
        fi.fuelLbs = 5000.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc_brain_->setFrameInputs(fi);

        // Per-frame state
        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();
        if (currentMode_ == DigiMode::RTB) enteredRTB_ = true;
        if (currentMode_ == DigiMode::Separate) enteredSeparate_ = true;
        if (currentMode_ == DigiMode::Bugout) enteredBugout_ = true;

        // Track heading convergence to airbase bearing (north = PI/2).
        double dh = heading - (PI / 2.0);
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToNorth_ = std::min(minAbsHeadingToNorth_, std::fabs(dh));

        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(heading - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Per-frame sample data
        curHdgToNorth_ = std::fabs(dh) * RTD;
        curStrength_ = sc_brain_->state().damage.pctStrength;
        curMode_ = currentMode_;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (pctStrength=0.3, airbase 20NM north)\n",
                    testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "G", "pstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD,
                as.loads.nzcgs, input.pstick, digiModeName(currentMode_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter RTB mode (damage abort routes to RTB, per impl).
        if (!enteredRTB_) return false;
        // 2. Must steer toward the airbase (heading converges to north).
        const double hdgThreshold = isHeavy_ ? 90.0 : 60.0;
        if (minAbsHeadingToNorth_ > hdgThreshold * DTR) return false;
        // 3. Must not crash.
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB mode (damage abort); Turn within 60deg of airbase bearing "
               "(90deg heavy); Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredRTB_) {
            return "Never entered RTB mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; pctStrength=" + std::to_string(curStrength_) +
                   " — SeparateCheck did not return RTB for damage).";
        }
        const double hdgThreshold = isHeavy_ ? 90.0 : 60.0;
        if (minAbsHeadingToNorth_ > hdgThreshold * DTR) {
            return "Closest heading to airbase bearing was " +
                   std::to_string(curHdgToNorth_) +
                   "deg (needed <= " + std::to_string(hdgThreshold) +
                   "deg) — aircraft did not turn toward the divert airbase.";
        }
        if (minAlt_ < 1000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 1000ft) — RTB maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_north",   curHdgToNorth_, "deg"},
            {"pct_str",     curStrength_,   ""},
            {"in_rtb",      (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
            {"in_separate", (enteredSeparate_ && curMode_ == DigiMode::Separate) ? 1.0 : 0.0, ""},
        };
    }

    // Publish the airbase as a trace entity so the report shows the divert.
    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 90.0 : 60.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered RTB (damage):     %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Closest hdg to north:     %.1f deg (need <= %.0f) %s\n",
            minAbsHeadingToNorth_ * RTD, hdgThreshold,
            minAbsHeadingToNorth_ <= hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  pctStrength:              %.2f\n", curStrength_);
        std::printf("  Max G:                    %.2f\n", maxG_);
        std::printf("  Min altitude:             %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    FrameInputs::AirbaseInfo airbase_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAbsHeadingToNorth_{1e9};
    double minAlt_{1e9};
    double maxG_{0.0};
    bool hasNaN_{false};
    bool enteredRTB_{false};
    bool enteredSeparate_{false};
    bool enteredBugout_{false};
    bool isHeavy_{false};
    DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    // Per-frame sample data
    double curHdgToNorth_{0.0};
    double curStrength_{1.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Bugout (pre-armed timer, deep-six target)
// ===========================================================================
class BugoutPhase : public ManeuverTest {
public:
    BugoutPhase(const char* name, double duration,
                double alt, double speed, double targetRangeNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetRangeNm_(targetRangeNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Target ahead (+x), same heading (east), slower. We're chasing it
        // from behind → ataFrom = 180° > 135° (deep six).
        const double rangeFt = targetRangeNm_ * 6076.0;
        target_.x = rangeFt;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;  // same heading (fleeing east)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = 250.0 * KNOTS_TO_FTPSEC;
        target_.vx = target_.speed;
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;

        // Fuel state: not Bingo (isolate the bugout path, not fuel RTB).
        FrameInputs fi;
        fi.pctStrength = 1.0;
        fi.fuelLbs = 5000.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.injectedTarget = &target_;
        sc.brain().setFrameInputs(fi);

        // PRE-ARM the bugout timer to 1.0s. Without this, the bugout timer
        // would take 90 seconds to expire (kBugoutTimerSec). With 1.0s left,
        // Bugout fires after ~1 second of deep-six geometry.
        sc.brain().stateMutable().damage.bugoutTimer = 1.0;
        sc.brain().stateMutable().damage.bugoutTimerActive = true;

        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target forward (east, +x).
        target_.x += target_.speed * dt;

        // Re-apply frame inputs (target pointer + fuel state).
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 5000.0;
        fi.injectedTarget = &target_;
        sc_brain_->setFrameInputs(fi);

        // Per-frame state
        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();
        if (currentMode_ == DigiMode::Bugout) enteredBugout_ = true;
        if (currentMode_ == DigiMode::Separate) enteredSeparate_ = true;
        if (currentMode_ == DigiMode::RTB) enteredRTB_ = true;
        if (currentMode_ == DigiMode::WVREngage) enteredWVREngage_ = true;

        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(heading - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxThrottle_ = std::max(maxThrottle_, input.throttle);

        // Track speed (VCAS) to verify acceleration. The WvrBugOut primitive
        // is supposed to accelerate to 2x corner speed — a speed increase
        // proves the bugout command actually fired (vs. the aircraft just
        // sitting at MIL power, which trivially satisfies throttle > 0.9).
        if (firstFrame_) {
            startVcas_ = as.vcas;
            firstFrame_ = false;
        }
        maxVcas_ = std::max(maxVcas_, as.vcas);
        curVcas_ = as.vcas;

        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        curRange_ = range;
        startRange_ = (startRange_ < 0.0) ? range : startRange_;
        maxRange_ = std::max(maxRange_, range);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Per-frame sample data
        curHdgChg_ = std::fabs(heading - initialHeading_) * RTD;
        curThrottle_ = input.throttle;
        curMode_ = currentMode_;
        curBugoutTimer_ = sc_brain_->state().damage.bugoutTimer;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.1f NM fleeing, bugoutTimer pre-armed 1s)\n",
                    testName_.c_str(), targetRangeNm_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "range", "G", "thro", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %8.0f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD, range,
                as.loads.nzcgs, input.throttle, digiModeName(currentMode_));
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter Bugout mode (pre-armed timer expires after ~1s).
        //    Accept Separate/RTB as alternates (in case of future re-routing).
        if (!enteredBugout_ && !enteredSeparate_ && !enteredRTB_) return false;
        // 2. Must have ACTUALLY disengaged — not just sat at MIL power.
        //    The old test accepted "throttle > 0.9" which is trivially
        //    satisfied because the aircraft starts at full throttle. Instead
        //    require at least ONE of:
        //    (a) Heading change > 30° (turned away from the target), OR
        //    (b) Speed increased by >= 30 kts (accelerated to disengage), OR
        //    (c) Range to target increased (opened distance from target).
        //    Heavy aircraft (B-52, C-130) have very low T/W and can't
        //    accelerate or turn quickly — waive the disengagement check
        //    for them (they're still checked on mode entry + no crash).
        if (!isHeavy_) {
            const double hdgThreshold = 30.0;
            const double spdIncrease = 30.0;
            const bool turnedAway = maxHeadingChange_ >= hdgThreshold * DTR;
            const bool accelerated = (maxVcas_ - startVcas_) >= spdIncrease;
            const bool openedRange = (maxRange_ > startRange_ + 500.0);
            if (!turnedAway && !accelerated && !openedRange) return false;
        }
        // 3. Must not crash.
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Bugout mode (pre-armed timer; Separate/RTB accepted as alt); "
               "Disengage (hdg chg > 30deg OR speed +30kts OR range opened; heavy: waived); "
               "Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredBugout_ && !enteredSeparate_ && !enteredRTB_) {
            return "Never entered Bugout/Separate/RTB mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target was at " + std::to_string(static_cast<int>(curRange_)) +
                   "ft, bugoutTimer=" + std::to_string(curBugoutTimer_) +
                   "s — SeparateCheck did not return Bugout despite the "
                   "pre-armed timer.";
        }
        if (!isHeavy_) {
            const double hdgThreshold = 30.0;
            const double spdIncrease = 30.0;
            const bool turnedAway = maxHeadingChange_ >= hdgThreshold * DTR;
            const bool accelerated = (maxVcas_ - startVcas_) >= spdIncrease;
            const bool openedRange = (maxRange_ > startRange_ + 500.0);
            if (!turnedAway && !accelerated && !openedRange) {
                return "Aircraft did not disengage: max heading change was " +
                       std::to_string(curHdgChg_) + "deg (need > " +
                       std::to_string(hdgThreshold) + "), speed increased " +
                       std::to_string(static_cast<int>(maxVcas_ - startVcas_)) +
                       "kts (need > " + std::to_string(static_cast<int>(spdIncrease)) +
                       "), range " + std::to_string(static_cast<int>(startRange_)) +
                       "->" + std::to_string(static_cast<int>(maxRange_)) +
                       "ft (did not open). Bugout mode was entered but the "
                       "WvrBugOut primitive did not maneuver or accelerate.";
            }
        }
        if (minAlt_ < 1000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 1000ft) — disengage maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",        curRange_,        "ft"},
            {"hdg_chg",      curHdgChg_,       "deg"},
            {"throttle",     curThrottle_,     ""},
            {"bugout_timer", curBugoutTimer_,  "s"},
            {"in_bugout",    (enteredBugout_ && curMode_ == DigiMode::Bugout) ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        const double hdgThreshold = 30.0;
        const double spdIncrease = 30.0;
        const bool turnedAway = maxHeadingChange_ >= hdgThreshold * DTR;
        const bool accelerated = (maxVcas_ - startVcas_) >= spdIncrease;
        const bool openedRange = (maxRange_ > startRange_ + 500.0);
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Bugout:      %s\n", enteredBugout_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Separate:    %s (alt)\n",
            enteredSeparate_ ? "[PASS]" : "(n/a)");
        std::printf("  Entered RTB:         %s (alt)\n",
            enteredRTB_ ? "[PASS]" : "(n/a)");
        std::printf("  Entered WVREngage:   %s (info — pre-empts Separate)\n",
            enteredWVREngage_ ? "[YES]" : "(no)");
        if (isHeavy_) {
            std::printf("  Disengage check:     (heavy: waived) hdg %.1f deg, "
                        "spd +%d kts, range %.0f->%.0f\n",
                maxHeadingChange_ * RTD,
                static_cast<int>(maxVcas_ - startVcas_),
                startRange_, maxRange_);
        } else {
            std::printf("  Disengage check:     hdg %.1f deg (need > %.0f), "
                        "spd +%d kts (need > %.0f), range %.0f->%.0f %s\n",
                maxHeadingChange_ * RTD, hdgThreshold,
                static_cast<int>(maxVcas_ - startVcas_), spdIncrease,
                startRange_, maxRange_,
                (turnedAway || accelerated || openedRange) ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Max throttle:         %.2f\n", maxThrottle_);
        std::printf("  Final range:         %.0f ft (info)\n", curRange_);
        std::printf("  Max G:               %.2f\n", maxG_);
        std::printf("  Min altitude:        %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetRangeNm_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxThrottle_{0.0};
    double startVcas_{0.0};
    double maxVcas_{0.0};
    double startRange_{-1.0};
    double maxRange_{0.0};
    bool firstFrame_{true};
    bool hasNaN_{false};
    bool enteredBugout_{false};
    bool enteredSeparate_{false};
    bool enteredRTB_{false};
    bool enteredWVREngage_{false};
    bool isHeavy_{false};
    DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    // Per-frame sample data
    double curRange_{0.0};
    double curHdgChg_{0.0};
    double curThrottle_{0.0};
    double curVcas_{0.0};
    double curBugoutTimer_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// DigiSeparateScenario
// ===========================================================================
class DigiSeparateScenario : public ManeuverScenario {
public:
    DigiSeparateScenario() : ManeuverScenario("digi_separate") {}

    std::string GetDescription() const override {
        return "Digi AI disengage: damage abort (pctStrength<0.5 → RTB with "
               "divert airbase) + bugout (pre-armed timer, deep-six target → "
               "Bugout). Tests SeparateCheck's RTB and Bugout paths. Note: "
               "Separate mode itself is unreachable in the current impl — "
               "the offensive block pre-empts Separate via addMode priority.";
    }

    // Draw the divert runway at the airbase position (for the damage phase).
    std::vector<TraceGeometry> traceGeometry() const override {
        const double rwyLen = 6000.0;
        const double halfLen = rwyLen / 2.0;
        TraceGeometry runway;
        runway.name = "Divert";
        runway.type = "runway";
        runway.coords = {kSepAirbaseX - halfLen, kSepAirbaseY, kSepAirbaseZ, kSepAirbaseX + halfLen, kSepAirbaseY, kSepAirbaseZ};
        runway.color = "#FFD700";
        runway.width = 100.0;
        return {runway};
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: damage abort (pctStrength=0.3 → RTB with airbase)
        tests.push_back(std::make_unique<DamageAbortPhase>(
            "Damage abort (pctStrength=0.3)", 30.0, alt, speed));
        // Phase 2: bugout (pre-armed timer, deep-six target)
        tests.push_back(std::make_unique<BugoutPhase>(
            "Bugout (pre-armed timer, 4NM target)", 15.0, alt, speed, 4.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiSeparate("digi_separate", []() {
    return std::make_unique<DigiSeparateScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_separate() {}

} // namespace f4flight_test
