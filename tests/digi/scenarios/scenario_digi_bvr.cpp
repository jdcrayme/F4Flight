// f4flight - scenarios/scenario_digi_bvr.cpp
//
// Maneuver test for digi AI BVR (Beyond Visual Range) engagement.
//
// Sets up a BVR engagement: AI aircraft starts 15 NM from a target flying
// the same heading, slower. The AI should:
//   1. Detect the target via the injected target pointer.
//   2. Enter BVREngage mode (range > 8 NM, within engageRange).
//   3. Steer toward the target via the chosen BVR tactic (Pursuit/Crank/etc.).
//   4. Not crash or NaN.
//
// The BvrEngageCheck requires:
//   - range > 8 NM (RAP distance — inside, MissileEngage handles)
//   - range <= engageRange = max(maxAAWpnRange * 1.3, 45 NM)
// At 15 NM, both conditions are met (8 < 15 < 45.5).
//
// Pass criteria:
//   - Enters BVREngage mode at some point
//   - Heading converges toward target bearing (proves steering toward target)
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
// Phase: BVR engagement
//
// Target 15 NM ahead (east, +x), same heading (east), slower (250 kts).
// The AI at corner speed (350 kts) is faster and closes slowly — the
// engagement stays in the BVR band for the full 30 s.
// ===========================================================================
class BVREngagePhase : public ManeuverTest {
public:
    BVREngagePhase(const char* name, double duration,
                   double alt, double speed, double targetSpeed,
                   double initialRangeNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);

        // Set up the FlightPlan with a Navigate task ahead (east)
        auto fp = std::make_shared<FlightPlan>();
        fp->pushTask(MissionTask{TaskType::Navigate, {607600.0, 0.0, -alt_}, speed_, alt_, kInvalidEntityId, 0.0});
        sc.brain().setFlightPlan(fp);

        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());
        // Record the aircraft's corner speed — used to determine if range
        // closure is a meaningful check. If the aircraft's corner speed is
        // close to the target speed (250 kts), the aircraft can't close
        // range regardless of AI behavior, so the range closure check is
        // waived.
        aircraftCornerSpeed_ = speed_;

        // Target ahead (+x), same heading (east, yaw=0), slower.
        // Both fly east — AI catches up from behind, slowly closing range.
        initialRangeFt_ = initialRangeNm_ * 6076.0;
        target_.x = initialRangeFt_;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;  // same heading (chase from behind)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = target_.speed;  // moving east (away)
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;

        // Inject the target directly (bypasses SensorFusion — like digi_wvr).
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target forward at its speed (east, +x).
        target_.x += target_.speed * dt;

        // Per-frame state
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        // Heading error to target bearing (target is at +x = east = heading 0).
        // Wrap to [-PI, PI].
        double dh = as.kin.sigma - 0.0;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        const double absHdgErr = std::fabs(dh);
        minAbsHeadingToEast_ = std::min(minAbsHeadingToEast_, absHdgErr);
        // Track the FINAL heading error (proves the AI was still engaging at
        // the end, not just at t=0 before any maneuver). The min-heading
        // check is trivially 0 at t=0 because the aircraft starts heading
        // east — without the final-heading check, the test would pass even
        // if the AI immediately turned 90° away from the target.
        finalAbsHeadingToEast_ = absHdgErr;
        // Count frames where the heading was within 30° of the target
        // bearing (sustained pursuit, not just a momentary alignment).
        if (absHdgErr <= 30.0 * DTR) framesOnHeading_++;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::BVREngage) {
            enteredBVREngage_ = true;
            framesInBvr_++;
        }
        curMode_ = mode;
        totalFrames_++;

        // Per-frame sample data
        curRange_ = range;
        curHdgErr_ = std::fabs(dh) * RTD;
        curG_ = as.loads.nzcgs;
        curInBvr_ = (mode == DigiMode::BVREngage);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.1f NM ahead, %.0f kts)\n",
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
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered BVREngage mode.
        if (!enteredBVREngage_) return false;
        // 2. Must have STAYED in BVREngage for a sustained period (at least
        //    50% of the phase). A momentary mode entry that immediately
        //    exits is not a real engagement. This catches regressions where
        //    the mode is entered but the brain can't sustain it.
        if (totalFrames_ > 0 && framesInBvr_ < totalFrames_ / 2) return false;
        // 3. Must have closed range on the target by at least 1.5 NM — proves
        //    the AI was actually pursuing, not just sitting at heading 0.
        //    (The aircraft starts at heading 0 = target bearing, so a heading
        //    check alone is trivially satisfied at t=0. Range closure proves
        //    active engagement.) The pure speed differential (350 vs 250 kts)
        //    gives ~0.83 NM closure in 30s — requiring 1.5 NM means the
        //    aircraft must also accelerate (proving the pursuit is active,
        //    not just physics). Heavy aircraft (B-52, C-130) and slow aircraft
        //    (A-10, corner speed ~250 kts) fly at ~the SAME speed as the
        //    target — so they can't close range regardless of AI behavior.
        //    Waive the range closure check for them (they're still checked on
        //    mode + heading + sustained).
        const bool canCloseRange = !isHeavy_ && (aircraftCornerSpeed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * 6076.0;
            if (maxRange_ - minRange_ < requiredClosureFt) return false;
        }
        // 4. Final heading must still be roughly toward the target (within
        //    45° of east). Proves the AI didn't wander off at the end.
        //    (minAbsHeadingToEast_ is trivially 0 at t=0 — the final heading
        //    check is the meaningful one.)
        const double hdgThreshold = isHeavy_ ? 90.0 : 45.0;
        if (finalAbsHeadingToEast_ > hdgThreshold * DTR) return false;
        // 5. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter BVREngage mode; Sustain BVREngage >= 50% of phase; "
               "Close range by >= 1.5NM (heavy/slow: waived); Final heading within 45deg of "
               "target bearing (90deg heavy); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredBVREngage_) {
            return "Never entered BVREngage mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target was at " + std::to_string(static_cast<int>(curRange_)) +
                   "ft — BvrEngageCheck did not pass (range may have closed "
                   "inside 8 NM, or target was not injected).";
        }
        if (totalFrames_ > 0 && framesInBvr_ < totalFrames_ / 2) {
            return "BVREngage was only active for " + std::to_string(framesInBvr_) +
                   " of " + std::to_string(totalFrames_) +
                   " frames — mode was not sustained (brain kept exiting BVREngage).";
        }
        const bool canCloseRange = !isHeavy_ && (aircraftCornerSpeed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * 6076.0;
            if (maxRange_ - minRange_ < requiredClosureFt) {
                return "Range only closed from " +
                       std::to_string(static_cast<int>(maxRange_)) + " to " +
                       std::to_string(static_cast<int>(minRange_)) +
                       "ft (needed >= " + std::to_string(static_cast<int>(requiredClosureFt)) +
                       "ft closure = 1.5NM) — aircraft did not actively pursue the target.";
            }
        }
        const double hdgThreshold = isHeavy_ ? 90.0 : 45.0;
        if (finalAbsHeadingToEast_ > hdgThreshold * DTR) {
            return "Final heading error to target bearing was " +
                   std::to_string(curHdgErr_) +
                   "deg (needed <= " + std::to_string(hdgThreshold) +
                   "deg) — aircraft wandered off the target bearing by end of phase.";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — BVR maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",   curRange_,  "ft"},
            {"hdg_err", curHdgErr_, "deg"},
            {"G",       curG_,      ""},
            {"in_bvr",  curInBvr_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget()
    // (set by sc.setTarget(&target_) in Init). No need to publish here.

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 90.0 : 45.0;
        const double requiredClosureFt = 1.5 * 6076.0;
        const bool canCloseRange = !isHeavy_ && (aircraftCornerSpeed_ > 300.0);
        std::printf("  --- Summary ---\n");
        std::printf("  Entered BVREngage:        %s\n", enteredBVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Sustained BVREngage:      %d/%d frames %s\n",
            framesInBvr_, totalFrames_,
            (totalFrames_ > 0 && framesInBvr_ >= totalFrames_ / 2) ? "[PASS]" : "[FAIL]");
        if (!canCloseRange) {
            std::printf("  Range closure:            %.0f ft (heavy/slow: waived)\n",
                maxRange_ - minRange_);
        } else {
            std::printf("  Range closure:            %.0f ft (need >= %.0f) %s\n",
                maxRange_ - minRange_, requiredClosureFt,
                (maxRange_ - minRange_ >= requiredClosureFt) ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Final hdg to east:        %.1f deg (need <= %.0f) %s\n",
            finalAbsHeadingToEast_ * RTD, hdgThreshold,
            finalAbsHeadingToEast_ <= hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Range: %.0f..%.0f ft (info)\n", minRange_, maxRange_);
        std::printf("  Max G:                    %.2f\n", maxG_);
        std::printf("  Min altitude:             %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_, initialRangeFt_;
    double aircraftCornerSpeed_{0.0};
    DigiEntity target_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double minAbsHeadingToEast_{std::numeric_limits<double>::max()};
    double finalAbsHeadingToEast_{0.0};
    int   framesInBvr_{0};
    int   totalFrames_{0};
    int   framesOnHeading_{0};
    bool hasNaN_{false};
    bool enteredBVREngage_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curHdgErr_{0.0};
    double curG_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curInBvr_{false};
};

// ===========================================================================
// DigiBVRScenario
// ===========================================================================
class DigiBVRScenario : public ManeuverScenario {
public:
    DigiBVRScenario() : ManeuverScenario("digi_bvr") {}

    std::string GetDescription() const override {
        return "Digi AI BVR engagement: target 15 NM ahead, same heading, slower. "
               "Tests BvrEngageCheck + BvrEngage dispatch (Pursuit/Crank/Beam/Drag "
               "tactics).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 20000.0;  // high altitude for BVR (missile kinematics)
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<BVREngagePhase>(
            "BVR engagement (15NM, 250kts)", 30.0, alt, speed, 250.0, 15.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiBVR("digi_bvr", []() {
    return std::make_unique<DigiBVRScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_bvr() {}

} // namespace f4flight_test
