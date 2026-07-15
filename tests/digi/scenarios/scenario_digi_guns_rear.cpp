// f4flight - scenarios/scenario_digi_guns_rear.cpp
//
// Maneuver test for digi AI rear-aspect (stern conversion) gun engagement.
//
// DIFFERENT from the existing digi_guns (head-on): this tests the classic
// stern-chase case where the AI starts BEHIND a slower target on the same
// heading, closes to gun range, and fires.
//
// Sets up: AI aircraft 4000 ft behind a target flying the same heading at
// 250 kts. AI at corner speed (350 kts) closes at ~100 kts = ~168 ft/s.
// Range drops below 3500 ft (gun entry range) in ~3 seconds, then the AI
// should:
//   1. Enter GunsEngage mode (range <= 3500 ft, ata < 35° * 1.25 = 43.75°).
//   2. Track the target with lead (CoarseGunsTrack → GunsAutoTrack).
//   3. Fire the gun when the pipper is on target (FineGunsTrack).
//   4. Not crash or NaN.
//
// The target is moving AWAY (same heading, slower) — the AI catches up from
// behind. This is the textbook "stern conversion" — different geometry than
// the head-on case in digi_guns.
//
// Pass criteria:
//   - Enters GunsEngage mode at some point
//   - Fires the gun (sustained >= 4 frames) — waived for heavy aircraft
//   - No NaN, no crash

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// Phase: Rear-aspect (stern conversion) gun engagement
// ===========================================================================
class RearGunsEngagePhase : public ManeuverTest {
public:
    RearGunsEngagePhase(const char* name, double duration,
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

        // Target ahead (+x), same heading (east, yaw=0), slower. Classic
        // stern chase — AI catches up from behind.
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

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target forward at its speed.
        target_.x += target_.speed * dt;

        // Track state
        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::GunsEngage) {
            enteredGunsEngage_ = true;
            if (input.fireGun) {
                firedGunInGunsEngage_ = true;
                fireFrames_++;
            }
        } else if (input.fireGun) {
            firedGunOutsideGunsEngage_ = true;  // diagnostic only
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Per-frame sample data
        curRange_ = std::fabs(target_.x - as.kin.x);
        // Target aspect: angle between target's velocity and LOS from target
        // to aircraft. 0 = head-on, 180 = tail-chase (stern conversion).
        const double losX = as.kin.x - target_.x;
        const double losY = as.kin.y - target_.y;
        const double losMag = std::sqrt(losX * losX + losY * losY);
        if (losMag > 1.0 && target_.speed > 1.0) {
            const double tvx = target_.vx;
            const double tvy = target_.vy;
            const double dot = (tvx * losX + tvy * losY) /
                               (target_.speed * losMag);
            curTargetAspect_ = std::acos(
                dot < -1.0 ? -1.0 : (dot > 1.0 ? 1.0 : dot)) * RTD;
        }
        curMode_ = mode;
        curFire_ = input.fireGun;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.0fft ahead, same heading, %.0f kts)\n",
                    testName_.c_str(), initialRangeFt_, targetSpeed_);
                std::printf("%6s %8s %6s %6s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "tgtX", "mode", "fire", "range");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            const double range = std::fabs(target_.x - as.kin.x);
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s %6s %6.0f\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                target_.x, modeBuf, input.fireGun ? "FIRE" : "",
                range);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered GunsEngage mode.
        if (!enteredGunsEngage_) return false;
        // 2. Must not have crashed.
        if (minAlt_ < 100.0) return false;
        // 3. Heavy aircraft can't pull enough G to track a gun target —
        //    accept "entered GunsEngage + didn't crash".
        if (isHeavy_) return true;
        // 4. Fighter/attack: must have fired the gun WHILE IN GunsEngage.
        if (!firedGunInGunsEngage_) return false;
        // 5. Sustained fire for >= 4 frames (0.067s at 60Hz). Fast aircraft
        //    like the F-22 close through the gun zone so quickly they may
        //    only get 4-5 frames of fire; 4 still proves it wasn't a
        //    single-frame glitch.
        if (fireFrames_ < 4) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GunsEngage mode; Min alt >= 100ft; (Heavy: fire waived); "
               "Fighter: fireGun true in GunsEngage for >= 4 frames; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredGunsEngage_) {
            return "Never entered GunsEngage mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target closed to " + std::to_string(static_cast<int>(curRange_)) +
                   "ft at aspect " + std::to_string(static_cast<int>(curTargetAspect_)) +
                   "deg — stern-chase closure did not meet gun entry criteria).";
        }
        if (minAlt_ < 100.0) {
            return "Aircraft descended to " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 100ft) — guns tracking pulled the aircraft too low.";
        }
        if (isHeavy_) return "";  // heavy: only mode+alt required, both passed
        if (!firedGunInGunsEngage_) {
            return "Entered GunsEngage but never fired the gun while in that mode "
                   "(final range " + std::to_string(static_cast<int>(curRange_)) +
                   "ft, target aspect " + std::to_string(static_cast<int>(curTargetAspect_)) +
                   "deg — pipper never settled on target).";
        }
        if (fireFrames_ < 4) {
            return "Fired the gun in GunsEngage for only " + std::to_string(fireFrames_) +
                   " frame(s) (needed >= 4 frames = 0.067s sustained fire) — "
                   "fire was not sustained.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",       curRange_,        "ft"},
            {"tgt_aspect",  curTargetAspect_, "deg"},
            {"in_guns",     (enteredGunsEngage_ && curMode_ == DigiMode::GunsEngage) ? 1.0 : 0.0, ""},
            {"fire",        curFire_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GunsEngage:        %s\n", enteredGunsEngage_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Fired gun (in GunsEngage): %s (heavy: waived)\n",
                firedGunInGunsEngage_ ? "[PASS]" : "[WAIVED]");
        } else {
            std::printf("  Fired gun (in GunsEngage): %s (%d frames, need >= 4) %s\n",
                firedGunInGunsEngage_ ? "yes" : "no", fireFrames_,
                (firedGunInGunsEngage_ && fireFrames_ >= 4) ? "[PASS]" : "[FAIL]");
        }
        if (firedGunOutsideGunsEngage_) {
            std::printf("  (note: fire flag seen outside GunsEngage mode)\n");
        }
        std::printf("  Target aspect (180=stern): %.0f deg\n", curTargetAspect_);
        std::printf("  Min altitude:              %.0f ft (need >= 100) %s\n",
            minAlt_, minAlt_ >= 100.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeFt_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double minAlt_{1e9};
    bool hasNaN_{false};
    bool enteredGunsEngage_{false};
    bool firedGunInGunsEngage_{false};
    bool firedGunOutsideGunsEngage_{false};
    bool isHeavy_{false};
    int  fireFrames_{0};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curTargetAspect_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curFire_{false};
};

// ===========================================================================
// DigiGunsRearScenario
// ===========================================================================
class DigiGunsRearScenario : public ManeuverScenario {
public:
    DigiGunsRearScenario() : ManeuverScenario("digi_guns_rear") {}

    std::string GetDescription() const override {
        return "Digi AI rear-aspect (stern conversion) gun engagement: AI starts "
               "behind a slower target on the same heading, closes to gun range, "
               "fires. Tests GunsEngage pursuit tracking (different geometry than "
               "the head-on digi_guns scenario).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Target at 4000 ft ahead, same heading, 250 kts. AI at corner speed
        // (350 kts) closes to gun range (~3500 ft) in ~3 s.
        tests.push_back(std::make_unique<RearGunsEngagePhase>(
            "Rear-aspect gun (4000ft chase)", 30.0, alt, speed, 250.0, 4000.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiGunsRear("digi_guns_rear", []() {
    return std::make_unique<DigiGunsRearScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_guns_rear() {}

} // namespace f4flight_test
