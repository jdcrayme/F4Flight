// f4flight - scenarios/scenario_digi_guns.cpp
//
// Maneuver test for digi AI offensive guns engagement.
//
// Sets up a head-on gun engagement: the AI aircraft starts 3 NM from a
// target flying toward it. The AI should:
//   1. Detect the target via SensorFusion
//   2. Enter WVREngage (target within 8 NM)
//   3. Transition to GunsEngage (target within 3500 ft, ata < 35°)
//   4. Track the target with lead (CoarseGunsTrack → GunsAutoTrack)
//   5. Fire the gun when the pipper is on target (FineGunsTrack)
//
// The test verifies:
//   - The AI enters GunsEngage mode at some point
//   - The AI fires the gun at some point (PilotInput.fireGun == true)
//   - The aircraft doesn't NaN or crash

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace manuver_test {

// ===========================================================================
// Phase: Head-on gun engagement
// ===========================================================================
class GunsEngagePhase : public ManeuverTest {
public:
    GunsEngagePhase(const char* name, double duration,
                    double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // Heavy aircraft (low T/W, low G) can't track a gun target well.
        // Accept "entered GunsEngage + didn't crash" for heavies.
        isHeavy_ = isHeavy(fm.config());

        // Target 3 NM ahead, heading toward us (head-on)
        target_.x = 3.0 * 6076.0;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = PI;  // heading west (toward us)
        target_.speed = 400.0;
        target_.vx = -400.0;
        target_.vy = 0.0;
        target_.vz = 0.0;

        truth_.clear();
        truth_.add(100, target_);

        // Provide truth state via the new FrameInputs API.
        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.truth = &truth_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us (head-on closure)
        target_.x += target_.vx * dt;
        truth_.clear();
        truth_.add(100, target_);

        // Track state
        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::GunsEngage) {
            enteredGunsEngage_ = true;
            // Track fire ONLY while in GunsEngage mode. The old test set
            // firedGun_ from input.fireGun in ANY mode — a transient fire
            // flag from WVREngage (or a stale flag from a prior phase)
            // would pass the test.
            if (input.fireGun) {
                firedGunInGunsEngage_ = true;
                fireFrames_++;
            }
        } else if (input.fireGun) {
            firedGunOutsideGunsEngage_ = true;  // diagnostic only
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (head-on gun engagement)\n", testName_.c_str());
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
        // Do NOT exit early on first fire — the old test ended the phase
        // the instant firedGun_ became true, which meant it never verified
        // SUSTAINED fire or fire-while-in-GunsEngage. Run the full duration.
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered GunsEngage mode.
        if (!enteredGunsEngage_) return false;
        // 2. Must not have crashed.
        if (minAlt_ < 100.0) return false;
        // 3. Heavy aircraft (B-52, C-130, etc.) can't pull enough G to track
        //    a gun target — accept "entered GunsEngage + didn't crash".
        if (isHeavy_) return true;
        // 4. Fighter/attack: must have fired the gun WHILE IN GunsEngage
        //    mode. The old test accepted fire in any mode (including
        //    transient fire flags from WVREngage before transitioning to
        //    GunsEngage).
        if (!firedGunInGunsEngage_) return false;
        // 5. Must have fired for at least 6 frames (0.1 s at 60 Hz) —
        //    proves the fire was sustained, not a single-frame glitch.
        if (fireFrames_ < 6) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GunsEngage:        %s\n", enteredGunsEngage_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Fired gun (in GunsEngage): %s (heavy: waived)\n",
                firedGunInGunsEngage_ ? "[PASS]" : "[WAIVED]");
        } else {
            std::printf("  Fired gun (in GunsEngage): %s (%d frames, need >= 6) %s\n",
                firedGunInGunsEngage_ ? "yes" : "no", fireFrames_,
                (firedGunInGunsEngage_ && fireFrames_ >= 6) ? "[PASS]" : "[FAIL]");
        }
        if (firedGunOutsideGunsEngage_) {
            std::printf("  (note: fire flag seen outside GunsEngage mode)\n");
        }
        std::printf("  Min altitude:              %.0f ft (need >= 100) %s\n",
            minAlt_, minAlt_ >= 100.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity target_;
    TruthState truth_;
    double nextPrint_{0.0};
    double minAlt_{1e9};
    bool hasNaN_{false};
    bool enteredGunsEngage_{false};
    bool firedGunInGunsEngage_{false};
    bool firedGunOutsideGunsEngage_{false};
    bool isHeavy_{false};
    int  fireFrames_{0};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// DigiGunsScenario
// ===========================================================================
class DigiGunsScenario : public ManeuverScenario {
public:
    DigiGunsScenario() : ManeuverScenario("digi_guns") {}

    std::string GetDescription() const override {
        return "Digi AI offensive guns engagement: head-on pass, AI should "
               "track with lead and fire the gun when the pipper is on target.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<GunsEngagePhase>(
            "Head-on gun engagement", 30.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiGuns("digi_guns", []() {
    return std::make_unique<DigiGunsScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_guns() {}

} // namespace manuver_test
