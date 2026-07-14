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

        sc.brain().setTruth(&truth_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us (head-on closure)
        target_.x += target_.vx * dt;
        truth_.clear();
        truth_.add(100, target_);

        // Track state
        if (sc_brain_->activeMode() == DigiMode::GunsEngage) {
            enteredGunsEngage_ = true;
        }
        if (input.fireGun) {
            firedGun_ = true;
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
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            const double range = std::fabs(target_.x - as.kin.x);
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s %6s %6.0f\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                target_.x, modeBuf, input.fireGun ? "FIRE" : "",
                range);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || firedGun_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Must have entered GunsEngage mode
        if (!enteredGunsEngage_) return false;
        // Must not have crashed
        if (minAlt_ < 100.0) return false;
        // Heavy aircraft (B-52, C-130, etc.) can't pull enough G to track
        // a gun target — accept "entered GunsEngage + didn't crash" for them.
        if (isHeavy_) return true;
        // Fighter/attack: must have fired the gun
        if (!firedGun_) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GunsEngage: %s\n", enteredGunsEngage_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Fired gun:          %s (heavy: waived)\n",
                firedGun_ ? "[PASS]" : "[WAIVED]");
        } else {
            std::printf("  Fired gun:          %s\n", firedGun_ ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Min altitude:       %.0f ft (need >= 100) %s\n",
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
    bool firedGun_{false};
    bool isHeavy_{false};
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
