// f4flight - scenarios/scenario_digi_sensors.cpp
//
// Maneuver test for digi AI autonomous sensor detection (Phase 3).
//
// This is an END-TO-END integration test that verifies the AI can autonomously
// detect threats and targets via the sensor system, without injected pointers.
//
// The test:
//   1. Sets up a real aircraft in level flight
//   2. Provides a TruthState with a threat/target each frame
//   3. The brain runs SensorFusion → builds SensorPicture → detects threats
//   4. Verifies the AI enters the correct mode based on autonomous detection

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
// Phase: Autonomous Missile Detection
//
// Provides a TruthState with a radar missile 5 NM ahead, closing. The brain
// should autonomously detect it via RWR and enter MissileDefeat mode.
// ===========================================================================
class AutonomousMissilePhase : public ManeuverTest {
public:
    AutonomousMissilePhase(const char* name, double duration,
                           double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Set up the missile in truth state
        missile_.x = 5.0 * 6076.0;  // 5 NM ahead (+x)
        missile_.y = 0.0;
        missile_.z = -alt_;
        missile_.yaw = PI;  // heading west (toward us)
        missile_.speed = 2000.0;
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        truth_.clear();
        truth_.add(200, missile_);

        // Provide truth via the new FrameInputs API.
        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.truth = &truth_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the missile toward the aircraft
        missile_.x -= 2000.0 * dt;  // moving west (toward us)
        if (missile_.x < 1000.0) missile_.x = 5.0 * 6076.0;  // reset

        // Update truth
        truth_.clear();
        truth_.add(200, missile_);

        // Track state
        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::MissileDefeat) {
            enteredMissileDefeat_ = true;
            // Track G during MissileDefeat specifically. The test is supposed
            // to verify the AI RESPONDED to the detected missile, not just
            // detected it. A regression where the AI enters MissileDefeat
            // but doesn't maneuver would pass the old test (which only
            // checked mode entry + sensor detection).
            maxGInDefeat_ = std::max(maxGInDefeat_, as.loads.nzcgs);
        }
        // Verify the sensor pipeline actually saw the missile (not just
        // that the brain latched the mode). The brain's state_.missileDefeat.incomingMissile
        // should be non-null after sensor fusion runs.
        if (sc_brain_->state().missileDefeat.incomingMissile != nullptr) {
            sensorSawMissile_ = true;
        }
        if (sc_brain_->state().missileDefeat.incomingMissileId != kInvalidEntityId) {
            sensorSawMissile_ = true;
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(as.kin.sigma));
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Per-frame sample data (for trace)
        curMissileRange_ = std::sqrt(
            (missile_.x - as.kin.x) * (missile_.x - as.kin.x) +
            (missile_.y - as.kin.y) * (missile_.y - as.kin.y));
        curTtgo_ = sc_brain_->state().missileDefeat.missileDefeatTtgo;
        curMode_ = mode;
        curSensorSaw_ = sensorSawMissile_;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (autonomous missile detection)\n", testName_.c_str());
                std::printf("%6s %8s %6s %6s %8s %6s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "missileX", "mode", "sensor");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s %6s\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                missile_.x, modeBuf, sensorSawMissile_ ? "SEEN" : "-");
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered MissileDefeat mode.
        if (!enteredMissileDefeat_) return false;
        // 2. Must have autonomously detected the missile via the sensor
        //    pipeline. The old test only checked mode-entry, which could
        //    pass from a stale pointer. Verify the brain's
        //    state_.missileDefeat.incomingMissile was actually populated by SensorFusion
        //    (not just latched from a prior frame).
        if (!sensorSawMissile_) return false;
        // 3. Must have actually MANEUVERED in response to the missile. The
        //    old test only checked mode entry + sensor detection — a
        //    regression where the AI detects the missile but doesn't react
        //    would pass. Require maxG >= 1.2 during MissileDefeat (above
        //    level-flight G of ~1.0, proving the AI rolled/pulled in
        //    response to the detected threat).
        if (maxGInDefeat_ < 1.2) return false;
        // 4. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter MissileDefeat mode via autonomous sensor fusion; "
               "Sensor pipeline saw incomingMissile; "
               "Maneuvered (maxG >= 1.2 during MissileDefeat); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredMissileDefeat_) {
            return "Never entered MissileDefeat mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; missile closed to " + std::to_string(static_cast<int>(curMissileRange_)) +
                   "ft — sensor fusion did not classify the threat as a missile).";
        }
        if (!sensorSawMissile_) {
            return "Entered MissileDefeat mode but sensor pipeline never populated "
                   "state_.missileDefeat.incomingMissile (mode entry may have been "
                   "a stale latch from a prior frame).";
        }
        if (maxGInDefeat_ < 1.2) {
            return "Max G during MissileDefeat was " + std::to_string(maxGInDefeat_) +
                   " (needed >= 1.2) — AI detected the missile via sensors but "
                   "did not maneuver in response (no roll/pull command issued).";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — defensive maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"msl_range",  curMissileRange_, "ft"},
            {"msl_ttgo",   curTtgo_,         "s"},
            {"in_defeat",  (enteredMissileDefeat_ && curMode_ == DigiMode::MissileDefeat) ? 1.0 : 0.0, ""},
            {"sensor_saw", curSensorSaw_ ? 1.0 : 0.0, ""},
        };
    }

    // The missile is auto-extracted by the framework via
    // state_.missileDefeat.incomingMissile (populated by SensorFusion from
    // the truth_ state). No need to publish here.

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Autonomous MissileDefeat: %s\n",
            enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Sensor saw missile:       %s\n",
            sensorSawMissile_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max G during MissileDefeat: %.2f (need >= 1.2) %s\n",
            maxGInDefeat_, maxGInDefeat_ >= 1.2 ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                    %.2f\n", maxG_);
        std::printf("  Max heading change:       %.1f deg\n", maxHeadingChange_ * RTD);
        std::printf("  Min altitude:             %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    TruthState truth_;
    double nextPrint_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxGInDefeat_{0.0};
    double maxHeadingChange_{0.0};
    bool hasNaN_{false};
    bool enteredMissileDefeat_{false};
    bool sensorSawMissile_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data (updated in Evaluate, read in traceSamples)
    double curMissileRange_{0.0};
    double curTtgo_{-1.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curSensorSaw_{false};
};

// ===========================================================================
// Phase: Autonomous Target Detection
//
// Provides a TruthState with a fighter 3 NM ahead. The brain should
// autonomously detect it via radar/visual and enter WVREngage mode.
// ===========================================================================
class AutonomousTargetPhase : public ManeuverTest {
public:
    AutonomousTargetPhase(const char* name, double duration,
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

        // Set up the target in truth state
        target_.x = 3.0 * 6076.0;  // 3 NM ahead
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = PI;  // heading west (toward us)
        target_.speed = 500.0;

        truth_.clear();
        truth_.add(100, target_);

        // Provide truth via the new FrameInputs API.
        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.truth = &truth_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Keep target roughly ahead (move slowly)
        target_.x += 400.0 * dt;  // target moves east (away)

        // Update truth
        truth_.clear();
        truth_.add(100, target_);

        // Track state
        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::WVREngage) {
            enteredWVREngage_ = true;
        }
        // Verify the sensor pipeline saw the target.
        if (sc_brain_->sensorPicture().bestTarget != nullptr) {
            sensorSawTarget_ = true;
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Per-frame sample data (for trace)
        curTargetRange_ = std::sqrt(
            (target_.x - as.kin.x) * (target_.x - as.kin.x) +
            (target_.y - as.kin.y) * (target_.y - as.kin.y));
        curMode_ = mode;
        curSensorSaw_ = sensorSawTarget_;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (autonomous target detection)\n", testName_.c_str());
                std::printf("%6s %8s %6s %6s %8s %6s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "targetX", "mode", "sensor");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s %6s\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                target_.x, modeBuf, sensorSawTarget_ ? "SEEN" : "-");
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered WVREngage mode.
        if (!enteredWVREngage_) return false;
        // 2. Must have autonomously detected the target via the sensor
        //    pipeline (radar/visual). Verify sensorPicture.bestTarget was
        //    populated — the old test only checked mode-entry.
        if (!sensorSawTarget_) return false;
        // 3. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter WVREngage mode via autonomous sensor fusion; "
               "Sensor pipeline saw bestTarget; Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredWVREngage_) {
            return "Never entered WVREngage mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target was at " + std::to_string(static_cast<int>(curTargetRange_)) +
                   "ft — sensor fusion did not classify it as a WVR threat).";
        }
        if (!sensorSawTarget_) {
            return "Entered WVREngage mode but sensor pipeline never populated "
                   "sensorPicture.bestTarget (mode entry may have been a stale "
                   "latch from a prior frame).";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — engagement maneuver pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"tgt_range",  curTargetRange_, "ft"},
            {"in_wvr",     (enteredWVREngage_ && curMode_ == DigiMode::WVREngage) ? 1.0 : 0.0, ""},
            {"sensor_saw", curSensorSaw_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget()
    // (populated by SensorFusion from the truth_ state). No need to publish.

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Autonomous WVREngage: %s\n",
            enteredWVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Sensor saw target:    %s\n",
            sensorSawTarget_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:         %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity target_;
    TruthState truth_;
    double nextPrint_{0.0};
    double minAlt_{1e9};
    bool hasNaN_{false};
    bool enteredWVREngage_{false};
    bool sensorSawTarget_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data (updated in Evaluate, read in traceSamples)
    double curTargetRange_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curSensorSaw_{false};
};

// ===========================================================================
// DigiSensorScenario
// ===========================================================================
class DigiSensorScenario : public ManeuverScenario {
public:
    DigiSensorScenario() : ManeuverScenario("digi_sensors") {}

    std::string GetDescription() const override {
        return "Digi AI autonomous sensor detection: missile (RWR) and target "
               "(radar/visual). No injected threats — AI detects via SensorFusion.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<AutonomousMissilePhase>(
            "Autonomous missile detection", 15.0, alt, speed));
        tests.push_back(std::make_unique<AutonomousTargetPhase>(
            "Autonomous target detection", 15.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiSensors("digi_sensors", []() {
    return std::make_unique<DigiSensorScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_sensors() {}

} // namespace f4flight_test
