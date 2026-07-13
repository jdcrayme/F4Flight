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

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace manuver_test {

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

        sc.brain().setTruth(&truth_);
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
        if (sc_brain_->activeMode() == DigiMode::MissileDefeat) {
            enteredMissileDefeat_ = true;
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (autonomous missile detection)\n", testName_.c_str());
                std::printf("%6s %8s %6s %6s %8s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "missileX", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                missile_.x, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        return enteredMissileDefeat_;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Autonomous MissileDefeat: %s\n",
            enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude: %.0f ft\n", minAlt_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    TruthState truth_;
    double nextPrint_{0.0};
    double minAlt_{1e9};
    bool hasNaN_{false};
    bool enteredMissileDefeat_{false};
    const DigiBrain* sc_brain_{nullptr};
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

        sc.brain().setTruth(&truth_);
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
        if (sc_brain_->activeMode() == DigiMode::WVREngage) {
            enteredWVREngage_ = true;
        }

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (autonomous target detection)\n", testName_.c_str());
                std::printf("%6s %8s %6s %6s %8s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "targetX", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %6.2f %6.2f %8.0f %6s\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                target_.x, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        return enteredWVREngage_;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Autonomous WVREngage: %s\n",
            enteredWVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude: %.0f ft\n", minAlt_);
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
    const DigiBrain* sc_brain_{nullptr};
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

} // namespace manuver_test
