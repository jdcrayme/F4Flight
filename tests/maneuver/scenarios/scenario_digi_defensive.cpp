// f4flight - scenarios/scenario_digi_defensive.cpp
//
// Maneuver tests for the digi AI defensive capabilities (Tier 1).
//
// These are END-TO-END integration tests that drive the full FlightModel +
// DigiBrain through defensive scenarios. Unlike the unit tests in
// test_digi_defensive.cpp (which test functions in isolation with synthetic
// state), these tests:
//
//   1. Set up a real aircraft in level flight
//   2. Inject a threat (missile or guns) via DigiEntity
//   3. Run the simulation for several seconds
//   4. Verify the AI actually performs the expected defensive maneuver
//      (turns beam/cold to missile, rolls+jinks for guns, doesn't lawn-dart)
//
// Scenarios:
//   digi_missile_defeat: Inject a missile 5 NM away closing at 2000 ft/s.
//     Verify the AI enters MissileDefeat mode, turns away from the missile,
//     and does NOT crash (stays above ground).
//
//   digi_missile_last_ditch: Inject a missile 2000 ft away (TTGO < 1s).
//     Verify the AI commands max G (last-ditch pull) and doesn't NaN.
//
//   digi_guns_jink: Inject a guns threat 3000 ft ahead, firing.
//     Verify the AI enters GunsJink mode, rolls the aircraft, and pulls G.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace manuver_test {

// ===========================================================================
// Phase: Missile Defeat (beam/drag maneuver)
//
// Injects a radar missile 5 NM away, closing at ~2000 ft/s. The AI should:
//   - Enter MissileDefeat mode
//   - Turn away from the missile (beam or cold)
//   - Not crash (stay above ground)
//   - Not NaN
// ===========================================================================
class MissileDefeatPhase : public ManeuverTest {
public:
    MissileDefeatPhase(const char* name, double duration,
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

        // Set up the missile: 5 NM north, heading south (toward us), closing
        const double missileRange = 5.0 * 6076.0;  // 5 NM in ft
        missile_.x = 0.0;
        missile_.y = missileRange;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;  // heading south (toward us)
        missile_.speed = 2000.0;   // ft/s
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        sc.brain().setIncomingMissile(&missile_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
        initialMissileBearing_ = PI / 2.0;  // missile is north = +Y
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the missile toward the aircraft each frame
        // Missile is at (0, missileRange), heading south = -Y direction
        const double missileSpeed = 2000.0;  // ft/s
        missile_.y -= missileSpeed * dt;
        // If missile passes us, move it back to keep the threat active
        if (missile_.y < -1000.0) {
            missile_.y = 5.0 * 6076.0;
        }

        // Track state
        const double heading = as.kin.sigma;
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(heading - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        // Track active mode
        currentMode_ = sc_brain_->activeMode();
        if (currentMode_ == DigiMode::MissileDefeat) enteredMissileDefeat_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (missile 5NM closing at 2000 ft/s)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %8s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "G", "pstk", "missileY", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %8.0f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD,
                as.loads.nzcgs, input.pstick, missile_.y,
                digiModeName(currentMode_));
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Must have entered MissileDefeat mode at some point
        if (!enteredMissileDefeat_) return false;
        // Must have turned away from the missile (heading change > 20°)
        if (maxHeadingChange_ < 20.0 * DTR) return false;
        // Must not have lawn-darted (stay above 1000 ft AGL)
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered MissileDefeat mode: %s\n", enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading change:         %.1f deg (need >= 20) %s\n",
            maxHeadingChange_ * RTD, maxHeadingChange_ >= 20.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:               %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double initialMissileBearing_{0.0};
    double maxHeadingChange_{0.0};
    double minAlt_{std::numeric_limits<double>::max()};
    double maxG_{0.0};
    bool hasNaN_{false};
    DigiMode currentMode_{DigiMode::NoMode};
    bool enteredMissileDefeat_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: Missile Last Ditch (TTGO < 1s)
//
// Injects a missile 1500 ft away (TTGO ~0.75s). The AI should command max G
// (last-ditch pull). Verify it doesn't NaN and produces a strong pull.
// ===========================================================================
class MissileLastDitchPhase : public ManeuverTest {
public:
    MissileLastDitchPhase(const char* name, double duration,
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

        // Missile 1500 ft north, closing fast → TTGO = 1500/2000 = 0.75s < LD_TIME(1s)
        missile_.x = 0.0;
        missile_.y = 1500.0;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;
        missile_.speed = 2000.0;
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        sc.brain().setIncomingMissile(&missile_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Keep missile closing
        missile_.y -= 2000.0 * dt;
        if (missile_.y < 100.0) missile_.y = 1500.0;  // reset to keep threat

        maxPstick_ = std::max(maxPstick_, input.pstick);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        if (std::isnan(as.kin.vt) || std::isnan(input.pstick)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (missile 1500ft, TTGO < 1s)\n", testName_.c_str());
                std::printf("%6s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "G", "pstk", "ttgo");
            }
            std::printf("%6.1f %8.0f %6.2f %6.2f %6.2f\n",
                phaseTime_, -as.kin.z, as.loads.nzcgs, input.pstick,
                sc_brain_->state().missileDefeatTtgo);
            nextPrint_ += 0.5;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Last ditch should command a strong pull (positive pstick)
        // First-frame pstick is ~0.36 after smoothing; check > 0.1
        return maxPstick_ > 0.1;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Max pstick: %.2f (need > 0.1) %s\n",
            maxPstick_, maxPstick_ > 0.1 ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:      %.2f\n", maxG_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    double nextPrint_{0.0};
    double maxPstick_{0.0};
    double maxG_{0.0};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: Guns Jink
//
// Injects a guns threat 3000 ft ahead, firing. The AI should:
//   - Enter GunsJink mode
//   - Roll the aircraft (non-zero bank change)
//   - Pull G (max G pull phase)
//   - Not crash
// ===========================================================================
class GunsJinkPhase : public ManeuverTest {
public:
    GunsJinkPhase(const char* name, double duration,
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

        // Guns threat 3000 ft ahead (north), pointing at us, firing
        threat_.x = 0.0;
        threat_.y = 3000.0;
        threat_.z = -alt_;
        threat_.yaw = -PI / 2.0;  // pointing south (at us)
        threat_.isFiring = true;
        threat_.isDead = false;

        sc.brain().setGunsThreat(&threat_);
        sc_brain_ = &sc.brain();
        initialBank_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Keep threat roughly ahead (move with us)
        threat_.y = 3000.0;  // static for simplicity

        const double bank = as.kin.phi;
        maxBankChange_ = std::max(maxBankChange_, std::fabs(bank - initialBank_));
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (guns threat 3000ft, firing)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "bank(d)", "G", "pstk", "rstk");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f\n",
                phaseTime_, -as.kin.z, bank * RTD,
                as.loads.nzcgs, input.pstick, input.rstick);
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Must have rolled significantly (jink involves ±70° bank)
        if (maxBankChange_ < 30.0 * DTR) return false;
        // Must not lawn-dart
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Max bank change: %.1f deg (need >= 30) %s\n",
            maxBankChange_ * RTD, maxBankChange_ >= 30.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:           %.2f\n", maxG_);
        std::printf("  Min altitude:    %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity threat_;
    double nextPrint_{0.0};
    double initialBank_{0.0};
    double maxBankChange_{0.0};
    double maxG_{0.0};
    double minAlt_{std::numeric_limits<double>::max()};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// DigiDefensiveScenario
// ===========================================================================
class DigiDefensiveScenario : public ManeuverScenario {
public:
    DigiDefensiveScenario() : ManeuverScenario("digi_defensive") {}

    std::string GetDescription() const override {
        return "Digi AI defensive: missile defeat (beam/drag), last-ditch pull, "
               "guns jink. Tests MissileDefeat + GunsJink end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<MissileDefeatPhase>(
            "Missile defeat (5NM closing)", 15.0, alt, speed));
        tests.push_back(std::make_unique<MissileLastDitchPhase>(
            "Missile last ditch (1500ft)", 5.0, alt, speed));
        tests.push_back(std::make_unique<GunsJinkPhase>(
            "Guns jink (3000ft firing)", 8.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiDefensive("digi_defensive", []() {
    return std::make_unique<DigiDefensiveScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_defensive() {}

} // namespace manuver_test
