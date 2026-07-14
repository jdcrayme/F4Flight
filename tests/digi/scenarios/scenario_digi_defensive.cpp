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

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

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
        maxGs_ = fm.config().geometry.maxGs;
        isHeavy_ = isHeavy(fm.config());

        // Set up the missile: 5 NM north, heading south (toward us), closing
        const double missileRange = 5.0 * 6076.0;  // 5 NM in ft
        missile_.x = 0.0;
        missile_.y = missileRange;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;  // heading south (toward us)
        missile_.speed = 2000.0;   // ft/s
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        // Inject the missile via the new FrameInputs API (production path).
        // The brain commits the threat to state_ on the next compute() call.
        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.injectedMissile = &missile_;
        sc.brain().setFrameInputs(fi);
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

        // "Turned away" check: the missile is north (+Y). The Drag maneuver
        // commands heading = missile heading = -PI/2 (south). Track the
        // minimum angular distance from the aircraft's heading to south.
        // wrapToSignedPi normalizes heading differences to [-PI, PI].
        double dh = heading - (-PI / 2.0);  // heading minus south
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToSouth_ = std::min(minAbsHeadingToSouth_, std::fabs(dh));

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
        // 1. Must have entered MissileDefeat mode at some point.
        if (!enteredMissileDefeat_) return false;
        // 2. Must have produced non-trivial G — at 45° bank, level-flight
        //    turn G = 1/cos(45°) = 1.41. Require > 1.2 so the test only
        //    passes when the AI actually rolled into the maneuver (not when
        //    it sat wings-level doing nothing).
        if (maxG_ < 1.2) return false;
        // 3. Must have turned toward south (away from north missile).
        //    The Drag maneuver commands heading = missile heading = -π/2.
        //    With maxBank=45° at corner speed, turn rate is ~3.3°/s, so in
        //    15 s the aircraft can rotate ~50°. Requiring it to get within
        //    60° of south is achievable and proves the turn direction is
        //    correct (toward south, not toward north where the missile is).
        if (minAbsHeadingToSouth_ > 60.0 * DTR) return false;
        // 4. Must not have lawn-darted. Test starts at 15000 ft, so 5000 ft
        //    is a generous floor (33% of start altitude).
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered MissileDefeat mode: %s\n", enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max G (need >= 1.2):        %.2f %s\n",
            maxG_, maxG_ >= 1.2 ? "[PASS]" : "[FAIL]");
        std::printf("  Closest heading to south:   %.1f deg (need <= 60) %s\n",
            minAbsHeadingToSouth_ * RTD, minAbsHeadingToSouth_ <= 60.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading change:         %.1f deg (info)\n", maxHeadingChange_ * RTD);
        std::printf("  Min altitude:               %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double initialMissileBearing_{0.0};  // retained for diagnostic printing
    double maxHeadingChange_{0.0};
    double minAbsHeadingToSouth_{std::numeric_limits<double>::max()};
    double minAlt_{std::numeric_limits<double>::max()};
    double maxG_{0.0};
    double maxGs_{9.0};
    bool   isHeavy_{false};
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
        maxGs_ = fm.config().geometry.maxGs;
        isHeavy_ = isHeavy(fm.config());

        // Missile 1500 ft north, closing fast → TTGO = 1500/2000 = 0.75s < LD_TIME(1s)
        missile_.x = 0.0;
        missile_.y = 1500.0;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;
        missile_.speed = 2000.0;
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.injectedMissile = &missile_;
        sc.brain().setFrameInputs(fi);
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
        if (sc_brain_->activeMode() == DigiMode::MissileDefeat) enteredMissileDefeat_ = true;

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
        // 1. Must have entered MissileDefeat mode (the last-ditch sub-mode
        //    runs inside MissileDefeat, so this proves the threat was seen).
        if (!enteredMissileDefeat_) return false;
        // 2. Must have commanded a strong pitch input — the last-ditch
        //    primitive calls SetPstick(maxGs, GCommand), which is the
        //    largest pitch command the FCS accepts. A pstick < 0.3 means
        //    either the FCS smoothed it away or the maneuver never ran.
        if (maxPstick_ < 0.3) return false;
        // 3. Must have produced significant G. The last-ditch primitive
        //    commands maxGs, but gsAvail at 15000 ft / corner speed caps
        //    the achievable G. For fighters (maxGs=7-9), gsAvail ~4-7G and
        //    we require >= 40% of maxGs (catches a regression where the
        //    last-ditch pull is wired to the wrong FCS output — the
        //    original `> 0.1` test passed even when the pull produced 0.2G).
        //    Heavies (maxGs ~2.3) have gsAvail ~0.8-1.5G at this condition;
        //    require >= 30% of maxGs (0.69G for B-52) — still catches a
        //    zero-pull regression but accepts the airframe's physical limit.
        const double gFraction = isHeavy_ ? 0.30 : 0.40;
        if (maxG_ < gFraction * maxGs_) return false;
        return true;
    }

    void Finish() const override {
        const double gFraction = isHeavy_ ? 0.30 : 0.40;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered MissileDefeat: %s\n", enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max pstick: %.2f (need >= 0.3) %s\n",
            maxPstick_, maxPstick_ >= 0.3 ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:      %.2f (need >= %.2f = %.0f%%*maxGs%s) %s\n",
            maxG_, gFraction * maxGs_, gFraction * 100.0, isHeavy_ ? " [HEAVY]" : "",
            maxG_ >= gFraction * maxGs_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    double nextPrint_{0.0};
    double maxPstick_{0.0};
    double maxG_{0.0};
    double maxGs_{9.0};
    bool   isHeavy_{false};
    bool hasNaN_{false};
    bool enteredMissileDefeat_{false};
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
        maxGs_ = fm.config().geometry.maxGs;
        isHeavy_ = isHeavy(fm.config());

        // Guns threat 3000 ft ahead (north), pointing at us, firing
        threat_.x = 0.0;
        threat_.y = 3000.0;
        threat_.z = -alt_;
        threat_.yaw = -PI / 2.0;  // pointing south (at us)
        threat_.isFiring = true;
        threat_.isDead = false;

        f4flight::digi::FrameInputs fi = sc.brain().frameInputs();
        fi.injectedGunsThreat = &threat_;
        sc.brain().setFrameInputs(fi);
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
        if (sc_brain_->activeMode() == DigiMode::GunsJink) enteredGunsJink_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (guns threat 3000ft, firing)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "bank(d)", "G", "pstk", "rstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, bank * RTD,
                as.loads.nzcgs, input.pstick, input.rstick, modeBuf);
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered GunsJink mode at some point. (The old test
        //    never checked this — the header claim "Enter GunsJink mode"
        //    was silently dropped.)
        if (!enteredGunsJink_) return false;
        // 2. Must have rolled significantly. The jink primitive sets
        //    fcsState.maxRoll = 190° and commands ±70° (kJinkRollAngle),
        //    but the bank is clamped to digi.maxRoll (45° in this test).
        //    Requiring > 25° ensures the AI actually rolled into the jink
        //    (not just drifted). 25° is conservative; in practice the jink
        //    reaches the 45° clamp within ~1 s.
        if (maxBankChange_ < 25.0 * DTR) return false;
        // 3. Must have pulled G. The jink primitive calls SetPstick(maxGs)
        //    during the pull phase. For fighters at 15000 ft, gsAvail caps
        //    the achievable G around 4-7G; require > 2.0 so a regression
        //    where the jink pull is wired wrong (e.g. SetPstick to the wrong
        //    FCS output) shows up as a 1.0G level-flight result. Heavies
        //    (B-52, C-130) have maxGs ~2.3 but their gsAvail at 15000 ft /
        //    corner speed is only ~1.1G — they physically cannot pull 2G
        //    here. For heavies, require > 1.0 (anything above level flight
        //    proves the pull command was issued; the airframe just can't
        //    deliver more G).
        const double gThreshold = isHeavy_ ? 1.05 : 2.0;
        if (maxG_ < gThreshold) return false;
        // 4. Must not lawn-dart. Test starts at 15000 ft; 5000 ft floor.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    void Finish() const override {
        const double gThreshold = isHeavy_ ? 1.05 : 2.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GunsJink:   %s\n", enteredGunsJink_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max bank change:    %.1f deg (need >= 25) %s\n",
            maxBankChange_ * RTD, maxBankChange_ >= 25.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:              %.2f (need >= %.2f%s) %s\n",
            maxG_, gThreshold, isHeavy_ ? " [HEAVY]" : "",
            maxG_ >= gThreshold ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:       %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity threat_;
    double nextPrint_{0.0};
    double initialBank_{0.0};
    double maxBankChange_{0.0};
    double maxG_{0.0};
    double maxGs_{9.0};
    double minAlt_{std::numeric_limits<double>::max()};
    bool   isHeavy_{false};
    bool hasNaN_{false};
    bool enteredGunsJink_{false};
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

} // namespace f4flight_test
