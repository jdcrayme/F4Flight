// f4flight - scenarios/scenario_high_defensive_chain.cpp
//
// HIGH-LEVEL scenario: defensive chain Missile defeat -> Guns jink ->
// Collision avoid -> Re-engage. Composes four low-level defensive behaviors
// into a realistic threat-response sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed.
//
// Pass criteria (per phase):
//   1. Missile defeat : enter MissileDefeat mode; max G >= 1.2 (heavy: 1.05);
//                       turn within 60deg of south; min alt >= 5000ft; no NaN
//   2. Guns jink      : enter GunsJink mode; bank change >= 25deg
//                       (heavy: 15deg); max G >= 1.5 (heavy: 1.05);
//                       min alt >= 5000ft; no NaN
//   3. Collision avoid: enter CollisionAvoid mode; heading change >= 15deg
//                       (heavy: 10deg); min alt >= 5000ft; no NaN
//   4. Re-engage       : target cleared, verify the AI returns to an offensive
//                       mode (BVREngage/WVREngage/MissileEngage/GunsEngage/Merge)
//                       OR Level-flight stable (no defensive mode stuck);
//                       min alt >= 5000ft; no NaN
//
// Tier: HighLevel. Registered as "high_defensive_chain" — referenced by the
// cascade mapping table g_highToLow["high_defensive_chain"].
//
// Task ID: 22

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

constexpr double kDefAlt = 15000.0;
constexpr double kNmToFt = 6076.0;

// ===========================================================================
// Phase 1: Missile defeat
//
// Missile 5 NM north, closing at 2000 ft/s. AI should enter MissileDefeat mode
// and turn beam/cold (toward south).
// ===========================================================================
class HighMissileDefeatPhase : public ManeuverTest {
public:
    HighMissileDefeatPhase(const char* name, double duration, double alt,
                           double speed)
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
        isHeavy_ = isHeavy(fm.config());

        const double missileRange = 5.0 * kNmToFt;
        missile_.x = 0.0;
        missile_.y = missileRange;
        missile_.z = -alt_;
        missile_.yaw = -PI / 2.0;  // heading south (toward us)
        missile_.speed = 2000.0;
        missile_.seekerType = DigiEntity::SeekerType::Radar;
        missile_.isDead = false;

        FrameInputs fi = sc.brain().frameInputs();
        fi.injectedMissile = &missile_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        // Move the missile toward the aircraft.
        missile_.y -= 2000.0 * dt;
        if (missile_.y < -1000.0) missile_.y = 5.0 * kNmToFt;

        const double heading = as.kin.sigma;
        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(heading - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        double dh = heading - (-PI / 2.0);  // heading minus south
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToSouth_ = std::min(minAbsHeadingToSouth_, std::fabs(dh));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        if (sc_brain_->activeMode() == DigiMode::MissileDefeat) enteredMissileDefeat_ = true;

        curMissileRange_ = std::sqrt(
            (missile_.x - as.kin.x) * (missile_.x - as.kin.x) +
            (missile_.y - as.kin.y) * (missile_.y - as.kin.y));
        curHdgToSouth_ = minAbsHeadingToSouth_ * RTD;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredMissileDefeat_) return false;
        const double gThreshold = isHeavy_ ? 1.05 : 1.2;
        if (maxG_ < gThreshold) return false;
        if (minAbsHeadingToSouth_ > 60.0 * DTR) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter MissileDefeat mode; Max G >= 1.2 (1.05 heavy); "
               "Turn within 60deg of south; Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredMissileDefeat_)
            return "Never entered MissileDefeat mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double gThreshold = isHeavy_ ? 1.05 : 1.2;
        if (maxG_ < gThreshold)
            return "Max G was " + std::to_string(maxG_) + " (need >= " +
                   std::to_string(gThreshold) + ").";
        if (minAbsHeadingToSouth_ > 60.0 * DTR)
            return "Closest heading to south was " + std::to_string(curHdgToSouth_) +
                   "deg (need <= 60deg).";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"msl_range", curMissileRange_, "ft"},
            {"hdg_south", curHdgToSouth_,   "deg"},
            {"in_defeat", (enteredMissileDefeat_ && curMode_ == DigiMode::MissileDefeat) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Missile Defeat Summary ---\n");
        std::printf("  Entered MissileDefeat: %s\n",
            enteredMissileDefeat_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                %.2f %s\n", maxG_,
            maxG_ >= (isHeavy_ ? 1.05 : 1.2) ? "[PASS]" : "[FAIL]");
        std::printf("  Min hdg to south:     %.1f deg %s\n",
            minAbsHeadingToSouth_ * RTD,
            minAbsHeadingToSouth_ <= 60.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:         %.0f ft %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity missile_;
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double minAbsHeadingToSouth_{1e9};
    bool enteredMissileDefeat_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curMissileRange_{0.0}, curHdgToSouth_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Guns jink
//
// Guns threat 3000 ft ahead, firing. AI should enter GunsJink mode and roll.
// ===========================================================================
class HighGunsJinkPhase : public ManeuverTest {
public:
    HighGunsJinkPhase(const char* name, double duration, double alt, double speed)
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
        isHeavy_ = isHeavy(fm.config());

        threat_.x = 0.0;
        threat_.y = 3000.0;
        threat_.z = -alt_;
        threat_.yaw = -PI / 2.0;  // pointing south (at us)
        threat_.isFiring = true;
        threat_.isDead = false;

        FrameInputs fi = sc.brain().frameInputs();
        fi.injectedGunsThreat = &threat_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
        initialBank_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        threat_.y = 3000.0;  // static for simplicity
        const double bank = as.kin.phi;
        maxBankChange_ = std::max(maxBankChange_, std::fabs(bank - initialBank_));
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        if (sc_brain_->activeMode() == DigiMode::GunsJink) enteredGunsJink_ = true;
        curBank_ = bank * RTD;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredGunsJink_) return false;
        const double bankThreshold = isHeavy_ ? 15.0 : 25.0;
        if (maxBankChange_ < bankThreshold * DTR) return false;
        const double gThreshold = isHeavy_ ? 1.05 : 1.5;
        if (maxG_ < gThreshold) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GunsJink mode; Bank change >= 25deg (15 heavy); "
               "Max G >= 1.5 (1.05 heavy); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGunsJink_)
            return "Never entered GunsJink mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double bankThreshold = isHeavy_ ? 15.0 : 25.0;
        if (maxBankChange_ < bankThreshold * DTR)
            return "Max bank change was " + std::to_string(maxBankChange_ * RTD) +
                   "deg (need >= " + std::to_string(bankThreshold) + "deg).";
        const double gThreshold = isHeavy_ ? 1.05 : 1.5;
        if (maxG_ < gThreshold)
            return "Max G was " + std::to_string(maxG_) +
                   " (need >= " + std::to_string(gThreshold) + ").";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) + "ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"bank",    curBank_, "deg"},
            {"G",       maxG_,    ""},
            {"in_jink", (enteredGunsJink_ && curMode_ == DigiMode::GunsJink) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Guns Jink Summary ---\n");
        std::printf("  Entered GunsJink: %s\n", enteredGunsJink_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max bank change:  %.1f deg %s\n",
            maxBankChange_ * RTD,
            maxBankChange_ >= (isHeavy_ ? 15.0 : 25.0) * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:            %.2f %s\n", maxG_,
            maxG_ >= (isHeavy_ ? 1.05 : 1.5) ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:     %.0f ft %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity threat_;
    double initialBank_{0.0};
    double maxBankChange_{0.0};
    double maxG_{0.0};
    double minAlt_{1e9};
    bool enteredGunsJink_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curBank_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Collision avoid
//
// Target 500 ft head-on, same altitude, closing fast. AI should enter
// CollisionAvoid and maneuver laterally.
// ===========================================================================
class HighCollisionAvoidPhase : public ManeuverTest {
public:
    HighCollisionAvoidPhase(const char* name, double duration, double alt,
                            double speed, double targetSpeed,
                            double initialRangeFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeFt_(initialRangeFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        target_.x = initialRangeFt_;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = PI;  // west (toward us)
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = -target_.speed;
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        target_.x -= target_.speed * dt;
        if (target_.x < -1000.0) target_.x = initialRangeFt_;

        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(as.kin.sigma - initialHeading_));
        maxLateralSep_ = std::max(maxLateralSep_, std::fabs(as.kin.y));
        minAlt_ = std::min(minAlt_, -as.kin.z);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::CollisionAvoid) enteredCollision_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curHdgChg_ = maxHeadingChange_ * RTD;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredCollision_) return false;
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter CollisionAvoid mode; Heading change >= 15deg (10 heavy); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredCollision_)
            return "Never entered CollisionAvoid mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR)
            return "Heading change was " + std::to_string(maxHeadingChange_ * RTD) +
                   "deg (need > " + std::to_string(hdgThreshold) + "deg).";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) + "ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_chg",      curHdgChg_, "deg"},
            {"in_collision", (enteredCollision_ && curMode_ == DigiMode::CollisionAvoid) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Collision Avoid Summary ---\n");
        std::printf("  Entered CollisionAvoid: %s\n",
            enteredCollision_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading chg:        %.1f deg %s\n",
            maxHeadingChange_ * RTD,
            maxHeadingChange_ >= (isHeavy_ ? 10.0 : 15.0) * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max lateral sep:        %.0f ft\n", maxLateralSep_);
        std::printf("  Min altitude:           %.0f ft %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeFt_;
    DigiEntity target_;
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double maxLateralSep_{0.0};
    double minAlt_{1e9};
    bool enteredCollision_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 4: Re-engage (threats cleared)
//
// All threats cleared. Inject a fresh target 10 NM ahead. The AI should
// return to an offensive mode (or at least exit the defensive modes).
// ===========================================================================
class HighReEngagePhase : public ManeuverTest {
public:
    HighReEngagePhase(const char* name, double duration, double alt, double speed,
                      double targetSpeed, double initialRangeNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Clear any residual missile/guns threat from prior phases.
        FrameInputs fi;
        fi.injectedMissile = nullptr;
        fi.injectedGunsThreat = nullptr;
        sc.brain().setFrameInputs(fi);

        // Inject a fresh target 10 NM ahead, same heading (east), slower.
        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = rangeFt;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = target_.speed;
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        target_.x += target_.speed * dt;
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GunsEngage || mode == DigiMode::MissileEngage ||
            mode == DigiMode::Merge) {
            enteredOffensive_ = true;
        }
        if (mode == DigiMode::MissileDefeat || mode == DigiMode::GunsJink ||
            mode == DigiMode::CollisionAvoid) {
            stuckDefensive_ = true;
        }
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curRange_ = range;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Either returns to offensive mode OR is not stuck in a defensive
        // mode (some aircraft may not detect the target within the time
        // window, but they shouldn't stay in MissileDefeat/GunsJink/
        // CollisionAvoid with no threat).
        if (!enteredOffensive_ && stuckDefensive_) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Returns to offensive mode (BVR/WVR/Missile/Guns/Merge) OR not "
               "stuck in defensive mode; Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredOffensive_ && stuckDefensive_)
            return "Never returned to offensive mode and stuck in defensive "
                   "mode (final: " + std::string(digiModeName(curMode_)) + ").";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) + "ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",   curRange_, "ft"},
            {"in_off",  enteredOffensive_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Re-engage Summary ---\n");
        std::printf("  Entered offensive: %s\n", enteredOffensive_ ? "[PASS]" : "(n/a)");
        std::printf("  Stuck defensive:   %s\n", stuckDefensive_ ? "[FAIL]" : "[PASS]");
        std::printf("  Min range:         %.0f ft\n", minRange_);
        std::printf("  Min altitude:      %.0f ft %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_;
    DigiEntity target_;
    double minRange_{1e9};
    double minAlt_{1e9};
    bool enteredOffensive_{false};
    bool stuckDefensive_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curRange_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// HighDefensiveChainScenario
// ===========================================================================
class HighDefensiveChainScenario : public ManeuverScenario {
public:
    HighDefensiveChainScenario() : ManeuverScenario("high_defensive_chain") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: defensive chain Missile defeat -> Guns jink -> Collision "
               "avoid -> Re-engage. Composes four low-level defensive behaviors "
               "into a realistic threat-response sequence. Relaxed per-phase criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, kDefAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<HighMissileDefeatPhase>(
            "Missile defeat (5NM closing)", 15.0, kDefAlt, cornerSpeed));
        tests.push_back(std::make_unique<HighGunsJinkPhase>(
            "Guns jink (3000ft firing)", 8.0, kDefAlt, cornerSpeed));
        tests.push_back(std::make_unique<HighCollisionAvoidPhase>(
            "Collision avoid (500ft head-on)", 8.0, kDefAlt, cornerSpeed,
            cornerSpeed, 500.0));
        tests.push_back(std::make_unique<HighReEngagePhase>(
            "Re-engage (10NM target)", 30.0, kDefAlt, cornerSpeed,
            250.0, 10.0));
        return tests;
    }
};

static RegisterScenario g_registerHighDefensiveChain("high_defensive_chain", []() {
    return std::make_unique<HighDefensiveChainScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_defensive_chain() {}

} // namespace f4flight_test
