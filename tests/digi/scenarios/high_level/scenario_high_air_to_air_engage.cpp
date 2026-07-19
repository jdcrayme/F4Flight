// f4flight - scenarios/scenario_high_air_to_air_engage.cpp
//
// HIGH-LEVEL scenario: air-to-air engagement chain BVR engage -> Merge ->
// WVR engage -> Separate (disengage). Composes four low-level air-to-air
// behaviors into a realistic engagement sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed.
//
// Pass criteria (per phase):
//   1. BVR engage  : enter BVREngage (or any offensive) mode; no NaN; min alt
//                    >= 5000ft
//   2. Merge       : enter Merge or WVREngage mode; heading change > 15deg
//                    (heavy: 10deg); min alt >= 5000ft; no NaN
//   3. WVR engage  : enter WVREngage (or any offensive) mode; aggressive
//                    maneuvering (G > 2.0 OR hdg change > 15deg, heavy: G > 1.1);
//                    min alt >= 5000ft; no NaN
//   4. Separate    : enter RTB/Bugout/Separate (disengage) mode OR pctStrength
//                    < 0.5 forces RTB; no NaN; min alt >= 1000ft
//
// Tier: HighLevel. Registered as "high_air_to_air_engage" — referenced by the
// cascade mapping tables g_highToLow["high_air_to_air_engage"] and g_e2eToHigh
// for e2e_barcap / e2e_tarcap / e2e_sweep / e2e_intercept / e2e_escort /
// digi_e2e_mission.
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

constexpr double kEngageAlt = 15000.0;
constexpr double kNmToFt    = 6076.0;

// ===========================================================================
// Phase 1: BVR engage
//
// Target 15 NM ahead, same heading (east), slower (250 kts). AI at corner
// speed should enter BVREngage and close on the target.
// ===========================================================================
class HighBVREngagePhase : public ManeuverTest {
public:
    HighBVREngagePhase(const char* name, double duration, double alt,
                       double speed, double targetSpeed, double initialRangeNm)
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

        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = rangeFt;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;  // east (same as us)
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
        if (mode == DigiMode::BVREngage) enteredBVR_ = true;
        if (mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GunsEngage || mode == DigiMode::MissileEngage ||
            mode == DigiMode::Merge)
            enteredOffensive_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curRange_ = range;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredOffensive_) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter offensive mode (BVR preferred); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredOffensive_)
            return "Never entered an offensive mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",   curRange_, "ft"},
            {"in_bvr",  (enteredBVR_ && curMode_ == DigiMode::BVREngage) ? 1.0 : 0.0, ""},
            {"in_off",  enteredOffensive_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- BVR Engage Summary ---\n");
        std::printf("  Entered offensive mode: %s (BVR=%d)\n",
            enteredOffensive_ ? "[PASS]" : "[FAIL]", enteredBVR_);
        std::printf("  Min range:    %.0f ft\n", minRange_);
        std::printf("  Min altitude: %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_;
    DigiEntity target_;
    const DigiBrain* sc_brain_{nullptr};
    double minRange_{1e9};
    double minAlt_{1e9};
    bool enteredBVR_{false};
    bool enteredOffensive_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curRange_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Merge
//
// Target 5000 ft ahead, head-on, closing fast. AI should enter Merge or
// WVREngage mode at the close pass.
// ===========================================================================
class HighMergePhase : public ManeuverTest {
public:
    HighMergePhase(const char* name, double duration, double alt, double speed,
                   double targetSpeed, double initialRangeFt)
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
        if (target_.x < -2000.0) target_.x = initialRangeFt_;

        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(as.kin.sigma - initialHeading_));
        minAlt_ = std::min(minAlt_, -as.kin.z);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Merge) enteredMerge_ = true;
        if (mode == DigiMode::WVREngage) enteredWVR_ = true;
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
        if (!enteredMerge_ && !enteredWVR_) return false;
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Merge or WVREngage mode; Heading change >= 15deg (10 heavy); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredMerge_ && !enteredWVR_)
            return "Never entered Merge or WVREngage (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR)
            return "Heading change was " + std::to_string(maxHeadingChange_ * RTD) +
                   "deg (need > " + std::to_string(hdgThreshold) + "deg).";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_chg", curHdgChg_, "deg"},
            {"in_merge",(enteredMerge_ && curMode_ == DigiMode::Merge) ? 1.0 : 0.0, ""},
            {"in_wvr",  (enteredWVR_ && curMode_ == DigiMode::WVREngage) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Merge Summary ---\n");
        std::printf("  Entered Merge/WVR: %s (Merge=%d WVR=%d Coll=%d)\n",
            (enteredMerge_ || enteredWVR_) ? "[PASS]" : "[FAIL]",
            enteredMerge_, enteredWVR_, enteredCollision_);
        std::printf("  Max heading chg: %.1f deg %s\n", maxHeadingChange_ * RTD,
            maxHeadingChange_ > (isHeavy_ ? 10.0 : 15.0) * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:    %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeFt_;
    DigiEntity target_;
    const DigiBrain* sc_brain_{nullptr};
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAlt_{1e9};
    bool enteredMerge_{false};
    bool enteredWVR_{false};
    bool enteredCollision_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: WVR engage
//
// Target 2 NM behind, slightly faster, in pursuit. AI should enter WVREngage
// and aggressively maneuver.
// ===========================================================================
class HighWVREngagePhase : public ManeuverTest {
public:
    HighWVREngagePhase(const char* name, double duration, double alt,
                       double speed, double targetSpeed, double initialRangeNm)
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

        // Target behind us (-X), heading east (chasing us), slightly faster.
        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = -rangeFt;
        target_.y = 0.0;
        target_.z = -alt_;
        target_.yaw = 0.0;  // east (chasing us)
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = target_.speed;
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        target_.x += target_.speed * dt;
        // If target passes us, loop it back.
        if (target_.x > 2000.0) target_.x = -initialRangeNm_ * kNmToFt;

        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(as.kin.sigma - initialHeading_));
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::WVREngage) enteredWVR_ = true;
        if (mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GunsEngage || mode == DigiMode::MissileEngage ||
            mode == DigiMode::Merge)
            enteredOffensive_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curG_ = as.loads.nzcgs;
        curHdgChg_ = maxHeadingChange_ * RTD;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredOffensive_) return false;
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR && maxG_ < gThreshold) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter offensive mode (WVR preferred); Maneuver (G > 2 OR hdg > 15deg, "
               "heavy: G > 1.1); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredOffensive_)
            return "Never entered offensive mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR && maxG_ < gThreshold)
            return "No maneuver: max hdg change " + std::to_string(maxHeadingChange_ * RTD) +
                   "deg AND max G " + std::to_string(maxG_) + ".";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 5000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"G",       curG_,      ""},
            {"hdg_chg", curHdgChg_, "deg"},
            {"in_wvr",  (enteredWVR_ && curMode_ == DigiMode::WVREngage) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- WVR Engage Summary ---\n");
        std::printf("  Entered offensive: %s (WVR=%d)\n",
            enteredOffensive_ ? "[PASS]" : "[FAIL]", enteredWVR_);
        std::printf("  Max G:           %.2f %s\n", maxG_,
            (maxG_ >= (isHeavy_ ? 1.1 : 2.0) ||
             maxHeadingChange_ >= (isHeavy_ ? 10.0 : 15.0) * DTR) ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading chg: %.1f deg\n", maxHeadingChange_ * RTD);
        std::printf("  Min altitude:    %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_;
    DigiEntity target_;
    const DigiBrain* sc_brain_{nullptr};
    double initialHeading_{0.0};
    double maxHeadingChange_{0.0};
    double maxG_{0.0};
    double minAlt_{1e9};
    bool enteredWVR_{false};
    bool enteredOffensive_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curG_{0.0}, curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 4: Separate (disengage)
//
// Aircraft damaged (pctStrength = 0.3, below 0.50 abort threshold). Airbase
// 20 NM north. AI should enter RTB / Bugout / Separate mode to disengage.
// Note: per the digi_separate scenario docs, the actual implementation routes
// damage-abort through RTB. We accept RTB / Bugout / Separate as "disengage".
// ===========================================================================
class HighSeparatePhase : public ManeuverTest {
public:
    HighSeparatePhase(const char* name, double duration, double alt, double speed)
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

        // Friendly airbase 20 NM north.
        airbase_.x = 0.0;
        airbase_.y = 20.0 * kNmToFt;
        airbase_.z = -5000.0;
        airbase_.runwayHeading = 0.0;
        airbase_.id = 100;

        // Damage state: pctStrength = 0.3 (below 0.50 abort threshold).
        FrameInputs fi;
        fi.pctStrength = 0.3;
        fi.fuelLbs = 5000.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);

        // Manually set divert airbase (so runRTB has a real target).
        sc.brain().stateMutable().fuel.divertAirbaseX = airbase_.x;
        sc.brain().stateMutable().fuel.divertAirbaseY = airbase_.y;
        sc.brain().stateMutable().fuel.divertAirbaseZ = airbase_.z;
        sc.brain().stateMutable().fuel.divertAirbaseHeading = airbase_.runwayHeading;
        sc.brain().stateMutable().fuel.hasDivertAirbase = true;

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        // Re-apply damage state each frame.
        FrameInputs fi = sc_brain_->frameInputs();
        fi.pctStrength = 0.3;
        fi.fuelLbs = 5000.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::RTB)      enteredRTB_ = true;
        if (mode == DigiMode::Separate) enteredSeparate_ = true;
        if (mode == DigiMode::Bugout)   enteredBugout_ = true;

        // Track heading convergence to north (airbase bearing).
        double dh = as.kin.sigma - (PI / 2.0);
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToNorth_ = std::min(minAbsHeadingToNorth_, std::fabs(dh));
        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curHdgToNorth_ = std::fabs(dh) * RTD;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRTB_ && !enteredSeparate_ && !enteredBugout_) return false;
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB / Separate / Bugout (disengage); Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRTB_ && !enteredSeparate_ && !enteredBugout_)
            return "Never entered a disengage mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 1000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_north", curHdgToNorth_, "deg"},
            {"in_rtb",    (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Separate Summary ---\n");
        std::printf("  Entered disengage: %s (RTB=%d Sep=%d Bug=%d)\n",
            (enteredRTB_ || enteredSeparate_ || enteredBugout_) ? "[PASS]" : "[FAIL]",
            enteredRTB_, enteredSeparate_, enteredBugout_);
        std::printf("  Min hdg to north:  %.1f deg\n", minAbsHeadingToNorth_ * RTD);
        std::printf("  Min altitude:      %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    FrameInputs::AirbaseInfo airbase_;
    DigiBrain* sc_brain_{nullptr};
    double minAlt_{1e9};
    double minAbsHeadingToNorth_{1e9};
    bool enteredRTB_{false};
    bool enteredSeparate_{false};
    bool enteredBugout_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curHdgToNorth_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// HighAirToAirEngageScenario
// ===========================================================================
class HighAirToAirEngageScenario : public ManeuverScenario {
public:
    HighAirToAirEngageScenario() : ManeuverScenario("high_air_to_air_engage") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: air-to-air engagement chain BVR -> Merge -> WVR -> "
               "Separate. Composes four low-level A/A behaviors into a "
               "realistic engagement sequence. Relaxed per-phase criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, kEngageAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: BVR engage (15NM target, 250 kts evader)
        tests.push_back(std::make_unique<HighBVREngagePhase>(
            "BVR engage (15NM)", 30.0, kEngageAlt, cornerSpeed, 250.0, 15.0));
        // Phase 2: Merge (5000ft head-on, fast closure)
        tests.push_back(std::make_unique<HighMergePhase>(
            "Merge (5000ft head-on)", 10.0, kEngageAlt, cornerSpeed,
            cornerSpeed, 5000.0));
        // Phase 3: WVR engage (2NM behind, faster pursuer)
        tests.push_back(std::make_unique<HighWVREngagePhase>(
            "WVR engage (2NM pursuit)", 30.0, kEngageAlt, cornerSpeed,
            cornerSpeed + 50.0, 2.0));
        // Phase 4: Separate (damage abort -> RTB)
        tests.push_back(std::make_unique<HighSeparatePhase>(
            "Separate (damage abort)", 30.0, kEngageAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerHighAirToAirEngage("high_air_to_air_engage", []() {
    return std::make_unique<HighAirToAirEngageScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_air_to_air_engage() {}

} // namespace f4flight_test
