// f4flight - scenarios/scenario_high_air_to_ground.cpp
//
// HIGH-LEVEL scenario: air-to-ground attack chain Ingress -> Dive attack ->
// Toss attack -> Egress. Composes four low-level A/G behaviors into a
// realistic strike mission sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed.
//
// Pass criteria (per phase):
//   1. Ingress       : enter nav mode (Waypoint or similar); climb to >= 8000ft
//                      (heavy: 6000ft); close range to target area by >= 3NM
//                      (heavy: 2NM); heading within 30deg of north; no NaN
//   2. Dive attack   : enter GroundMnvr mode; release weapon; min alt > 500ft;
//                      no NaN
//   3. Toss attack   : enter GroundMnvr mode; release weapon; min alt > 200ft;
//                      no NaN
//   4. Egress        : RTB-mode (or nav-mode) closure to home base >= 2NM
//                      (heavy: 1NM); min alt >= 1000ft; no NaN
//
// Tier: HighLevel. Registered as "high_air_to_ground" — referenced by the
// cascade mapping tables g_highToLow["high_air_to_ground"] and g_e2eToHigh
// for digi_e2e_ground_attack.
//
// Task ID: 22

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

constexpr double kRwyHeading   = PI / 2.0;   // north
constexpr double kMissionAlt   = 15000.0;
constexpr double kNmToFt       = 6076.0;

// ===========================================================================
// Phase 1: Ingress
//
// Aircraft repositioned at 5000ft, 300 kts, heading north. Target area 10NM
// north at 15000ft. Verify the brain enters a navigation mode and closes the
// range.
// ===========================================================================
class HighIngressPhase : public ManeuverTest {
public:
    HighIngressPhase(const char* name, double duration, double alt, double speed,
                     double targetAlt, double wpRangeNm)
        : ManeuverTest(name, duration), startAlt_(alt), speed_(speed),
          targetAlt_(targetAlt), wpRangeNm_(wpRangeNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), startAlt_, speed_ * KNOTS_TO_FTPSEC,
                kRwyHeading, true);
        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setAltitude(targetAlt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        wpX_ = 0.0;
        wpY_ = wpRangeNm_ * kNmToFt;
        wpZ_ = -targetAlt_;
        std::vector<Vec3> wps;
        wps.push_back({wpX_, wpY_, wpZ_});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        // Clear residual ground-ops state.
        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        initialRange_ = std::sqrt(wpX_ * wpX_ + wpY_ * wpY_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double alt = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, alt);
        const double dx = wpX_ - as.kin.x;
        const double dy = wpY_ - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);

        double dh = as.kin.sigma - kRwyHeading;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingErr_ = std::min(minAbsHeadingErr_, std::fabs(dh));

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Waypoint || mode == DigiMode::RTB ||
            mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GroundMnvr)
            enteredNavMode_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_   = alt;
        curRange_ = range;
        curMode_  = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (minRange_ < 1.0 * kNmToFt);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredNavMode_) return false;
        const double altThreshold = isHeavy_ ? 6000.0 : 8000.0;
        if (maxAlt_ < altThreshold) return false;
        if (minAbsHeadingErr_ > 30.0 * DTR) return false;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 3.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter nav mode; Climb to >= 8000ft (6000 heavy); "
               "Heading within 30deg of north; Close range by >= 3NM (2NM heavy); "
               "No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredNavMode_)
            return "Never entered nav mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double altThreshold = isHeavy_ ? 6000.0 : 8000.0;
        if (maxAlt_ < altThreshold)
            return "Max altitude " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= " + std::to_string(static_cast<int>(altThreshold)) + "ft).";
        if (minAbsHeadingErr_ > 30.0 * DTR)
            return "Heading error to north > 30deg.";
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 3.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt)
            return "Range closure " +
                   std::to_string((initialRange_ - minRange_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 2.0 : 3.0) + "NM).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",      curAlt_,   "ft"},
            {"range_wp", curRange_, "ft"},
            {"in_nav",   (enteredNavMode_ && curMode_ != DigiMode::Takeoff &&
                          curMode_ != DigiMode::Landing) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"slot", wpX_, wpY_, wpZ_, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- Ingress Summary ---\n");
        std::printf("  Entered nav mode:   %s\n", enteredNavMode_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:       %.0f ft %s\n", maxAlt_,
            maxAlt_ >= (isHeavy_ ? 6000.0 : 8000.0) ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading err:    %.1f deg %s\n",
            minAbsHeadingErr_ * RTD,
            minAbsHeadingErr_ <= 30.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Range closure:      %.2f NM %s\n",
            (initialRange_ - minRange_) / kNmToFt,
            (initialRange_ - minRange_) >= (isHeavy_ ? 2.0 : 3.0) * kNmToFt ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double startAlt_, speed_, targetAlt_, wpRangeNm_;
    double wpX_{0.0}, wpY_{0.0}, wpZ_{0.0};
    double initialRange_{0.0};
    double minRange_{1e9};
    double maxAlt_{0.0};
    double minAbsHeadingErr_{1e9};
    bool enteredNavMode_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0}, curRange_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2/3: Generic ground-attack delivery phase
//
// Repositioned at alt, speed, south of target by startOffsetNm. Inject a
// ground target + select an attack profile. Verify GroundMnvr mode entry +
// weapon release.
// ===========================================================================
class HighAGDeliveryPhase : public ManeuverTest {
public:
    HighAGDeliveryPhase(const char* name, double duration, double alt,
                        double speed, double startOffsetNm,
                        AgAttackProfile profile, double minAltFloor,
                        const char* profileLabel)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffsetNm_(startOffsetNm), profile_(profile),
          minAltFloor_(minAltFloor), profileLabel_(profileLabel) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -startOffsetNm_ * kNmToFt;
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(60.0);

        groundTarget_.x = 0.0;
        groundTarget_.y = 0.0;
        groundTarget_.z = 0.0;
        groundTarget_.isDead = false;
        groundTarget_.dcm = dcmFromEuler(0.0, 0.0, 0.0);

        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
        fi.injectedAgProfile = profile_;
        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double dx = groundTarget_.x - as.kin.x;
        const double dy = groundTarget_.y - as.kin.y;
        const double distToTarget = std::sqrt(dx * dx + dy * dy);
        minDistToTarget_ = std::min(minDistToTarget_, distToTarget);

        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);

        if (sc_brain_->activeMode() == DigiMode::GroundMnvr) enteredGroundMnvr_ = true;
        if (input.releaseConsent) {
            weaponReleased_ = true;
            releaseAlt_ = altAGL;
        }
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curAlt_ = altAGL;
        curDist_ = distToTarget;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredGroundMnvr_) return false;
        if (!weaponReleased_) return false;
        if (minAlt_ < minAltFloor_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GroundMnvr; Release weapon; Min alt > " +
               std::to_string(static_cast<int>(minAltFloor_)) +
               "ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_)
            return "Never entered GroundMnvr mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!weaponReleased_)
            return "Never released weapon — " + std::string(profileLabel_) +
                   " profile did not reach release condition.";
        if (minAlt_ < minAltFloor_)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need > " + std::to_string(static_cast<int>(minAltFloor_)) + "ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",       curAlt_,  "ft"},
            {"d_target",  curDist_, "ft"},
            {"in_ground", (enteredGroundMnvr_ && curMode_ == DigiMode::GroundMnvr) ? 1.0 : 0.0, ""},
            {"released",  weaponReleased_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- %s Summary ---\n", profileLabel_.c_str());
        std::printf("  Entered GroundMnvr: %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:    %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        if (weaponReleased_)
            std::printf("  Release altitude:   %.0f ft\n", releaseAlt_);
        std::printf("  Min altitude:       %.0f ft (need > %.0f) %s\n",
            minAlt_, minAltFloor_, minAlt_ >= minAltFloor_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, startOffsetNm_;
    AgAttackProfile profile_;
    double minAltFloor_;
    std::string profileLabel_;
    DigiEntity groundTarget_;
    const DigiBrain* sc_brain_{nullptr};
    double minDistToTarget_{1e9};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double releaseAlt_{0.0};
    bool enteredGroundMnvr_{false};
    bool weaponReleased_{false};
    bool hasNaN_{false};
    double curAlt_{0.0}, curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 4: Egress
//
// Repositioned 10NM north of origin at mission altitude, fuel below bingo,
// friendly airbase at origin. Verify the AI enters RTB / nav mode and closes
// the distance to home base.
// ===========================================================================
class HighEgressPhase : public ManeuverTest {
public:
    HighEgressPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startY = 10.0 * kNmToFt;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = startY;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        airbase_.x = 0.0;
        airbase_.y = 0.0;
        airbase_.z = 0.0;
        airbase_.runwayHeading = kRwyHeading;
        airbase_.id = 270;

        FrameInputs fi;
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);
        sc.brain().clearTarget();

        initialDist_ = startY;
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedTarget = nullptr;
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::RTB)      enteredRTB_ = true;
        if (mode == DigiMode::Waypoint) enteredNav_ = true;

        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = dist;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRTB_ && !enteredNav_) return false;
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 2.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt) return false;
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB or nav mode; Close distance by >= 2NM (1NM heavy); "
               "Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRTB_ && !enteredNav_)
            return "Never entered RTB or nav mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 2.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt)
            return "Distance closure " +
                   std::to_string((initialDist_ - minDist_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 1.0 : 2.0) + "NM).";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 1000ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"dist_ab", curDist_, "ft"},
            {"in_rtb",  (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- Egress Summary ---\n");
        std::printf("  Entered RTB/nav:   %s (RTB=%d nav=%d)\n",
            (enteredRTB_ || enteredNav_) ? "[PASS]" : "[FAIL]",
            enteredRTB_, enteredNav_);
        std::printf("  Dist closure:      %.2f NM %s\n",
            (initialDist_ - minDist_) / kNmToFt,
            (initialDist_ - minDist_) >= (isHeavy_ ? 1.0 : 2.0) * kNmToFt ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:      %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    FrameInputs::AirbaseInfo airbase_;
    DigiBrain* sc_brain_{nullptr};
    double initialDist_{0.0};
    double minDist_{1e9};
    double minAlt_{1e9};
    bool enteredRTB_{false};
    bool enteredNav_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// HighAirToGroundScenario
// ===========================================================================
class HighAirToGroundScenario : public ManeuverScenario {
public:
    HighAirToGroundScenario() : ManeuverScenario("high_air_to_ground") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: air-to-ground attack chain Ingress -> Dive attack -> "
               "Toss attack -> Egress. Composes four low-level A/G behaviors "
               "into a realistic strike mission. Relaxed per-phase criteria.";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;
        const double rwLen = 10000.0;
        const double rwHalf = rwLen / 2.0;
        TraceGeometry tg;
        tg.name = "RWY";
        tg.type = "runway";
        tg.coords = {0.0, -rwHalf, 0.0, 0.0, rwHalf, 0.0};
        tg.color = "#3a3a4a";
        tg.width = 150.0;
        geom.push_back(tg);
        return geom;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, kMissionAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: Ingress to target area (10NM north)
        tests.push_back(std::make_unique<HighIngressPhase>(
            "Ingress (10NM to target)", 90.0,
            5000.0, cornerSpeed, kMissionAlt, 10.0));
        // Phase 2: Dive-bomb profile (12k ft start, 6 NM south, min alt 500ft)
        tests.push_back(std::make_unique<HighAGDeliveryPhase>(
            "Dive-bomb attack", 90.0,
            12000.0, cornerSpeed, 6.0,
            AgAttackProfile::DiveBomb, 500.0, "DiveBomb"));
        // Phase 3: Toss-bomb profile (1500 ft start, 8 NM south, min alt 200ft)
        tests.push_back(std::make_unique<HighAGDeliveryPhase>(
            "Toss attack", 110.0,
            1500.0, 400.0, 8.0,
            AgAttackProfile::TossBomb, 200.0, "TossBomb"));
        // Phase 4: Egress (10NM north of origin, bingo fuel)
        tests.push_back(std::make_unique<HighEgressPhase>(
            "Egress RTB (10NM to home)", 90.0,
            kMissionAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerHighAirToGround("high_air_to_ground", []() {
    return std::make_unique<HighAirToGroundScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_air_to_ground() {}

} // namespace f4flight_test
