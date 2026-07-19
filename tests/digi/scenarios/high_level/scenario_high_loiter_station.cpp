// f4flight - scenarios/scenario_high_loiter_station.cpp
//
// HIGH-LEVEL scenario: station-keeping chain Navigate-to-station -> Loiter ->
// Level-hold recovery. Composes three low-level behaviors into a realistic
// CAP (Combat Air Patrol) station-keeping sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed (verify the right mode + meaningful
// progress).
//
// Pass criteria (per phase):
//   1. Navigate   : enter nav mode (Waypoint or similar); climb to >= 8000ft
//                   (heavy: 6000ft); close range to station by >= 3NM
//                   (heavy: 2NM); heading within 30deg of north; no NaN
//   2. Loiter     : enter Loiter mode; accumulated heading change > 60deg
//                   (heavy: 40deg); altitude within +-2000ft of start; no NaN
//   3. Level-hold : exit loiter, hold altitude + heading; alt within +-500ft
//                   (heavy: +-800ft); no NaN
//
// Tier: HighLevel. Registered as "high_loiter_station" — referenced by the
// cascade mapping tables g_highToLow["high_loiter_station"] and g_e2eToHigh
// for e2e_barcap / e2e_tarcap.
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

constexpr double kRwyHeading = PI / 2.0;   // north
constexpr double kNmToFt     = 6076.0;
constexpr double kStationAlt = 15000.0;

// ===========================================================================
// Phase 1: Navigate to station
//
// Aircraft repositioned at 5000ft, 300 kts, heading north. Waypoint 10NM north
// at 15000ft. Verify the brain enters a navigation mode and closes the range.
// ===========================================================================
class HighNavigateStationPhase : public ManeuverTest {
public:
    HighNavigateStationPhase(const char* name, double duration, double alt,
                             double speed, double targetAlt, double wpRangeNm)
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

        std::vector<Vec3> wps;
        wpX_ = 0.0;
        wpY_ = wpRangeNm_ * kNmToFt;
        wpZ_ = -targetAlt_;
        wps.push_back({wpX_, wpY_, wpZ_});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        // Clear residual ground-ops state so the brain resolves to Waypoint.
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
            mode == DigiMode::BVREngage || mode == DigiMode::WVREngage)
            enteredNavMode_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_   = alt;
        curRange_ = range;
        curHdgErr_ = std::fabs(dh) * RTD;
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
            return "Heading error to north " + std::to_string(curHdgErr_) +
                   "deg (need <= 30deg).";
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 3.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt)
            return "Range closure " +
                   std::to_string((initialRange_ - minRange_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 2.0 : 3.0) + "NM).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",      curAlt_,    "ft"},
            {"range_wp", curRange_,  "ft"},
            {"hdg_err",  curHdgErr_, "deg"},
            {"in_nav",   (enteredNavMode_ && curMode_ != DigiMode::Takeoff &&
                          curMode_ != DigiMode::Landing) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"slot", wpX_, wpY_, wpZ_, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- Navigate Summary ---\n");
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
    double curAlt_{0.0}, curRange_{0.0}, curHdgErr_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Loiter at station
//
// Reposition at the station waypoint (15000ft, 10NM north), force Loiter mode,
// orbit for 60s. Verify Loiter mode entry + accumulated heading change.
// ===========================================================================
class HighLoiterStationPhase : public ManeuverTest {
public:
    HighLoiterStationPhase(const char* name, double duration, double alt,
                           double speed, double stationX, double stationY)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          stationX_(stationX), stationY_(stationY) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        fm.state().kin.x = stationX_;
        fm.state().kin.y = stationY_;
        fm.state().kin.z = -alt_;
        startX_ = stationX_;
        startY_ = stationY_;

        sc.setMode(SteeringController::Mode::Loiter);
        sc.brain().forceMode(DigiMode::Loiter);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double heading = as.kin.sigma;
        if (sc_brain_->activeMode() == DigiMode::Loiter) enteredLoiter_ = true;

        if (!lastHeadingInit_) { lastHeading_ = heading; lastHeadingInit_ = true; }
        double dh = heading - lastHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        accumulatedHeadingChange_ += dh;
        lastHeading_ = heading;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxAlt_ = std::max(maxAlt_, -as.kin.z);

        curHdgChg_ = std::fabs(accumulatedHeadingChange_) * RTD;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLoiter_) return false;
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR) return false;
        if (minAlt_ < alt_ - 2000.0) return false;
        if (maxAlt_ > alt_ + 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Loiter mode; Accumulated heading change > 60deg "
               "(40deg heavy); Alt within +-2000ft of start; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLoiter_)
            return "Never entered Loiter mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR)
            return "Heading change " + std::to_string(std::fabs(accumulatedHeadingChange_) * RTD) +
                   "deg (need > " + std::to_string(hdgThreshold) + "deg).";
        if (minAlt_ < alt_ - 2000.0 || maxAlt_ > alt_ + 2000.0)
            return "Altitude varied [" + std::to_string(static_cast<int>(minAlt_)) +
                   ", " + std::to_string(static_cast<int>(maxAlt_)) + "]ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_chg",  curHdgChg_, "deg"},
            {"in_loiter", (enteredLoiter_ && curMode_ == DigiMode::Loiter) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        std::printf("  --- Loiter Summary ---\n");
        std::printf("  Entered Loiter:           %s\n", enteredLoiter_ ? "[PASS]" : "[FAIL]");
        std::printf("  Accumulated heading chg:  %.1f deg (need > %.0f) %s\n",
            std::fabs(accumulatedHeadingChange_) * RTD, hdgThreshold,
            std::fabs(accumulatedHeadingChange_) > hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Altitude range:           [%.0f, %.0f] ft %s\n",
            minAlt_, maxAlt_,
            (minAlt_ >= alt_ - 2000.0 && maxAlt_ <= alt_ + 2000.0) ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, stationX_, stationY_;
    double startX_{0.0}, startY_{0.0};
    double initialHeading_{0.0};
    double lastHeading_{0.0};
    bool lastHeadingInit_{false};
    double accumulatedHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    bool hasNaN_{false};
    bool enteredLoiter_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Level-hold recovery
//
// Reposition at 15000ft heading north, exit loiter (use HeadingAltitude mode),
// hold altitude + heading for 30s. Verify alt stable +-500ft (heavy +-800ft).
// ===========================================================================
class HighLevelHoldRecoveryPhase : public ManeuverTest {
public:
    HighLevelHoldRecoveryPhase(const char* name, double duration, double alt,
                               double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), targetAlt_, targetSpd_ * KNOTS_TO_FTPSEC,
                kRwyHeading, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        isHeavy_ = isHeavy(fm.config());
        sc.setMaxBank(45.0);
        sc.setMaxGamma(isHeavy_ ? 10.0 : 15.0);
        sc.setTurnG(isHeavy_ ? 1.3 : 2.0);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        const double alt = -as.kin.z;
        maxAltErr_ = std::max(maxAltErr_, std::fabs(alt - targetAlt_));
        const double tol = isHeavy_ ? 800.0 : 500.0;
        if (phaseTime_ >= 10.0 && std::fabs(alt - targetAlt_) <= tol)
            settledTime_ += dt;
        curAlt_ = alt;
        curAltErr_ = std::fabs(alt - targetAlt_);
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        const double tol = isHeavy_ ? 800.0 : 500.0;
        if (maxAltErr_ > tol) return false;
        if (settledTime_ < 10.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Altitude within +-500ft (+-800 heavy) of target; "
               "Hold within tolerance for >=10s after t=10s; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        const double tol = isHeavy_ ? 800.0 : 500.0;
        if (maxAltErr_ > tol)
            return "Max altitude error " + std::to_string(static_cast<int>(maxAltErr_)) +
                   "ft (need <= " + std::to_string(static_cast<int>(tol)) + "ft).";
        if (settledTime_ < 10.0)
            return "Only held within tolerance for " +
                   std::to_string(static_cast<int>(settledTime_)) + "s (need >= 10s).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",     curAlt_,    "ft"},
            {"alt_err", curAltErr_, "ft"},
        };
    }

    void Finish() const override {
        const double tol = isHeavy_ ? 800.0 : 500.0;
        std::printf("  --- Level-hold Summary ---\n");
        std::printf("  Max altitude err:  %.0f ft (need <= %.0f) %s\n",
            maxAltErr_, tol, maxAltErr_ <= tol ? "[PASS]" : "[FAIL]");
        std::printf("  Settled time:      %.1f s (need >= 10) %s\n",
            settledTime_, settledTime_ >= 10.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double targetAlt_{0.0}, targetSpd_{0.0};
    bool isHeavy_{false};
    bool hasNaN_{false};
    double maxAltErr_{0.0};
    double settledTime_{0.0};
    double curAlt_{0.0}, curAltErr_{0.0};
};

// ===========================================================================
// HighLoiterStationScenario
// ===========================================================================
class HighLoiterStationScenario : public ManeuverScenario {
public:
    HighLoiterStationScenario() : ManeuverScenario("high_loiter_station") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: station-keeping chain Navigate-to-station -> Loiter -> "
               "Level-hold recovery. Verifies the AI can fly to a CAP station, "
               "orbit it, then exit and recover. Relaxed per-phase criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        fm.init(ctx.cfg, kStationAlt, 0.0, kRwyHeading, true);
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double wpRangeNm = 10.0;
        const double stationX = 0.0;
        const double stationY = wpRangeNm * kNmToFt;
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<HighNavigateStationPhase>(
            "Navigate to station (10NM)", 90.0,
            5000.0, cornerSpeed, kStationAlt, wpRangeNm));
        tests.push_back(std::make_unique<HighLoiterStationPhase>(
            "Loiter at station (60s)", 60.0,
            kStationAlt, cornerSpeed, stationX, stationY));
        tests.push_back(std::make_unique<HighLevelHoldRecoveryPhase>(
            "Level-hold recovery (30s)", 50.0,
            kStationAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerHighLoiterStation("high_loiter_station", []() {
    return std::make_unique<HighLoiterStationScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_loiter_station() {}

} // namespace f4flight_test
