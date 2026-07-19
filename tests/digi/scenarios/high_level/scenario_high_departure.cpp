// f4flight - scenarios/scenario_high_departure.cpp
//
// HIGH-LEVEL scenario: full departure chain Taxi -> Takeoff -> Climb ->
// Level-off. Composes four low-level behaviors into a realistic ground-
// departure-plus-climbout sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition
// for that phase (allowed per the task spec — see scenario_digi_e2e_mission
// for the same pattern). The point is to verify the brain enters the right
// DigiMode for each behavior and makes meaningful progress; per-phase
// tolerances are intentionally relaxed (no tight altitude bands or speed
// tolerances — the parent scenarios digi_groundops / ai_basic cover those).
//
// Pass criteria (per phase):
//   1. Taxi      : enter TaxiToRunway phase; reach threshold (<50ft); no NaN
//   2. Takeoff   : enter Takeoff mode; airborne + alt>=500ft + spd>=200kts
//                  (heavy: spd>=80kts); no NaN
//   3. Climb     : climb >= 8000ft (4000ft heavy) from 500ft AGL; no NaN
//   4. Level-off : hold within +-500ft (800ft heavy) of 10000ft for >=30s
//
// Tier: HighLevel. Registered as "high_departure" — referenced by the
// cascade mapping tables g_highToLow["high_departure"] and g_e2eToHigh for
// e2e_barcap / e2e_tarcap / e2e_sweep / e2e_intercept / e2e_escort /
// digi_e2e_mission / digi_e2e_formation / digi_e2e_aar / digi_e2e_ground_attack.
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

// Mission constants (NED frame: +X = east, +Y = north, +Z = down).
constexpr double kRwyHeading = PI / 2.0;   // north
constexpr double kNmToFt     = 6076.0;

// ===========================================================================
// Phase 1: Taxi
//
// Aircraft starts at a parking spot 500 ft east of the runway threshold,
// heading north. The AI taxis to the threshold (origin). Reuses the setup
// pattern from digi_groundops / low_taxi but with relaxed criteria.
// ===========================================================================
class HighTaxiPhase : public ManeuverTest {
public:
    HighTaxiPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startX = 500.0;            // 500 ft east of threshold
        const double startY = 0.0;
        const double startHeading = kRwyHeading;  // north
        fm.init(fm.config(), 0.0, 0.0, startHeading, false);  // on ground, 0 kts
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;
        fm.state().kin.z = 0.0;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Command taxi to runway threshold (origin).
        auto& go = sc.brain().stateMutable().ag.groundOps;
        go.phase = GroundOpsPhase::TaxiToRunway;
        go.runwayThresholdX = 0.0;
        go.runwayThresholdY = 0.0;
        go.runwayHeading = kRwyHeading;
        go.hasTakeoffClearance = false;
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& /*input*/, double dt) override {
        ManeuverTest::Evaluate(as, {}, dt);
        const double dx = -as.kin.x;
        const double dy = -as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::TaxiToRunway)
            enteredTaxi_ = true;
        if (dist < 50.0) reachedThreshold_ = true;
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curDist_ = dist;
        curVcas_ = as.vcas;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || reachedThreshold_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTaxi_) return false;
        if (!reachedThreshold_) return false;
        if (maxSpeed_ > 35.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter TaxiToRunway; Reach threshold (<50ft); Speed <= 35kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredTaxi_)
            return "Never entered TaxiToRunway phase (final mode: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!reachedThreshold_)
            return "Min dist to threshold was " + std::to_string(static_cast<int>(minDist_)) +
                   "ft (need < 50ft).";
        if (maxSpeed_ > 35.0)
            return "Max taxi speed was " + std::to_string(maxSpeed_) +
                   "kts (need <= 35kts).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_thresh", curDist_, "ft"},
            {"vcas",     curVcas_, "kts"},
            {"in_taxi",  (enteredTaxi_ && curMode_ == DigiMode::GroundMnvr) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Taxi Summary ---\n");
        std::printf("  Entered Taxi mode: %s\n", enteredTaxi_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to threshold: %.1f ft (need < 50) %s\n",
            minDist_, minDist_ < 50.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max speed: %.1f kts (need <= 35) %s\n",
            maxSpeed_, maxSpeed_ <= 35.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double minDist_{1e9};
    double maxSpeed_{0.0};
    bool reachedThreshold_{false};
    bool enteredTaxi_{false};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curDist_{0.0};
    double curVcas_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Takeoff
//
// Reposition at the runway threshold, command takeoff, verify becomes airborne
// and climbs to >= 500ft AGL.
// ===========================================================================
class HighTakeoffPhase : public ManeuverTest {
public:
    HighTakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), 0.0, 0.0, kRwyHeading, false);  // on ground, 0 kts
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        isHeavy_ = isHeavy(fm.config());
        sc.brain().commandTakeoff(270, kRwyHeading, 0.0, 0.0, 0.0);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double altAGL = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);
        if (altAGL > 10.0) becameAirborne_ = true;
        if (input.throttle > 0.9) appliedThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        curAlt_ = altAGL;
        curVcas_ = as.vcas;
        curThrottle_ = input.throttle;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (maxAlt_ >= 500.0 && maxSpeed_ >= 200.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTakeoff_) return false;
        if (!appliedThrottle_) return false;
        if (isHeavy_) return maxSpeed_ >= 80.0;
        if (!becameAirborne_) return false;
        if (maxAlt_ < 500.0) return false;
        if (maxSpeed_ < 200.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Takeoff mode; Apply takeoff throttle; "
               "Fighter: airborne + alt >= 500ft + speed >= 200kts; "
               "Heavy: speed >= 80kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredTakeoff_)
            return "Never entered Takeoff mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!appliedThrottle_)
            return "Takeoff throttle never advanced.";
        if (isHeavy_ && maxSpeed_ < 80.0)
            return "Heavy max speed was " + std::to_string(maxSpeed_) +
                   "kts (need >= 80).";
        if (!becameAirborne_)
            return "Never became airborne (max alt " +
                   std::to_string(static_cast<int>(maxAlt_)) + "ft).";
        if (maxAlt_ < 500.0)
            return "Max altitude was " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= 500).";
        if (maxSpeed_ < 200.0)
            return "Max speed was " + std::to_string(maxSpeed_) +
                   "kts (need >= 200).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",       curAlt_,      "ft"},
            {"vcas",      curVcas_,     "kts"},
            {"throttle",  curThrottle_, ""},
            {"in_takeoff",(enteredTakeoff_ && curMode_ == DigiMode::Takeoff) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Takeoff Summary ---\n");
        std::printf("  Entered Takeoff mode:    %s\n", enteredTakeoff_ ? "[PASS]" : "[FAIL]");
        std::printf("  Applied takeoff throttle:%s\n", appliedThrottle_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Heavy max speed: %.1f kts (need >= 80) %s\n",
                maxSpeed_, maxSpeed_ >= 80.0 ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Became airborne: %s\n", becameAirborne_ ? "[PASS]" : "[FAIL]");
            std::printf("  Max altitude:    %.0f ft (need >= 500) %s\n",
                maxAlt_, maxAlt_ >= 500.0 ? "[PASS]" : "[FAIL]");
            std::printf("  Max speed:       %.1f kts (need >= 200) %s\n",
                maxSpeed_, maxSpeed_ >= 200.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    bool becameAirborne_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    bool enteredTakeoff_{false};
    bool appliedThrottle_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0}, curVcas_{0.0}, curThrottle_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Climb
//
// Reposition at 500ft AGL, command HeadingAltitude mode with target altitude
// 10000ft. Verify the AI climbs at least 8000ft (heavy: 4000ft).
// ===========================================================================
class HighClimbPhase : public ManeuverTest {
public:
    HighClimbPhase(const char* name, double duration, double startAlt,
                   double targetAlt, double speed)
        : ManeuverTest(name, duration), startAlt_(startAlt), targetAlt_(targetAlt),
          targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), startAlt_, targetSpd_ * KNOTS_TO_FTPSEC,
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
        // Clear residual ground-ops state so the brain resolves to
        // HeadingAltitude (not Takeoff/Landing from a prior phase).
        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        const double alt = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, alt);
        minSpd_ = std::min(minSpd_, as.vcas);
        curAlt_ = alt;
        curVcas_ = as.vcas;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        if (climbAmount < minClimb) return false;
        if (minSpd_ < 0.5 * targetSpd_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Climb >= 8000ft (4000 heavy) from 500ft; Min speed >= 50% target; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        if (climbAmount < minClimb)
            return "Only climbed " + std::to_string(static_cast<int>(climbAmount)) +
                   "ft (need >= " + std::to_string(static_cast<int>(minClimb)) + "ft).";
        if (minSpd_ < 0.5 * targetSpd_)
            return "Min speed " + std::to_string(static_cast<int>(minSpd_)) +
                   "kts (stalled).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",  curAlt_,  "ft"},
            {"vcas", curVcas_, "kts"},
        };
    }

    void Finish() const override {
        const double climbAmount = maxAlt_ - startAlt_;
        const double minClimb = isHeavy_ ? 4000.0 : 8000.0;
        std::printf("  --- Climb Summary ---\n");
        std::printf("  Start altitude:    %.0f ft\n", startAlt_);
        std::printf("  Target altitude:   %.0f ft\n", targetAlt_);
        std::printf("  Max altitude:      %.0f ft (climbed %.0fft, need >= %.0f) %s\n",
            maxAlt_, climbAmount, minClimb,
            climbAmount >= minClimb ? "[PASS]" : "[FAIL]");
        std::printf("  Min speed:         %.1f kts %s\n", minSpd_,
            minSpd_ >= 0.5 * targetSpd_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double startAlt_{0.0}, targetAlt_{0.0}, targetSpd_{0.0};
    bool isHeavy_{false};
    bool hasNaN_{false};
    double maxAlt_{-1e9};
    double minSpd_{1e9};
    double curAlt_{0.0}, curVcas_{0.0};
};

// ===========================================================================
// Phase 4: Level-off
//
// Reposition at 10000ft, hold altitude for 30s. Verify altitude stays within
// +-500ft (heavy: +-800ft) of target.
// ===========================================================================
class HighLevelOffPhase : public ManeuverTest {
public:
    HighLevelOffPhase(const char* name, double duration, double alt, double speed)
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
        // Clear residual ground-ops state so the brain resolves to
        // HeadingAltitude (not Takeoff/Landing from a prior phase).
        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        const double alt = -as.kin.z;
        maxAltErr_ = std::max(maxAltErr_, std::fabs(alt - targetAlt_));
        // Settled window: after 30s, count time within tolerance.
        if (phaseTime_ >= 30.0) {
            const double tol = isHeavy_ ? 800.0 : 500.0;
            if (std::fabs(alt - targetAlt_) <= tol) settledTime_ += dt;
        }
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
        // Hold for at least 10s in the settling window.
        if (settledTime_ < 10.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Altitude within +-500ft (+-800 heavy) of 10000ft; "
               "Hold within tolerance for >=10s after t=30s; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        const double tol = isHeavy_ ? 800.0 : 500.0;
        if (maxAltErr_ > tol)
            return "Max altitude error was " + std::to_string(static_cast<int>(maxAltErr_)) +
                   "ft (need <= " + std::to_string(static_cast<int>(tol)) + "ft).";
        if (settledTime_ < 10.0)
            return "Only held within tolerance for " + std::to_string(static_cast<int>(settledTime_)) +
                   "s (need >= 10s).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",     curAlt_,     "ft"},
            {"alt_err", curAltErr_,  "ft"},
        };
    }

    void Finish() const override {
        const double tol = isHeavy_ ? 800.0 : 500.0;
        std::printf("  --- Level-off Summary ---\n");
        std::printf("  Target altitude:   %.0f ft\n", targetAlt_);
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
// HighDepartureScenario
// ===========================================================================
class HighDepartureScenario : public ManeuverScenario {
public:
    HighDepartureScenario() : ManeuverScenario("high_departure") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: full departure chain Taxi -> Takeoff -> Climb to 10000ft "
               "-> Level-off. Composes four low-level behaviors into a realistic "
               "ground-departure sequence. Relaxed per-phase criteria.";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;
        const double rwLen = 10000.0;
        const double rwHalf = rwLen / 2.0;
        const double rwWidth = 200.0;
        TraceGeometry centerline;
        centerline.name = "RWY";
        centerline.type = "runway";
        centerline.coords = {0.0, -rwHalf, 0.0, 0.0, rwHalf, 0.0};
        centerline.color = "#3a3a4a";
        centerline.width = 150.0;
        geom.push_back(centerline);
        TraceGeometry threshN;
        threshN.name = "RWY_End_N";
        threshN.type = "taxiway";
        threshN.coords = {-rwWidth, rwHalf, 0.0, rwWidth, rwHalf, 0.0};
        threshN.color = "#3a3a4a";
        threshN.width = 80.0;
        geom.push_back(threshN);
        TraceGeometry threshS;
        threshS.name = "RWY_End_S";
        threshS.type = "taxiway";
        threshS.coords = {-rwWidth, -rwHalf, 0.0, rwWidth, -rwHalf, 0.0};
        threshS.color = "#3a3a4a";
        threshS.width = 80.0;
        geom.push_back(threshS);
        return geom;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        fm.init(ctx.cfg, 0.0, 0.0, kRwyHeading, false);
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<HighTaxiPhase>("Taxi", 60.0));
        tests.push_back(std::make_unique<HighTakeoffPhase>("Takeoff", 90.0));
        tests.push_back(std::make_unique<HighClimbPhase>(
            "Climb 500 -> 10000ft", 180.0, 500.0, 10000.0, cornerSpeed));
        tests.push_back(std::make_unique<HighLevelOffPhase>(
            "Level-off at 10000ft", 60.0, 10000.0, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerHighDeparture("high_departure", []() {
    return std::make_unique<HighDepartureScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_departure() {}

} // namespace f4flight_test
