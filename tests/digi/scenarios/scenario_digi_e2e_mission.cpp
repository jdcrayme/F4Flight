// f4flight - scenarios/scenario_digi_e2e_mission.cpp
//
// End-to-end multi-phase mission scenario for the digi AI.
//
// This scenario chains four phases of a single combat mission to exercise
// the full mode pipeline of the DigiBrain in mission order:
//
//   Phase 1 "Takeoff"        : ground start → Takeoff mode → climb out
//   Phase 2 "Navigate"       : climb + waypoint following to a 20 NM IP
//   Phase 3 "Intercept"      : inject an evading target → BVR/WVR engage
//   Phase 4 "RTB"            : bingo fuel + airbase → RTB → divert
//
// The framework runs the phases in order, calling Init() for each. Each
// phase's Init() re-initializes the FlightModel to a deterministic starting
// condition (the task description explicitly allows repositioning via
// fm.init() so the test does not depend on the exact end-state of the
// previous phase). The DigiBrain state (mode, config, frame inputs) is
// cleared / reconfigured by each Init() as needed.
//
// Pass criteria are intentionally relaxed compared to the per-mode
// scenarios (digi_groundops, digi_bvr, digi_rtb) — the goal here is to
// verify the brain enters the RIGHT MODE for each mission segment and
// makes meaningful progress, not to re-verify the per-mode tolerances.
//
// Task ID: 14-a

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// Mission constants (NED frame: +X = east, +Y = north, +Z = down).
constexpr double kRwyHeading   = PI / 2.0;   // north
constexpr double kMissionAlt   = 15000.0;    // ft MSL
constexpr double kNmToFt       = 6076.0;

// ===========================================================================
// Phase 1: Takeoff
//
// Aircraft starts on the runway at the origin, heading north. The brain is
// commanded to take off. We verify the Takeoff mode latches, the throttle
// advances, and the aircraft rotates and climbs out.
// ===========================================================================
class E2ETakeoffPhase : public ManeuverTest {
public:
    E2ETakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), 0.0, 0.0, kRwyHeading, false);  // on ground, 0 kts
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        isHeavy_ = isHeavy(fm.config());

        // Command takeoff on runway heading (north). Runway threshold at
        // origin, sea level. ATC clearance is auto-granted inside
        // commandTakeoff().
        sc.brain().commandTakeoff(270, kRwyHeading, 0.0, 0.0, 0.0);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;  // groundZ = 0
        maxAlt_   = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (altAGL > 10.0)              becameAirborne_ = true;
        if (input.throttle > 0.9)       appliedTakeoffThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_     = altAGL;
        curVcas_    = as.vcas;
        curThrottle_= input.throttle;
        curMode_    = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (maxAlt_ >= 500.0 && maxSpeed_ >= 200.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTakeoff_)         return false;
        if (!appliedTakeoffThrottle_) return false;
        if (isHeavy_) return maxSpeed_ >= 80.0;  // heavy: just need acceleration
        if (!becameAirborne_)         return false;
        if (maxAlt_   < 500.0)        return false;
        if (maxSpeed_ < 200.0)        return false;
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
        if (!appliedTakeoffThrottle_)
            return "Takeoff throttle never advanced (max " +
                   std::to_string(curThrottle_) + ", needed > 0.9).";
        if (isHeavy_ && maxSpeed_ < 80.0)
            return "Heavy max speed " + std::to_string(maxSpeed_) +
                   "kts (need >= 80).";
        if (!becameAirborne_)
            return "Never became airborne (max alt " +
                   std::to_string(static_cast<int>(maxAlt_)) + "ft).";
        if (maxAlt_ < 500.0)
            return "Max altitude " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= 500).";
        if (maxSpeed_ < 200.0)
            return "Max speed " + std::to_string(maxSpeed_) +
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
        std::printf("  Applied takeoff throttle:%s\n", appliedTakeoffThrottle_ ? "[PASS]" : "[FAIL]");
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
    bool appliedTakeoffThrottle_{false};
    const DigiBrain* sc_brain_{nullptr};

    double curAlt_{0.0};
    double curVcas_{0.0};
    double curThrottle_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Navigate to intercept point
//
// After takeoff, the aircraft climbs and navigates to a waypoint 20 NM north
// at 15000 ft. We re-init the aircraft at a low post-takeoff altitude (3000 ft,
// 300 kts, heading north) and command a single waypoint at (0, 20NM, -15000).
//
// Verify the brain:
//   - Exits Takeoff mode and navigates (Waypoint mode or similar)
//   - Climbs toward the waypoint altitude (reaches 10000+ ft)
//   - Holds a north heading (within 25° of north)
//   - Closes the range to the waypoint by at least 5 NM
// ===========================================================================
class E2ENavigatePhase : public ManeuverTest {
public:
    E2ENavigatePhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at a low post-takeoff condition: 3000 ft, 300 kts, north.
        const double startAlt = 3000.0;
        const double startSpeed = 300.0;
        fm.init(fm.config(), startAlt, startSpeed * KNOTS_TO_FTPSEC,
                kRwyHeading, true);

        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setAltitude(kMissionAlt);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Waypoint 20 NM north at mission altitude. Single-waypoint plan.
        std::vector<Vec3> wps;
        wps.push_back({0.0, 20.0 * kNmToFt, -kMissionAlt});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        // Exit any residual Takeoff / Landing ground-ops state from the
        // previous phase so the brain resolves to Waypoint navigation.
        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        wpX_ = 0.0;
        wpY_ = 20.0 * kNmToFt;
        wpZ_ = -kMissionAlt;
        initialRange_ = std::sqrt((wpX_ - 0.0) * (wpX_ - 0.0) +
                                  (wpY_ - 0.0) * (wpY_ - 0.0));

        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double alt = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, alt);

        // Range to waypoint (2D, north-east).
        const double dx = wpX_ - as.kin.x;
        const double dy = wpY_ - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);

        // Heading convergence to north (PI/2 in this convention).
        double dh = as.kin.sigma - kRwyHeading;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingErr_ = std::min(minAbsHeadingErr_, std::fabs(dh));

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Waypoint || mode == DigiMode::RTB ||
            mode == DigiMode::BVREngage || mode == DigiMode::WVREngage)
            enteredNavMode_ = true;
        if (mode == DigiMode::Takeoff || mode == DigiMode::Landing)
            stuckInGroundOps_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_   = alt;
        curRange_ = range;
        curHdgErr_= std::fabs(dh) * RTD;
        curMode_  = mode;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (climb 3k->15k ft, navigate to 20NM north)\n",
                    testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, alt, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.rstick, digiModeName(mode));
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (minRange_ < 1.0 * kNmToFt);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered a navigation mode (not stuck in Takeoff).
        if (!enteredNavMode_) return false;
        // 2. Must have climbed meaningfully toward the waypoint altitude.
        //    Heavy aircraft (B-52, C-130) climb slowly; relax to 6000 ft.
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        if (maxAlt_ < altThreshold) return false;
        // 3. Must hold a roughly north heading (toward the waypoint).
        if (minAbsHeadingErr_ > 25.0 * DTR) return false;
        // 4. Must close the range to the waypoint by at least 5 NM
        //    (heavy: 2 NM). Started at 20 NM = 121520 ft.
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter nav mode (not stuck in Takeoff); Climb to >= 10000ft "
               "(6000ft heavy); Heading within 25deg of north; "
               "Close range to waypoint by >= 5NM (2NM heavy); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredNavMode_)
            return "Never entered a navigation mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   "); stuck in ground-ops.";
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        if (maxAlt_ < altThreshold)
            return "Max altitude was " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= " + std::to_string(static_cast<int>(altThreshold)) +
                   "ft) — did not climb toward waypoint altitude.";
        if (minAbsHeadingErr_ > 25.0 * DTR)
            return "Heading error to north was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= 25deg) — did not hold north heading.";
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt)
            return "Range closure was " +
                   std::to_string((initialRange_ - minRange_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 2.0 : 5.0) +
                   "NM) — did not close on the waypoint.";
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

    // Publish the waypoint as a "slot" entity so it shows in the report.
    std::vector<ThreatEntity> traceEntities() const override {
        return {{"slot", wpX_, wpY_, wpZ_, 0.0}};
    }

    void Finish() const override {
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        std::printf("  --- Navigate Summary ---\n");
        std::printf("  Entered nav mode:   %s\n", enteredNavMode_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:       %.0f ft (need >= %.0f) %s\n",
            maxAlt_, altThreshold, maxAlt_ >= altThreshold ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading err:    %.1f deg (need <= 25) %s\n",
            minAbsHeadingErr_ * RTD,
            minAbsHeadingErr_ <= 25.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Range closure:      %.2f NM (need >= %.1f) %s\n",
            (initialRange_ - minRange_) / kNmToFt, isHeavy_ ? 2.0 : 5.0,
            (initialRange_ - minRange_) >= requiredClosureFt ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double wpX_{0.0}, wpY_{0.0}, wpZ_{0.0};
    double initialRange_{0.0};
    double minRange_{1e9};
    double maxAlt_{0.0};
    double minAbsHeadingErr_{1e9};
    bool enteredNavMode_{false};
    bool stuckInGroundOps_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    double curAlt_{0.0};
    double curRange_{0.0};
    double curHdgErr_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Intercept evading target
//
// The aircraft is repositioned at mission altitude heading north. A target
// (DigiEntity) is injected 10 NM ahead, slightly offset east, flying north
// (away from the aircraft) at a slower speed. The brain should detect the
// target, enter an offensive mode (BVREngage / WVREngage / GunsEngage /
// MissileEngage), close range, and maneuver.
//
// The target flies straight north at its speed; we update its position each
// frame in Evaluate().
// ===========================================================================
class E2EInterceptPhase : public ManeuverTest {
public:
    E2EInterceptPhase(const char* name, double duration,
                      double alt, double speed, double targetSpeed,
                      double initialRangeNm, double offsetFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm),
          offsetFt_(offsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Target ahead (+Y north), slightly offset east (+X), same altitude,
        // flying north (away) at a slower speed.
        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = offsetFt_;
        target_.y = rangeFt;
        target_.z = -alt_;
        target_.yaw = kRwyHeading;   // north
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = target_.speed;  // north
        target_.vz = 0.0;
        target_.isDead = false;

        // Inject the target (bypasses SensorFusion — like digi_bvr / digi_wvr).
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = kRwyHeading;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target north at its speed.
        target_.y += target_.speed * dt;

        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_   = std::max(maxG_, as.loads.nzcgs);

        // Track heading change from the initial heading (maneuver indicator).
        double dh = as.kin.sigma - initialHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(dh));

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GunsEngage || mode == DigiMode::MissileEngage ||
            mode == DigiMode::Merge) {
            enteredOffensive_ = true;
        }
        if (mode == DigiMode::BVREngage) enteredBVR_ = true;
        if (mode == DigiMode::WVREngage) enteredWVR_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curRange_ = range;
        curG_     = as.loads.nzcgs;
        curMode_  = mode;
        curHdgChg_= maxHeadingChange_ * RTD;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.1f NM ahead, %.0f kts, evading north)\n",
                    testName_.c_str(), initialRangeNm_, targetSpeed_);
                std::printf("%6s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "G", "pstk", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.0f %6.2f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.loads.nzcgs,
                input.pstick, input.rstick, digiModeName(mode));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered an offensive mode.
        if (!enteredOffensive_) return false;
        // 2. Must close range on the target. Pure chase closure at
        //    (speed - targetSpeed) over 60s; the brain also accelerates in
        //    pursuit. Require at least 1.5 NM closure (heavy/slow: waived).
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * kNmToFt;
            if (maxRange_ - minRange_ < requiredClosureFt) return false;
        }
        // 3. Must maneuver: heading change > 15° OR G > 2 (heavy: 1.1).
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const double hdgThreshold = 15.0 * DTR;
        if (maxHeadingChange_ < hdgThreshold && maxG_ < gThreshold) return false;
        // 4. Must not lawn-dart.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter offensive mode (BVR/WVR/Guns/Missile/Merge); "
               "Close range by >= 1.5NM (heavy/slow: waived); "
               "Maneuver (heading change > 15deg OR G > 2.0); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredOffensive_)
            return "Never entered an offensive mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   "; final range " + std::to_string(static_cast<int>(curRange_)) +
                   "ft — target was not classified as a threat).";
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * kNmToFt;
            if (maxRange_ - minRange_ < requiredClosureFt)
                return "Range closure was " +
                       std::to_string((maxRange_ - minRange_) / kNmToFt) +
                       "NM (need >= 1.5NM) — did not close on target.";
        }
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        if (maxHeadingChange_ < 15.0 * DTR && maxG_ < gThreshold)
            return "No maneuver: max heading change " +
                   std::to_string(maxHeadingChange_ * RTD) +
                   "deg (need > 15) AND max G " + std::to_string(maxG_) +
                   " (need > " + std::to_string(gThreshold) + ").";
        if (minAlt_ < 5000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 5000ft) — descended below floor.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",    curRange_,  "ft"},
            {"G",        curG_,      ""},
            {"hdg_chg",  curHdgChg_, "deg"},
            {"in_off",   enteredOffensive_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        std::printf("  --- Intercept Summary ---\n");
        std::printf("  Entered offensive mode: %s (BVR=%d WVR=%d)\n",
            enteredOffensive_ ? "[PASS]" : "[FAIL]",
            enteredBVR_, enteredWVR_);
        if (canCloseRange) {
            std::printf("  Range closure: %.2f NM (need >= 1.5) %s\n",
                (maxRange_ - minRange_) / kNmToFt,
                (maxRange_ - minRange_) >= 1.5 * kNmToFt ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Range closure: %.2f NM (heavy/slow: waived)\n",
                (maxRange_ - minRange_) / kNmToFt);
        }
        std::printf("  Max heading change: %.1f deg (need > 15 OR G>2) %s\n",
            maxHeadingChange_ * RTD,
            (maxHeadingChange_ >= 15.0 * DTR || maxG_ >= gThreshold) ? "[PASS]" : "[FAIL]");
        std::printf("  Max G: %.2f (need > %.1f OR hdg>15) %s\n",
            maxG_, gThreshold,
            (maxHeadingChange_ >= 15.0 * DTR || maxG_ >= gThreshold) ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude: %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeNm_, offsetFt_;
    double initialHeading_{0.0};
    DigiEntity target_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxHeadingChange_{0.0};
    bool enteredOffensive_{false};
    bool enteredBVR_{false};
    bool enteredWVR_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    double curRange_{0.0};
    double curG_{0.0};
    double curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 4: RTB and land
//
// After the intercept, the target is cleared and the aircraft is commanded
// to RTB. We reposition the aircraft 25 NM north of the origin at mission
// altitude, heading north (away from the airbase). Fuel is set below bingo
// and a friendly airbase is provided at the origin. The brain should enter
// RTB mode, turn toward the airbase (south), and close the distance.
//
// A full landing is not required (the landing approach is covered by
// digi_groundops); we verify the RTB mode latches and the aircraft heads
// back toward the divert field.
// ===========================================================================
class E2ERTBPhase : public ManeuverTest {
public:
    E2ERTBPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 25 NM north of the origin, heading north (away from base).
        const double startY = 25.0 * kNmToFt;
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

        // Friendly airbase at the origin (sea level), runway heading north.
        airbase_.x = 0.0;
        airbase_.y = 0.0;
        airbase_.z = 0.0;
        airbase_.runwayHeading = kRwyHeading;
        airbase_.id = 270;

        // Bingo fuel: fuelLbs below bingoFuelLbs triggers FuelCheck → RTB.
        FrameInputs fi;
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);

        // Clear any residual offensive target from the previous phase.
        sc.brain().clearTarget();

        initialDist_ = startY;  // aircraft at (0, 25NM), airbase at (0, 0)

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Re-apply fuel + airbase state each frame so FuelCheck re-evaluates
        // and the brain keeps the divert field (defensive, like digi_separate).
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedTarget = nullptr;  // ensure no stale target
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::RTB)       enteredRTB_ = true;
        if (mode == DigiMode::Landing)   enteredLanding_ = true;

        // Distance to airbase (2D).
        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);

        // Heading convergence to south (-PI/2): the airbase is south of the
        // aircraft (bearing = -PI/2). Track how close the heading gets.
        double dh = as.kin.sigma - (-PI / 2.0);
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToSouth_ = std::min(minAbsHeadingToSouth_, std::fabs(dh));

        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = dist;
        curHdgErr_ = std::fabs(dh) * RTD;
        curFuelLbs_ = sc_brain_->state().fuel.fuelLbs;
        curMode_ = mode;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (bingo fuel, airbase 25NM south at origin)\n",
                    testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "dAB(ft)", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %8.0f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, as.kin.sigma * RTD, dist,
                as.loads.nzcgs, input.rstick, digiModeName(mode));
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter RTB mode (bingo fuel → FuelCheck → AirbaseCheck → RTB).
        if (!enteredRTB_) return false;
        // 2. Must turn toward the airbase (south). Started heading north,
        //    needs to reverse to south (180° turn). Allow 90° tolerance.
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToSouth_ > hdgThreshold * DTR) return false;
        // 3. Must close the distance to the airbase (prove it's flying back).
        //    Heavy/slow aircraft get a 1 NM threshold.
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt) return false;
        // 4. Must not crash.
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB mode; Turn within 90deg of airbase bearing (120deg heavy); "
               "Close distance by >= 3NM (1NM heavy); Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRTB_)
            return "Never entered RTB mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   "; fuel " + std::to_string(static_cast<int>(curFuelLbs_)) +
                   "lbs — FuelCheck did not declare Bingo).";
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToSouth_ > hdgThreshold * DTR)
            return "Heading error to airbase bearing was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= " + std::to_string(hdgThreshold) +
                   "deg) — did not turn toward the divert field.";
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt)
            return "Distance closure was " +
                   std::to_string((initialDist_ - minDist_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 1.0 : 3.0) +
                   "NM) — did not close on the airbase.";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 1000ft) — descended below safe floor.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"dist_ab",    curDist_,    "ft"},
            {"hdg_err",    curHdgErr_,  "deg"},
            {"fuel",       curFuelLbs_, "lb"},
            {"in_rtb",     (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
            {"in_landing", (curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        std::printf("  --- RTB Summary ---\n");
        std::printf("  Entered RTB:          %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading to south: %.1f deg (need <= %.0f) %s\n",
            minAbsHeadingToSouth_ * RTD, hdgThreshold,
            minAbsHeadingToSouth_ <= hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to airbase:  %.0f ft (closure %.2f NM, need >= %.1f) %s\n",
            minDist_, (initialDist_ - minDist_) / kNmToFt, isHeavy_ ? 1.0 : 3.0,
            (initialDist_ - minDist_) >= requiredClosureFt ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Landing:      %s\n", enteredLanding_ ? "[PASS]" : "(n/a)");
        std::printf("  Min altitude:         %.0f ft (need >= 1000) %s\n",
            minAlt_, minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    FrameInputs::AirbaseInfo airbase_;
    double nextPrint_{0.0};
    double initialDist_{0.0};
    double minDist_{1e9};
    double minAlt_{1e9};
    double minAbsHeadingToSouth_{1e9};
    bool enteredRTB_{false};
    bool enteredLanding_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    DigiBrain* sc_brain_{nullptr};

    double curDist_{0.0};
    double curHdgErr_{0.0};
    double curFuelLbs_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// DigiE2EMissionScenario
// ===========================================================================
class DigiE2EMissionScenario : public ManeuverScenario {
public:
    DigiE2EMissionScenario() : ManeuverScenario("digi_e2e_mission") {}

    std::string GetDescription() const override {
        return "End-to-end digi AI mission: takeoff -> navigate to 20NM IP -> "
               "intercept an evading target (BVR/WVR) -> RTB to origin airbase. "
               "Tests the full mode pipeline (Takeoff -> Waypoint -> BVREngage "
               "-> RTB) in mission order.";
    }

    // Draw the home runway at the origin (north-south) so the visualization
    // shows where takeoff and RTB happen.
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

        fm.init(ctx.cfg, 0.0, 0.0, kRwyHeading, false);

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double targetSpeed = 250.0;  // slower evader

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: Takeoff (30s — fighters take off in <20s; heavies get
        // an 80-kts acceleration check).
        tests.push_back(std::make_unique<E2ETakeoffPhase>("Takeoff", 30.0));
        // Phase 2: Navigate to 20 NM IP at 15000 ft (90s).
        tests.push_back(std::make_unique<E2ENavigatePhase>("Navigate to IP", 90.0));
        // Phase 3: Intercept evading target 10 NM ahead, 250 kts, 0.5 NM
        // east offset (60s).
        tests.push_back(std::make_unique<E2EInterceptPhase>(
            "Intercept evading target", 60.0,
            kMissionAlt, cornerSpeed, targetSpeed, 10.0, 3000.0));
        // Phase 4: RTB to origin airbase, bingo fuel (90s).
        tests.push_back(std::make_unique<E2ERTBPhase>(
            "RTB to origin", 90.0, kMissionAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerDigiE2EMission("digi_e2e_mission", []() {
    return std::make_unique<DigiE2EMissionScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_e2e_mission() {}

} // namespace f4flight_test
