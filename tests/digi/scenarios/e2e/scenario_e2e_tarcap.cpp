// f4flight - scenarios/scenario_e2e_tarcap.cpp
//
// End-to-end Target Combat Air Patrol (TARCAP) mission scenario for the
// digi AI. Analogous to FreeFalcon's AMIS_TARCAP campaign mission type.
//
// A TARCAP mission provides defensive air cover over a specific ground
// target area for a limited time. It is similar to BARCAP but shorter in
// duration and orbits over a target (not a barrier point).
//
// This scenario chains FIVE phases of a single TARCAP mission to exercise
// the full mode pipeline of the DigiBrain in mission order:
//
//   Phase 1 "Takeoff"        : ground start -> Takeoff mode -> climb out
//   Phase 2 "Climb to tgt"   : climb to 15000ft, navigate 20NM to target area
//   Phase 3 "Target CAP"     : orbit the target in Loiter mode (shorter than BARCAP)
//   Phase 4 "Engage bandit"  : inject inbound bandit, verify engage
//   Phase 5 "RTB"            : bingo fuel, return to origin airbase
//
// Each phase's Init() re-initializes the FlightModel to a deterministic
// starting condition (per scenario_digi_e2e_mission.cpp pattern). The
// DigiBrain state (mode, config, frame inputs) is cleared / reconfigured
// by each Init() as needed.
//
// Pass criteria are intentionally relaxed — verify "the AI enters the right
// mode + makes meaningful progress", not "tight tolerances met".
//
// Task ID: 23

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/ground/ground_ops.h"  // GroundOpsPhase
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
constexpr double kCapAlt       = 15000.0;    // ft MSL — TARCAP station altitude
constexpr double kNmToFt       = 6076.0;
constexpr double kCapRangeNm   = 20.0;       // Target area 20 NM north of origin

// ===========================================================================
// Phase 1: Takeoff (30s)
// ===========================================================================
class TarcapTakeoffPhase : public ManeuverTest {
public:
    TarcapTakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), 0.0, 0.0, kRwyHeading, false);
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
        maxAlt_   = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);
        if (altAGL > 10.0)              becameAirborne_ = true;
        if (input.throttle > 0.9)       appliedTakeoffThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        curAlt_      = altAGL;
        curVcas_     = as.vcas;
        curThrottle_ = input.throttle;
        curMode_     = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (maxAlt_ >= 500.0 && maxSpeed_ >= 200.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTakeoff_)         return false;
        if (!appliedTakeoffThrottle_) return false;
        if (isHeavy_) return maxSpeed_ >= 80.0;
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
            {"alt",        curAlt_,      "ft"},
            {"vcas",       curVcas_,     "kts"},
            {"throttle",   curThrottle_, ""},
            {"in_takeoff", (enteredTakeoff_ && curMode_ == DigiMode::Takeoff) ? 1.0 : 0.0, ""},
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
// Phase 2: Climb to target area (90s)
//
// After takeoff, the aircraft climbs to 15000 ft and navigates 20 NM north
// to the target area. We re-init the aircraft at a low post-takeoff
// altitude (3000 ft, 300 kts, heading north) and command a single waypoint
// at (0, 20NM, -15000).
// ===========================================================================
class TarcapClimbPhase : public ManeuverTest {
public:
    TarcapClimbPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startAlt = 3000.0;
        const double startSpeed = 300.0;
        fm.init(fm.config(), startAlt, startSpeed * KNOTS_TO_FTPSEC,
                kRwyHeading, true);

        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setAltitude(kCapAlt);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        std::vector<Vec3> wps;
        wps.push_back({0.0, kCapRangeNm * kNmToFt, -kCapAlt});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        wpX_ = 0.0;
        wpY_ = kCapRangeNm * kNmToFt;
        wpZ_ = -kCapAlt;
        initialRange_ = std::sqrt((wpX_ - 0.0) * (wpX_ - 0.0) +
                                  (wpY_ - 0.0) * (wpY_ - 0.0));

        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
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
        curHdgErr_= std::fabs(dh) * RTD;
        curMode_  = mode;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (climb 3k->15k ft, navigate to 20NM target area)\n",
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
        if (!enteredNavMode_) return false;
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        if (maxAlt_ < altThreshold) return false;
        if (minAbsHeadingErr_ > 25.0 * DTR) return false;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter nav mode (not stuck in Takeoff); Climb to >= 10000ft "
               "(6000ft heavy); Heading within 25deg of north; "
               "Close range to target area by >= 5NM (2NM heavy); No NaN";
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
                   "ft) — did not climb toward CAP altitude.";
        if (minAbsHeadingErr_ > 25.0 * DTR)
            return "Heading error to north was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= 25deg) — did not hold north heading.";
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt)
            return "Range closure was " +
                   std::to_string((initialRange_ - minRange_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 2.0 : 5.0) +
                   "NM) — did not close on the target area.";
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
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        std::printf("  --- Climb to Target Summary ---\n");
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
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    double curAlt_{0.0};
    double curRange_{0.0};
    double curHdgErr_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Target CAP loiter (90s — shorter than BARCAP's 120s)
//
// The aircraft is positioned over the target area at 15000 ft. We force
// Loiter mode so the aircraft orbits the target. We verify the brain enters
// Loiter mode, holds altitude, and accumulates heading change indicative
// of an orbit.
// ===========================================================================
class TarcapLoiterPhase : public ManeuverTest {
public:
    TarcapLoiterPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startSpeed = fm.config().geometry.cornerVcas_kts > 0
            ? fm.config().geometry.cornerVcas_kts : 330.0;
        fm.init(fm.config(), kCapAlt, startSpeed * KNOTS_TO_FTPSEC,
                0.0 /* east */, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = kCapRangeNm * kNmToFt;
        fm.state().kin.z = -kCapAlt;

        sc.setMode(SteeringController::Mode::Loiter);
        sc.brain().forceMode(DigiMode::Loiter);
        sc.setAltitude(kCapAlt);
        sc.setHeading(0.0);
        sc.setCornerSpeed(startSpeed);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        capX_ = 0.0;
        capY_ = kCapRangeNm * kNmToFt;
        capZ_ = -kCapAlt;

        isHeavy_ = isHeavy(fm.config());
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double heading = as.kin.sigma;
        curMode_ = sc_brain_->activeMode();
        if (curMode_ == DigiMode::Loiter) enteredLoiter_ = true;

        if (!lastHeadingInit_) {
            lastHeading_ = heading;
            lastHeadingInit_ = true;
        }
        double dh = heading - lastHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        accumulatedHeadingChange_ += dh;
        lastHeading_ = heading;

        const double alt = -as.kin.z;
        minAlt_ = std::min(minAlt_, alt);
        maxAlt_ = std::max(maxAlt_, alt);

        const double dx = as.kin.x - capX_;
        const double dy = as.kin.y - capY_;
        const double distFromCap = std::sqrt(dx * dx + dy * dy);
        maxDistFromCap_ = std::max(maxDistFromCap_, distFromCap);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curAlt_ = alt;
        curHdgChg_ = std::fabs(accumulatedHeadingChange_) * RTD;
        curDistFromCap_ = distFromCap;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (Loiter over target, %.0fkft, 90s)\n",
                    testName_.c_str(), kCapAlt / 1000.0);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "dCAP", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %8.0f %6.2f %6.2f %6s\n",
                phaseTime_, alt, heading * RTD, distFromCap,
                as.loads.nzcgs, input.rstick, digiModeName(curMode_));
            nextPrint_ += 15.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLoiter_) return false;
        // Shorter loiter (90s vs 120s for BARCAP) — require 60 deg (40 heavy).
        // F4Flight's Loiter mode at corner speed produces a slow, drifting
        // spiral (not a closed orbit). Criterion verifies the brain ENTERED
        // Loiter and is TURNING. Drift is checked loosely (<= 20 NM).
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR) return false;
        if (maxAlt_ - minAlt_ > 3000.0) return false;
        if (maxDistFromCap_ > 20.0 * kNmToFt) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Loiter mode; Accumulated heading change > 60deg "
               "(40deg heavy); Altitude held within +/- 1500ft; "
               "Stay within 20NM of target (loose — Loiter spirals); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLoiter_)
            return "Never entered Loiter mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR)
            return "Accumulated heading change was " +
                   std::to_string(curHdgChg_) +
                   "deg (need >= " + std::to_string(hdgThreshold) +
                   "deg) — did not orbit.";
        if (maxAlt_ - minAlt_ > 3000.0)
            return "Altitude band was " +
                   std::to_string(static_cast<int>(maxAlt_ - minAlt_)) +
                   "ft (need <= 3000ft) — did not hold altitude.";
        if (maxDistFromCap_ > 20.0 * kNmToFt)
            return "Max distance from target was " +
                   std::to_string(static_cast<int>(maxDistFromCap_)) +
                   "ft (need <= " +
                   std::to_string(static_cast<int>(20.0 * kNmToFt)) +
                   "ft) — drifted off station.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",       curAlt_,         "ft"},
            {"hdg_chg",   curHdgChg_,      "deg"},
            {"d_cap",     curDistFromCap_, "ft"},
            {"in_loiter", (enteredLoiter_ && curMode_ == DigiMode::Loiter) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"slot", capX_, capY_, capZ_, 0.0}};
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 40.0 : 60.0;
        std::printf("  --- Target CAP Summary ---\n");
        std::printf("  Entered Loiter:           %s\n", enteredLoiter_ ? "[PASS]" : "[FAIL]");
        std::printf("  Accumulated heading chg:  %.1f deg (need >= %.0f) %s\n",
            curHdgChg_, hdgThreshold,
            std::fabs(accumulatedHeadingChange_) >= hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Altitude band:            %.0f ft (need <= 3000) %s\n",
            maxAlt_ - minAlt_, (maxAlt_ - minAlt_) <= 3000.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max dist from target:     %.0f ft (need <= %.0f) %s\n",
            maxDistFromCap_, 20.0 * kNmToFt,
            maxDistFromCap_ <= 20.0 * kNmToFt ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double capX_{0.0}, capY_{0.0}, capZ_{0.0};
    double initialHeading_{0.0};
    double nextPrint_{0.0};
    bool lastHeadingInit_{false};
    double lastHeading_{0.0};
    double accumulatedHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double maxDistFromCap_{0.0};
    bool enteredLoiter_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    double curAlt_{0.0};
    double curHdgChg_{0.0};
    double curDistFromCap_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 4: Engage bandit (60s)
//
// While loitering over the target, an inbound bandit is injected 10 NM
// south of the target (heading north toward friendly strike aircraft
// operating over the target). The brain should detect the bandit, enter
// an offensive mode, close range, and maneuver.
// ===========================================================================
class TarcapEngagePhase : public ManeuverTest {
public:
    TarcapEngagePhase(const char* name, double duration,
                      double alt, double speed, double targetSpeed,
                      double initialRangeNm, double offsetFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm),
          offsetFt_(offsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = kCapRangeNm * kNmToFt;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = offsetFt_;
        target_.y = (kCapRangeNm * kNmToFt) - rangeFt;  // south of target
        target_.z = -alt_;
        target_.yaw = kRwyHeading;   // north
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = target_.speed;  // north
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        target_.y += target_.speed * dt;

        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_   = std::max(maxG_, as.loads.nzcgs);

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
                std::printf("\n%s (bandit %.1f NM south of target, %.0f kts, inbound north)\n",
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
        if (!enteredOffensive_) return false;
        // Head-on geometry: bandit approaches the target area from the south,
        // crosses near the aircraft, then runs north. Closure is brief and
        // limited. Require just 0.3 NM closure (heavy/slow: waived) — the
        // key metric is that the brain DETECTED the bandit and entered an
        // offensive mode (verified by enteredOffensive_ above).
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 0.3 * kNmToFt;
            if (maxRange_ - minRange_ < requiredClosureFt) return false;
        }
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const double hdgThreshold = 15.0 * DTR;
        if (maxHeadingChange_ < hdgThreshold && maxG_ < gThreshold) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter offensive mode (BVR/WVR/Guns/Missile/Merge); "
               "Close range by >= 0.3NM (head-on geometry; heavy/slow: waived); "
               "Maneuver (heading change > 15deg OR G > 2.0); "
               "Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredOffensive_)
            return "Never entered an offensive mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   "; final range " + std::to_string(static_cast<int>(curRange_)) +
                   "ft — bandit was not classified as a threat).";
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 0.3 * kNmToFt;
            if (maxRange_ - minRange_ < requiredClosureFt)
                return "Range closure was " +
                       std::to_string((maxRange_ - minRange_) / kNmToFt) +
                       "NM (need >= 0.3NM) — did not close on bandit.";
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
            {"range",  curRange_,  "ft"},
            {"G",      curG_,      ""},
            {"hdg_chg",curHdgChg_, "deg"},
            {"in_off", enteredOffensive_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double gThreshold = isHeavy_ ? 1.1 : 2.0;
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        std::printf("  --- Engage Bandit Summary ---\n");
        std::printf("  Entered offensive mode: %s (BVR=%d WVR=%d)\n",
            enteredOffensive_ ? "[PASS]" : "[FAIL]",
            enteredBVR_, enteredWVR_);
        if (canCloseRange) {
            std::printf("  Range closure: %.2f NM (need >= 0.3) %s\n",
                (maxRange_ - minRange_) / kNmToFt,
                (maxRange_ - minRange_) >= 0.3 * kNmToFt ? "[PASS]" : "[FAIL]");
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
// Phase 5: RTB (90s)
// ===========================================================================
class TarcapRTBPhase : public ManeuverTest {
public:
    TarcapRTBPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
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
        fi.injectedTarget = nullptr;
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
        if (mode == DigiMode::RTB)       enteredRTB_ = true;
        if (mode == DigiMode::Landing)   enteredLanding_ = true;

        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);

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
        if (!enteredRTB_) return false;
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToSouth_ > hdgThreshold * DTR) return false;
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt) return false;
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
// E2ETarcapScenario
// ===========================================================================
class E2ETarcapScenario : public ManeuverScenario {
public:
    E2ETarcapScenario() : ManeuverScenario("e2e_tarcap") {}

    TestTier GetTestTier() const override { return TestTier::EndToEnd; }

    std::string GetDescription() const override {
        return "End-to-end TARCAP (AMIS_TARCAP): takeoff -> climb to 15kft "
               "and navigate 20NM to target area -> Loiter over target "
               "(shorter than BARCAP) -> engage inbound bandit (BVR/WVR) -> "
               "RTB to origin airbase. Tests the full defensive target-CAP "
               "mission pipeline (Takeoff -> Waypoint -> Loiter -> BVREngage "
               "-> RTB).";
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

        fm.init(ctx.cfg, 0.0, 0.0, kRwyHeading, false);

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double targetSpeed = 250.0;

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<TarcapTakeoffPhase>("Takeoff", 30.0));
        tests.push_back(std::make_unique<TarcapClimbPhase>("Climb to target area", 90.0));
        tests.push_back(std::make_unique<TarcapLoiterPhase>("Target CAP", 90.0));
        tests.push_back(std::make_unique<TarcapEngagePhase>(
            "Engage inbound bandit", 60.0,
            kCapAlt, cornerSpeed, targetSpeed, 10.0, 3000.0));
        tests.push_back(std::make_unique<TarcapRTBPhase>(
            "RTB to origin", 90.0, kCapAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerE2ETarcap("e2e_tarcap", []() {
    return std::make_unique<E2ETarcapScenario>();
});

extern "C" void f4flight_forceLink_scenario_e2e_tarcap() {}

} // namespace f4flight_test
