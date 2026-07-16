// f4flight - scenarios/scenario_digi_e2e_ground_attack.cpp
//
// End-to-end multi-phase ground-attack mission scenario for the digi AI.
//
// This scenario chains four phases of a single A/G strike mission to exercise
// the full mode pipeline of the DigiBrain in mission order:
//
//   Phase 1 "Takeoff"        : ground start -> Takeoff mode -> climb out
//   Phase 2 "Navigate"       : climb + waypoint following to the target area
//   Phase 3 "Dive-bomb"      : inject a ground target -> GroundMnvr dive-bomb
//   Phase 4 "RTB"            : bingo fuel + airbase -> RTB -> divert
//
// The framework runs the phases in order, calling Init() for each. Each
// phase's Init() re-initializes the FlightModel to a deterministic starting
// condition (the task description explicitly allows repositioning via
// fm.init() so the test does not depend on the exact end-state of the
// previous phase). The DigiBrain state (mode, config, frame inputs) is
// cleared / reconfigured by each Init() as needed.
//
// Pass criteria are intentionally relaxed compared to the per-mode scenarios
// (digi_groundops, digi_ground_attack, digi_rtb) -- the goal here is to
// verify the brain enters the RIGHT MODE for each mission segment and makes
// meaningful progress, not to re-verify the per-mode tolerances.
//
// Task ID: 16-a

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
constexpr double kMissionAlt   = 15000.0;    // ft MSL
constexpr double kNmToFt       = 6076.0;

// ===========================================================================
// Phase 1: Takeoff
//
// Aircraft starts on the runway at the origin, heading north. The brain is
// commanded to take off. We verify the Takeoff mode latches, the throttle
// advances, and the aircraft rotates and climbs out.
// ===========================================================================
class E2EGATakeoffPhase : public ManeuverTest {
public:
    E2EGATakeoffPhase(const char* name, double duration)
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
        if (maxAlt_   < 100.0)        return false;
        if (maxSpeed_ < 200.0)        return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Takeoff mode; Apply takeoff throttle; "
               "Fighter: airborne + alt >= 100ft + speed >= 200kts; "
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
        if (maxAlt_ < 100.0)
            return "Max altitude " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= 100).";
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
            std::printf("  Max altitude:    %.0f ft (need >= 100) %s\n",
                maxAlt_, maxAlt_ >= 100.0 ? "[PASS]" : "[FAIL]");
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
// Phase 2: Navigate to target area
//
// After takeoff, the aircraft climbs and navigates toward the target area at
// the origin. We re-init the aircraft at 15000 ft, 6 NM south of the target,
// heading north. A single waypoint is set at the target area (origin) at
// mission altitude.
//
// Verify the brain:
//   - Exits Takeoff mode and navigates (Waypoint mode or similar)
//   - Maintains / reaches mission altitude (10000+ ft)
//   - Holds a north heading (within 25 deg of north)
//   - Closes the range to the target area to within 5 NM
// ===========================================================================
class E2EGANavigatePhase : public ManeuverTest {
public:
    E2EGANavigatePhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at mission altitude, 6 NM south of the target, heading north.
        const double startSpeed = 350.0;
        const double startY = -6.0 * kNmToFt;  // 6 NM south of target
        fm.init(fm.config(), kMissionAlt, startSpeed * KNOTS_TO_FTPSEC,
                kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = startY;
        fm.state().kin.z = -kMissionAlt;

        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setAltitude(kMissionAlt);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Waypoint at the target area (origin) at mission altitude.
        std::vector<Vec3> wps;
        wps.push_back({0.0, 0.0, -kMissionAlt});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        // Exit any residual Takeoff / Landing ground-ops state from the
        // previous phase so the brain resolves to Waypoint navigation.
        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        wpX_ = 0.0;
        wpY_ = 0.0;
        wpZ_ = -kMissionAlt;
        initialRange_ = std::sqrt((wpX_ - 0.0) * (wpX_ - 0.0) +
                                  (wpY_ - startY) * (wpY_ - startY));

        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double alt = -as.kin.z;
        maxAlt_ = std::max(maxAlt_, alt);

        // Range to target area (2D, north-east).
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
            mode == DigiMode::BVREngage || mode == DigiMode::WVREngage ||
            mode == DigiMode::GroundMnvr)
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
                std::printf("\n%s (navigate 6NM south -> target area at origin)\n",
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
        // 2. Must maintain / reach mission altitude (10000+ ft).
        //    Heavy aircraft get a relaxed floor.
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        if (maxAlt_ < altThreshold) return false;
        // 3. Must hold a roughly north heading (toward the target).
        if (minAbsHeadingErr_ > 25.0 * DTR) return false;
        // 4. Must close to within 5 NM of the target area.
        //    Started 6 NM south; need to close to <= 5 NM (1 NM closure).
        //    Heavy: within 5.5 NM (0.5 NM closure).
        const double maxRangeThreshold = (isHeavy_ ? 5.5 : 5.0) * kNmToFt;
        if (minRange_ > maxRangeThreshold) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter nav mode (not stuck in Takeoff); Alt >= 10000ft "
               "(6000ft heavy); Heading within 25deg of north; "
               "Close to within 5NM of target area; No NaN";
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
                   "ft) -- did not maintain mission altitude.";
        if (minAbsHeadingErr_ > 25.0 * DTR)
            return "Heading error to north was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= 25deg) -- did not hold north heading.";
        const double maxRangeThreshold = (isHeavy_ ? 5.5 : 5.0) * kNmToFt;
        if (minRange_ > maxRangeThreshold)
            return "Min range to target was " +
                   std::to_string(minRange_ / kNmToFt) +
                   "NM (need <= " +
                   std::to_string(isHeavy_ ? 5.5 : 5.0) +
                   "NM) -- did not close on target area.";
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

    // Publish the target area as a "slot" entity so it shows in the report.
    std::vector<ThreatEntity> traceEntities() const override {
        return {{"slot", wpX_, wpY_, wpZ_, 0.0}};
    }

    void Finish() const override {
        const double altThreshold = isHeavy_ ? 6000.0 : 10000.0;
        const double maxRangeThreshold = (isHeavy_ ? 5.5 : 5.0) * kNmToFt;
        std::printf("  --- Navigate Summary ---\n");
        std::printf("  Entered nav mode:   %s\n", enteredNavMode_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:       %.0f ft (need >= %.0f) %s\n",
            maxAlt_, altThreshold, maxAlt_ >= altThreshold ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading err:    %.1f deg (need <= 25) %s\n",
            minAbsHeadingErr_ * RTD,
            minAbsHeadingErr_ <= 25.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min range to tgt:   %.2f NM (need <= %.1f) %s\n",
            minRange_ / kNmToFt, isHeavy_ ? 5.5 : 5.0,
            minRange_ <= maxRangeThreshold ? "[PASS]" : "[FAIL]");
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
// Phase 3: Dive-bomb attack on ground target
//
// The aircraft is repositioned at mission altitude, 6 NM south of the target
// area, heading north. A ground target (DigiEntity) is injected at the origin
// (0, 0, 0). The brain should detect the target, enter GroundMnvr mode,
// execute the dive-bomb profile (approach -> dive -> release -> pullout ->
// egress), release the weapon, and pull out without crashing.
//
// We re-inject the ground target each frame in Evaluate() to keep the
// state_.ag.groundTarget pointer fresh (defensive, matches the per-frame
// FrameInputs pattern used by digi_rtb / digi_separate for fuel state).
// ===========================================================================
class E2EGADiveBombPhase : public ManeuverTest {
public:
    E2EGADiveBombPhase(const char* name, double duration,
                       double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at mission altitude, 6 NM south of the target, heading north.
        const double startY = -6.0 * kNmToFt;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = startY;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);   // allow steep banks for attack maneuvers
        sc.setMaxGamma(45.0);  // allow steep dive/climb for dive-bomb

        // --- Set up the ground target at the origin (ground level) ---
        groundTarget_.x = 0.0;
        groundTarget_.y = 0.0;
        groundTarget_.z = 0.0;
        groundTarget_.vx = 0.0;
        groundTarget_.vy = 0.0;
        groundTarget_.vz = 0.0;
        groundTarget_.yaw = 0.0;
        groundTarget_.pitch = 0.0;
        groundTarget_.roll = 0.0;
        groundTarget_.speed = 0.0;
        groundTarget_.isDead = false;
        groundTarget_.dcm = dcmFromEuler(0.0, 0.0, 0.0);

        // Inject the ground target (DiveBomb profile is the default).
        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
        sc.brain().setFrameInputs(fi);

        // Clear any residual offensive target from a previous phase.
        sc.brain().clearTarget();

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Re-inject the ground target each frame (defensive; keeps the
        // pointer fresh and matches the per-frame FrameInputs pattern).
        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
        sc_brain_->setFrameInputs(fi);

        // Track distance to target.
        const double dx = groundTarget_.x - as.kin.x;
        const double dy = groundTarget_.y - as.kin.y;
        const double distToTarget = std::sqrt(dx * dx + dy * dy);
        minDistToTarget_ = std::min(minDistToTarget_, distToTarget);

        // Track altitude (AGL, ground at z=0).
        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);

        // Track mode entry.
        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::GroundMnvr) enteredGroundMnvr_ = true;

        // Track weapon release.
        if (input.releaseConsent) {
            weaponReleased_ = true;
            releaseAlt_ = altAGL;
            releaseDist_ = distToTarget;
        }

        // Track egress (distance increasing after release).
        if (weaponReleased_ && !startedEgress_) {
            prevDist_ = distToTarget;
            startedEgress_ = true;
        } else if (weaponReleased_ && distToTarget > prevDist_) {
            egressing_ = true;
        }
        prevDist_ = distToTarget;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        // Per-frame sample data.
        curAlt_ = altAGL;
        curDist_ = distToTarget;
        curMode_ = mode;
        curPhase_ = sc_brain_->state().ag.agApproach;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (dive-bomb on ground target at origin, "
                            "start 6NM south at %.0fft)\n",
                    testName_.c_str(), alt_);
                std::printf("%6s %8s %8s %6s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "dTgt", "vcas", "pstk", "thrt", "mode", "phase");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %6.1f %6.2f %6.2f %6s %6d\n",
                phaseTime_, altAGL, distToTarget, as.vcas,
                input.pstick, input.throttle, modeBuf,
                sc_brain_->state().ag.agApproach);
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter GroundMnvr mode.
        if (!enteredGroundMnvr_) return false;
        // 2. Must release the weapon.
        if (!weaponReleased_) return false;
        // 3. Must not crash (min altitude > 500 ft AGL).
        if (minAlt_ < 500.0) return false;
        // 4. Must start egressing (distance increasing after release).
        if (!egressing_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GroundMnvr; Release weapon; "
               "Min alt > 500ft (no crash); Egress after release; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_) {
            return "Never entered GroundMnvr mode (final mode: " +
                   std::string(digiModeName(curMode_)) + ").";
        }
        if (!weaponReleased_) {
            return "Never released weapon -- dive profile did not reach "
                   "release altitude (min alt " +
                   std::to_string(static_cast<int>(minAlt_)) + "ft, "
                   "min dist " + std::to_string(static_cast<int>(minDistToTarget_)) +
                   "ft).";
        }
        if (minAlt_ < 500.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed > 500ft) -- aircraft crashed during pullout.";
        }
        if (!egressing_) {
            return "Never started egressing -- aircraft did not fly away "
                   "after release.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",        curAlt_,   "ft"},
            {"d_target",   curDist_,  "ft"},
            {"in_ground",  (enteredGroundMnvr_ && curMode_ == DigiMode::GroundMnvr) ? 1.0 : 0.0, ""},
            {"released",   weaponReleased_ ? 1.0 : 0.0, ""},
            {"ag_phase",   static_cast<double>(curPhase_), ""},
        };
    }

    // Publish the ground target as a "target" entity so it shows in the report.
    std::vector<ThreatEntity> traceEntities() const override {
        return {{"target", groundTarget_.x, groundTarget_.y, groundTarget_.z, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- Dive-bomb Summary ---\n");
        std::printf("  Entered GroundMnvr:  %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:     %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        if (weaponReleased_) {
            std::printf("  Release altitude:    %.0f ft\n", releaseAlt_);
            std::printf("  Release distance:    %.0f ft\n", releaseDist_);
        }
        std::printf("  Min altitude:        %.0f ft (need > 500) %s\n",
            minAlt_, minAlt_ >= 500.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Egressing:           %s\n", egressing_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity groundTarget_;
    DigiBrain* sc_brain_{nullptr};
    double nextPrint_{0.0};

    double minDistToTarget_{1e9};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double releaseAlt_{0.0};
    double releaseDist_{0.0};
    double prevDist_{0.0};
    bool enteredGroundMnvr_{false};
    bool weaponReleased_{false};
    bool startedEgress_{false};
    bool egressing_{false};
    bool hasNaN_{false};

    double curAlt_{0.0};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    int curPhase_{0};
};

// ===========================================================================
// Phase 4: RTB and land
//
// After the attack, the ground target is cleared and the aircraft is
// commanded to RTB. We reposition the aircraft 25 NM north of the origin at
// mission altitude, heading north (away from the airbase). Fuel is set below
// bingo and a friendly airbase is provided at the origin. The brain should
// enter RTB mode, turn toward the airbase (south), and close the distance.
//
// A full landing is not required (the landing approach is covered by
// digi_groundops); we verify the RTB mode latches and the aircraft heads
// back toward the divert field.
// ===========================================================================
class E2EGARTBPhase : public ManeuverTest {
public:
    E2EGARTBPhase(const char* name, double duration, double alt, double speed)
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

        // Bingo fuel: fuelLbs below bingoFuelLbs triggers FuelCheck -> RTB.
        FrameInputs fi;
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedGroundTarget = nullptr;  // clear ground target from Phase 3
        fi.injectedTarget = nullptr;        // clear any offensive target
        sc.brain().setFrameInputs(fi);

        // Explicitly clear any residual A/G attack state.
        sc.brain().stateMutable().ag.groundTarget = nullptr;
        sc.brain().stateMutable().ag.agApproach = 0;
        sc.brain().stateMutable().ag.reachedIP = false;

        // Clear any residual offensive target from the previous phase.
        sc.brain().clearTarget();

        initialDist_ = startY;  // aircraft at (0, 25NM), airbase at (0, 0)

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Re-apply fuel + airbase state each frame so FuelCheck re-evaluates
        // and the brain keeps the divert field (defensive, like digi_rtb).
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedGroundTarget = nullptr;  // ensure no stale ground target
        fi.injectedTarget = nullptr;        // ensure no stale offensive target
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
        // 1. Must enter RTB mode (bingo fuel -> FuelCheck -> AirbaseCheck -> RTB).
        if (!enteredRTB_) return false;
        // 2. Must turn toward the airbase (south). Started heading north,
        //    needs to reverse to south (180 deg turn). Allow 90 deg tolerance.
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
                   "lbs -- FuelCheck did not declare Bingo).";
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToSouth_ > hdgThreshold * DTR)
            return "Heading error to airbase bearing was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= " + std::to_string(hdgThreshold) +
                   "deg) -- did not turn toward the divert field.";
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt)
            return "Distance closure was " +
                   std::to_string((initialDist_ - minDist_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 1.0 : 3.0) +
                   "NM) -- did not close on the airbase.";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (need >= 1000ft) -- descended below safe floor.";
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
// DigiE2EGroundAttackScenario
// ===========================================================================
class DigiE2EGroundAttackScenario : public ManeuverScenario {
public:
    DigiE2EGroundAttackScenario() : ManeuverScenario("digi_e2e_ground_attack") {}

    std::string GetDescription() const override {
        return "End-to-end digi AI ground-attack mission: takeoff -> navigate "
               "to target area -> dive-bomb attack on ground target -> RTB to "
               "origin airbase. Tests the full mode pipeline (Takeoff -> "
               "Waypoint -> GroundMnvr -> RTB) in mission order.";
    }

    // Draw the home runway at the origin (north-south) so the visualization
    // shows where takeoff and RTB happen.
    std::vector<SceneLine> sceneGeometry() const override {
        std::vector<SceneLine> lines;
        const double rwLen = 10000.0;
        const double rwHalf = rwLen / 2.0;
        SceneLine centerline;
        centerline.label = "RWY";
        centerline.x1 = 0.0; centerline.y1 = -rwHalf; centerline.z1 = 0.0;
        centerline.x2 = 0.0; centerline.y2 =  rwHalf; centerline.z2 = 0.0;
        centerline.color = "#3a3a4a";
        centerline.width = 150.0;
        lines.push_back(centerline);
        return lines;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        fm.init(ctx.cfg, 0.0, 0.0, kRwyHeading, false);

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: Takeoff (30s -- fighters take off in <20s; heavies get
        // an 80-kts acceleration check).
        tests.push_back(std::make_unique<E2EGATakeoffPhase>("Takeoff", 30.0));
        // Phase 2: Navigate to target area 6 NM ahead (90s).
        tests.push_back(std::make_unique<E2EGANavigatePhase>(
            "Navigate to target area", 90.0));
        // Phase 3: Dive-bomb attack on ground target at origin (60s).
        tests.push_back(std::make_unique<E2EGADiveBombPhase>(
            "Dive-bomb ground target", 60.0,
            kMissionAlt, cornerSpeed));
        // Phase 4: RTB to origin airbase, bingo fuel (90s).
        tests.push_back(std::make_unique<E2EGARTBPhase>(
            "RTB to origin", 90.0, kMissionAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerDigiE2EGroundAttack("digi_e2e_ground_attack", []() {
    return std::make_unique<DigiE2EGroundAttackScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_e2e_ground_attack() {}

} // namespace f4flight_test
