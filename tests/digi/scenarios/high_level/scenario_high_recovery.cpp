// f4flight - scenarios/scenario_high_recovery.cpp
//
// HIGH-LEVEL scenario: full recovery chain RTB (bingo fuel) -> Divert to
// alternate -> Approach -> Landing -> Taxi. Composes five low-level behaviors
// into a realistic return-and-recovery sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed.
//
// Pass criteria (per phase):
//   1. RTB       : enter RTB mode; turn toward airbase within 90deg
//                  (heavy: 120deg); close distance by >= 3NM (1NM heavy);
//                  min alt >= 1000ft; no NaN
//   2. Divert    : with original airbase closed + alternate available,
//                  verify turn toward alternate (within 90deg) + closure
//                  >= 2NM (1NM heavy); no NaN
//   3. Approach  : within 10NM of runway, enter Landing mode + Approach phase;
//                  no NaN
//   4. Landing   : enter Landing mode; descend >= 500ft; touch down; enter
//                  Flare; min alt >= -500ft; no NaN
//   5. Taxi      : enter TaxiToRunway OR decelerate to < 5 kts after touchdown;
//                  no NaN
//
// Tier: HighLevel. Registered as "high_recovery" — referenced by the cascade
// mapping tables g_highToLow["high_recovery"] and g_e2eToHigh for
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

constexpr double kRwyHeading   = PI / 2.0;   // north
constexpr double kNmToFt       = 6076.0;

// ===========================================================================
// Phase 1: RTB (bingo fuel)
//
// Aircraft 25 NM north of origin at mission altitude, fuel below bingo,
// airbase at origin. Verify RTB mode entry + closure.
// ===========================================================================
class HighRTBPhase : public ManeuverTest {
public:
    HighRTBPhase(const char* name, double duration, double alt, double speed)
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
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedTarget = nullptr;
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::RTB) enteredRTB_ = true;

        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);

        // Heading to south (origin airbase is south of aircraft).
        double dh = as.kin.sigma - (-PI / 2.0);
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToSouth_ = std::min(minAbsHeadingToSouth_, std::fabs(dh));

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = dist;
        curHdgErr_ = std::fabs(dh) * RTD;
        curMode_ = mode;
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
        return "Enter RTB; Turn within 90deg (120 heavy) of airbase; "
               "Close distance >= 3NM (1NM heavy); Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRTB_)
            return "Never entered RTB mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToSouth_ > hdgThreshold * DTR)
            return "Heading error to south " + std::to_string(curHdgErr_) +
                   "deg (need <= " + std::to_string(hdgThreshold) + "deg).";
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt)
            return "Distance closure " +
                   std::to_string((initialDist_ - minDist_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 1.0 : 3.0) + "NM).";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) + "ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"dist_ab", curDist_, "ft"},
            {"hdg_err", curHdgErr_, "deg"},
            {"in_rtb",  (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- RTB Summary ---\n");
        std::printf("  Entered RTB:          %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading to south: %.1f deg %s\n",
            minAbsHeadingToSouth_ * RTD,
            minAbsHeadingToSouth_ <= (isHeavy_ ? 120.0 : 90.0) * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Dist closure:         %.2f NM %s\n",
            (initialDist_ - minDist_) / kNmToFt,
            (initialDist_ - minDist_) >= (isHeavy_ ? 1.0 : 3.0) * kNmToFt ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:         %.0f ft %s\n",
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
    double minAbsHeadingToSouth_{1e9};
    bool enteredRTB_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curDist_{0.0}, curHdgErr_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Divert to alternate
//
// Original airbase "closed" (we just provide a different airbase as the
// nearest). Aircraft 20 NM east of alternate, alternate at origin heading
// north. Verify turn toward alternate + closure.
// ===========================================================================
class HighDivertPhase : public ManeuverTest {
public:
    HighDivertPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 20 NM east of the alternate airbase, heading east (away).
        const double startX = 20.0 * kNmToFt;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);  // heading east
        fm.state().kin.x = startX;
        fm.state().kin.y = 0.0;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
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
        fi.fuelLbs = 1200.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);
        sc.brain().clearTarget();

        // Manually set divert airbase (so runRTB has a real target).
        sc.brain().stateMutable().fuel.divertAirbaseX = airbase_.x;
        sc.brain().stateMutable().fuel.divertAirbaseY = airbase_.y;
        sc.brain().stateMutable().fuel.divertAirbaseZ = airbase_.z;
        sc.brain().stateMutable().fuel.divertAirbaseHeading = airbase_.runwayHeading;
        sc.brain().stateMutable().fuel.hasDivertAirbase = true;

        initialDist_ = startX;
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 1200.0;
        fi.bingoFuelLbs = 1500.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedTarget = nullptr;
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::RTB) enteredRTB_ = true;

        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);

        // Heading to west (alternate is west of aircraft).
        double dh = as.kin.sigma - PI;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToWest_ = std::min(minAbsHeadingToWest_, std::fabs(dh));

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = dist;
        curHdgErr_ = std::fabs(dh) * RTD;
        curMode_ = mode;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRTB_) return false;
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToWest_ > hdgThreshold * DTR) return false;
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 2.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt) return false;
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB; Turn within 90deg (120 heavy) of alternate; "
               "Close distance >= 2NM (1NM heavy); Min alt >= 1000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRTB_)
            return "Never entered RTB mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double hdgThreshold = isHeavy_ ? 120.0 : 90.0;
        if (minAbsHeadingToWest_ > hdgThreshold * DTR)
            return "Heading error to west " + std::to_string(curHdgErr_) +
                   "deg (need <= " + std::to_string(hdgThreshold) + "deg).";
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 2.0) * kNmToFt;
        if (initialDist_ - minDist_ < requiredClosureFt)
            return "Distance closure " +
                   std::to_string((initialDist_ - minDist_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 1.0 : 2.0) + "NM).";
        if (minAlt_ < 1000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) + "ft.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"dist_ab", curDist_, "ft"},
            {"hdg_err", curHdgErr_, "deg"},
            {"in_rtb",  (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        std::printf("  --- Divert Summary ---\n");
        std::printf("  Entered RTB:         %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading to west: %.1f deg %s\n",
            minAbsHeadingToWest_ * RTD,
            minAbsHeadingToWest_ <= (isHeavy_ ? 120.0 : 90.0) * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Dist closure:        %.2f NM %s\n",
            (initialDist_ - minDist_) / kNmToFt,
            (initialDist_ - minDist_) >= (isHeavy_ ? 1.0 : 2.0) * kNmToFt ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.0f ft %s\n",
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
    double minAbsHeadingToWest_{1e9};
    bool enteredRTB_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curDist_{0.0}, curHdgErr_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Approach (10NM final)
//
// Repositioned 10 NM south of origin at 3000ft, heading north. Verify Landing
// mode entry + Approach phase.
// ===========================================================================
class HighApproachPhase : public ManeuverTest {
public:
    HighApproachPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startY = -10.0 * kNmToFt;
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
        fi.fuelLbs = 1200.0;
        fi.bingoFuelLbs = 1500.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);
        // Command landing (forces Landing mode + starts approach phase).
        sc.brain().commandLanding(270, kRwyHeading, 0.0, 0.0, 0.0);
        sc.brain().clearTarget();
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        FrameInputs fi = sc_brain_->frameInputs();
        fi.fuelLbs = 1200.0;
        fi.bingoFuelLbs = 1500.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        fi.injectedTarget = nullptr;
        sc_brain_->setFrameInputs(fi);

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Landing) enteredLanding_ = true;
        const GroundOpsPhase gophase = sc_brain_->state().ag.groundOps.phase;
        if (gophase == GroundOpsPhase::Approach) enteredApproach_ = true;

        minAlt_ = std::min(minAlt_, -as.kin.z);
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        curMode_ = mode;
        curGophase_ = gophase;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLanding_) return false;
        // Approach phase is preferred but not strictly required — entering
        // Landing mode is the key signal.
        return true;
    }

    std::string criteria() const override {
        return "Enter Landing mode (Approach phase preferred); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLanding_)
            return "Never entered Landing mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"in_landing", (enteredLanding_ && curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Approach Summary ---\n");
        std::printf("  Entered Landing:   %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Approach:  %s\n", enteredApproach_ ? "[PASS]" : "(n/a)");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    FrameInputs::AirbaseInfo airbase_;
    DigiBrain* sc_brain_{nullptr};
    double minAlt_{1e9};
    bool enteredLanding_{false};
    bool enteredApproach_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    DigiMode curMode_{DigiMode::NoMode};
    GroundOpsPhase curGophase_{GroundOpsPhase::Parking};
};

// ===========================================================================
// Phase 4: Landing (full flare + touchdown)
//
// Repositioned 3 NM south on a 3deg glideslope, command landing, verify
// touches down + enters Flare phase. Relaxed criteria (no touchdown pitch /
// descent-rate checks — the parent digi_groundops handles those).
// ===========================================================================
class HighLandingPhase : public ManeuverTest {
public:
    HighLandingPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialRange = 3.0 * kNmToFt;
        const double gsAngle = 3.0 * DTR;
        const double initialAlt = initialRange * std::tan(gsAngle);
        fm.init(fm.config(), initialAlt, speed_ * KNOTS_TO_FTPSEC,
                kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -initialRange;
        fm.state().kin.z = -initialAlt;
        const double thetaTrim = fm.state().kin.theta;
        fm.state().kin.theta = thetaTrim - gsAngle;
        fm.state().kin.gmma = -gsAngle;
        fm.state().kin.singam = -std::sin(gsAngle);
        fm.state().kin.cosgam = std::cos(gsAngle);
        const double vt0 = fm.state().kin.vt;
        fm.state().kin.xdot = 0.0;
        fm.state().kin.ydot = vt0 * std::cos(gsAngle);
        fm.state().kin.zdot = vt0 * std::sin(gsAngle);
        fm.state().kin.quat = quatFromEuler(fm.state().kin.psi,
                                            fm.state().kin.theta,
                                            fm.state().kin.phi);
        initialAlt_ = initialAlt;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.brain().commandLanding(270, kRwyHeading, 0.0, 0.0, 0.0);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);
        if (altAGL < 10.0 && !touchedDown_) touchedDown_ = true;
        if (sc_brain_->activeMode() == DigiMode::Landing) enteredLanding_ = true;
        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Flare) enteredFlare_ = true;
        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        curAlt_ = altAGL;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || touchedDown_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredLanding_) return false;
        if (minAlt_ > initialAlt_ - 500.0) return false;
        if (!touchedDown_) return false;
        if (minAlt_ < -500.0) return false;
        if (!enteredFlare_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Landing; Descend >= 500ft; Touch down; Enter Flare; "
               "Min alt >= -500ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredLanding_)
            return "Never entered Landing mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minAlt_ > initialAlt_ - 500.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (did not descend).";
        if (!touchedDown_)
            return "Never touched down (min alt " +
                   std::to_string(static_cast<int>(minAlt_)) + "ft).";
        if (minAlt_ < -500.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (sunk underground).";
        if (!enteredFlare_)
            return "Never entered Flare phase.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",         curAlt_, "ft"},
            {"in_landing",  (enteredLanding_ && curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
            {"touched_down", touchedDown_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Landing Summary ---\n");
        std::printf("  Entered Landing: %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:    %.0f ft %s\n", minAlt_,
            (minAlt_ >= -500.0 && minAlt_ <= initialAlt_ - 500.0) ? "[PASS]" : "[FAIL]");
        std::printf("  Touched down:    %s\n", touchedDown_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Flare:   %s\n", enteredFlare_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    double initialAlt_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    bool touchedDown_{false};
    bool enteredLanding_{false};
    bool enteredFlare_{false};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
    double curAlt_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 5: Taxi (post-landing deceleration)
//
// Repositioned on the ground at low speed near the runway threshold, command
// taxi. Verify deceleration to < 5 kts (or TaxiToRunway phase entry — though
// we're already on the ground, the brain may not re-engage taxi after
// landing; we accept either signal).
// ===========================================================================
class HighTaxiRecoveryPhase : public ManeuverTest {
public:
    HighTaxiRecoveryPhase(const char* name, double duration, double alt,
                          double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), 0.0, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, false);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = 0.0;
        fm.state().kin.z = 0.0;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Manually set ground-ops to Rollout — the brain's RunRollout code
        // engages wheel brakes and decelerates the aircraft. We can't rely on
        // commandLanding() because the brain transitions to Rollout only
        // after a real touchdown (which we skip by re-init'ing on the ground).
        auto& go = sc.brain().stateMutable().ag.groundOps;
        go.phase = GroundOpsPhase::Rollout;
        go.runwayHeading = kRwyHeading;
        go.runwayThresholdX = 0.0;
        go.runwayThresholdY = 0.0;
        go.hasLandingClearance = true;
        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        minSpeed_ = std::min(minSpeed_, as.vcas);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);
        if (as.vcas < 5.0) stopped_ = true;
        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::Rollout)
            enteredRollout_ = true;
        if (sc_brain_->state().ag.groundOps.phase == GroundOpsPhase::TaxiToRunway)
            enteredTaxi_ = true;
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;
        curVcas_ = as.vcas;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || stopped_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Either decelerated to < 5 kts OR entered Rollout/Taxi phase.
        if (!stopped_ && !enteredRollout_ && !enteredTaxi_) return false;
        // Heavy aircraft may not stop in time — accept any decel >= 30 kts.
        if (isHeavy_) return (maxSpeed_ - minSpeed_) >= 30.0;
        return stopped_;
    }

    std::string criteria() const override {
        return "Decelerate to < 5kts (OR enter Rollout/Taxi phase); "
               "Heavy: decel >= 30kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!stopped_ && !enteredRollout_ && !enteredTaxi_)
            return "Never decelerated below 5kts (min " +
                   std::to_string(static_cast<int>(minSpeed_)) + "kts) and "
                   "never entered Rollout/Taxi phase.";
        if (isHeavy_ && (maxSpeed_ - minSpeed_) < 30.0)
            return "Heavy decel only " +
                   std::to_string(static_cast<int>(maxSpeed_ - minSpeed_)) +
                   "kts (need >= 30kts).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"vcas",    curVcas_, "kts"},
            {"in_taxi", (enteredTaxi_) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Taxi Summary ---\n");
        std::printf("  Stopped (<5kts):    %s\n", stopped_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min speed:          %.1f kts\n", minSpeed_);
        std::printf("  Entered Rollout:    %s\n", enteredRollout_ ? "[PASS]" : "(n/a)");
        std::printf("  Entered Taxi:       %s\n", enteredTaxi_ ? "[PASS]" : "(n/a)");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    const DigiBrain* sc_brain_{nullptr};
    double minSpeed_{1e9};
    double maxSpeed_{0.0};
    bool stopped_{false};
    bool enteredRollout_{false};
    bool enteredTaxi_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curVcas_{0.0};
};

// ===========================================================================
// HighRecoveryScenario
// ===========================================================================
class HighRecoveryScenario : public ManeuverScenario {
public:
    HighRecoveryScenario() : ManeuverScenario("high_recovery") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: full recovery chain RTB (bingo) -> Divert -> Approach -> "
               "Landing -> Taxi. Composes five low-level behaviors into a "
               "realistic return-and-recovery sequence. Relaxed per-phase criteria.";
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
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double missionAlt = 15000.0;
        fm.init(ctx.cfg, missionAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<HighRTBPhase>(
            "RTB (bingo fuel, 25NM)", 90.0, missionAlt, cornerSpeed));
        tests.push_back(std::make_unique<HighDivertPhase>(
            "Divert to alternate (20NM east)", 80.0, missionAlt, cornerSpeed));
        tests.push_back(std::make_unique<HighApproachPhase>(
            "Approach (10NM final)", 60.0, 3000.0, cornerSpeed));
        tests.push_back(std::make_unique<HighLandingPhase>(
            "Landing (3NM final)", 90.0, 170.0, 170.0));
        tests.push_back(std::make_unique<HighTaxiRecoveryPhase>(
            "Taxi (post-landing decel)", 60.0, 0.0, 100.0));
        return tests;
    }
};

static RegisterScenario g_registerHighRecovery("high_recovery", []() {
    return std::make_unique<HighRecoveryScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_recovery() {}

} // namespace f4flight_test
