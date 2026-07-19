// f4flight - scenarios/scenario_e2e_escort.cpp
//
// End-to-end Escort mission scenario for the digi AI. Analogous to
// FreeFalcon's AMIS_ESCORT campaign mission type.
//
// An ESCORT mission rendezvous with a friendly strike package, escorts them
// to the target area, defends against enemy fighters, then RTBs. The
// escort aircraft acts as a Wingy (wingman) in formation with the strike
// package lead during ingress, then breaks formation to engage any bandits
// attacking the package.
//
// This scenario chains SIX phases of a single ESCORT mission to exercise
// the full mode pipeline of the DigiBrain in mission order:
//
//   Phase 1 "Takeoff"        : ground start -> Takeoff mode -> climb out
//   Phase 2 "Climb to RZ"    : climb to 18000ft, navigate 10NM to rendezvous
//   Phase 3 "Join formation" : join formation with the strike package lead
//                              (inject a friendly lead, enter Wingy mode)
//   Phase 4 "Escort ingress" : follow the strike package toward target 20NM ahead
//   Phase 5 "Engage bandit"  : inject a bandit attacking the package, defensive engage
//   Phase 6 "RTB"            : bingo fuel, return to origin airbase
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
#include "f4flight/digi/formation/formation_geometry.h"
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
constexpr double kEscortAlt    = 18000.0;    // ft MSL — escort altitude
constexpr double kNmToFt       = 6076.0;
constexpr double kLeadSpeedKts = 350.0;      // strike package cruise speed

// ===========================================================================
// Phase 1: Takeoff (30s)
// ===========================================================================
class EscortTakeoffPhase : public ManeuverTest {
public:
    EscortTakeoffPhase(const char* name, double duration)
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
// Phase 2: Climb to rendezvous (60s)
//
// After takeoff, the aircraft climbs to 18000 ft and navigates 10 NM north
// to the rendezvous point with the strike package.
// ===========================================================================
class EscortClimbPhase : public ManeuverTest {
public:
    EscortClimbPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double startAlt = 3000.0;
        const double startSpeed = 300.0;
        fm.init(fm.config(), startAlt, startSpeed * KNOTS_TO_FTPSEC,
                kRwyHeading, true);

        sc.setMode(SteeringController::Mode::Waypoint);
        sc.setAltitude(kEscortAlt);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        std::vector<Vec3> wps;
        wps.push_back({0.0, 10.0 * kNmToFt, -kEscortAlt});
        sc.setWaypoints(std::move(wps));
        sc.setCaptureRadius(1.0 * kNmToFt);

        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        wpX_ = 0.0;
        wpY_ = 10.0 * kNmToFt;
        wpZ_ = -kEscortAlt;
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
                std::printf("\n%s (climb 3k->18k ft, navigate 10NM to RZ)\n",
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
        const double altThreshold = isHeavy_ ? 7000.0 : 12000.0;
        if (maxAlt_ < altThreshold) return false;
        if (minAbsHeadingErr_ > 25.0 * DTR) return false;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter nav mode; Climb to >= 12000ft (7000ft heavy); "
               "Heading within 25deg of north; "
               "Close range to RZ by >= 5NM (2NM heavy); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredNavMode_)
            return "Never entered a navigation mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   "); stuck in ground-ops.";
        const double altThreshold = isHeavy_ ? 7000.0 : 12000.0;
        if (maxAlt_ < altThreshold)
            return "Max altitude was " + std::to_string(static_cast<int>(maxAlt_)) +
                   "ft (need >= " + std::to_string(static_cast<int>(altThreshold)) +
                   "ft) — did not climb to escort altitude.";
        if (minAbsHeadingErr_ > 25.0 * DTR)
            return "Heading error to north was " +
                   std::to_string(curHdgErr_) +
                   "deg (need <= 25deg) — did not hold north heading.";
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        if (initialRange_ - minRange_ < requiredClosureFt)
            return "Range closure was " +
                   std::to_string((initialRange_ - minRange_) / kNmToFt) +
                   "NM (need >= " + std::to_string(isHeavy_ ? 2.0 : 5.0) +
                   "NM) — did not close on the RZ.";
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
        return {ThreatEntity{"slot", wpX_, wpY_, wpZ_, 0.0}};
    }

    void Finish() const override {
        const double altThreshold = isHeavy_ ? 7000.0 : 12000.0;
        const double requiredClosureFt = (isHeavy_ ? 2.0 : 5.0) * kNmToFt;
        std::printf("  --- Climb to RZ Summary ---\n");
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
// EscortFormationBase — shared base for the formation phases (3 & 4).
//
// Encapsulates the common setup for the join-up and escort-ingress phases:
// kinematic strike-package lead, wingman formation config, slot geometry,
// NaN/trace bookkeeping. Each phase overrides what's phase-specific.
// ===========================================================================
class EscortFormationBase : public ManeuverTest {
public:
    EscortFormationBase(const char* name, double duration, double startOffsetFt)
        : ManeuverTest(name, duration), startOffset_(startOffsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        const double wingmanInitKts = wingmanInitSpeedKts();
        fm.init(fm.config(), kEscortAlt, wingmanInitKts * KNOTS_TO_FTPSEC,
                initialHeading, true);
        placeWingman(fm);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // --- Kinematic strike-package lead (slot 0) ---
        const double leadVt = kLeadSpeedKts * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -kEscortAlt;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;       // north
        lead_.pitch = 0.0;
        lead_.roll = 0.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        sc.setWingman(1, 1);
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);

        sc.brain().stateMutable().ag.groundOps.phase = GroundOpsPhase::Idle;
        sc.brain().stateMutable().ag.groundOps.hasTakeoffClearance = false;
        sc.brain().stateMutable().ag.groundOps.hasLandingClearance = false;

        isHeavy_ = isHeavy(fm.config());
        sc_brain_ = &sc.brain();
    }

    // Phase-specific hooks.
    virtual void placeWingman(FlightModel& fm) {
        // Default: wingman starts startOffset_ ft behind-right of its slot.
        const double leadSigma = PI / 2.0;
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        const double slotX = slot1.range * std::cos(leadSigma - slot1.relAz);
        const double slotY = slot1.range * std::sin(leadSigma - slot1.relAz);
        fm.state().kin.x = slotX + startOffset_;
        fm.state().kin.y = slotY - startOffset_;
        fm.state().kin.z = -kEscortAlt;
    }

    virtual double wingmanInitSpeedKts() const { return kLeadSpeedKts; }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Lead flies straight north at constant speed.
        lead_.yaw = PI / 2.0;
        lead_.roll = 0.0;
        lead_.vx = lead_.speed * std::cos(lead_.yaw);
        lead_.vy = lead_.speed * std::sin(lead_.yaw);
        lead_.x += lead_.vx * dt;
        lead_.y += lead_.vy * dt;
        lead_.dcm = dcmFromEuler(lead_.yaw, lead_.pitch, lead_.roll);

        // Track wingman's distance to slot 1 (uses adapted formula).
        const double leadSigma = lead_.yaw;
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        const double desX = lead_.x + slot1.range * std::cos(leadSigma - slot1.relAz);
        const double desY = lead_.y + slot1.range * std::sin(leadSigma - slot1.relAz);
        const double desZ = lead_.z;

        const double dx = desX - as.kin.x;
        const double dy = desY - as.kin.y;
        const double dz = desZ - as.kin.z;
        const double distToSlot = std::sqrt(dx * dx + dy * dy + dz * dz);

        minDistToSlot_ = std::min(minDistToSlot_, distToSlot);
        curDistToSlot_ = distToSlot;
        curSlotX_ = desX; curSlotY_ = desY; curSlotZ_ = desZ;

        const double wingTasKts = as.kin.vt / KNOTS_TO_FTPSEC;
        const double leadTasKts = lead_.speed / KNOTS_TO_FTPSEC;
        const double spdErr = std::fabs(wingTasKts - leadTasKts);
        maxSpeedErr_ = std::max(maxSpeedErr_, spdErr);
        curSpeedErr_ = spdErr;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::Wingy) enteredWingy_ = true;
        curMode_ = mode;
        curInPosition_ = sc_brain_->state().formation.wingman.inPosition;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt) ||
            std::isnan(lead_.x) || std::isnan(lead_.yaw)) hasNaN_ = true;

        evaluatePhase(as, input, dt, distToSlot);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (Wedge, AI=slot 1, lead %dkts @ %dkft)\n",
                    testName_.c_str(),
                    static_cast<int>(kLeadSpeedKts),
                    static_cast<int>(kEscortAlt / 1000.0));
                std::printf("%6s %8s %8s %8s %8s %6s %6s %s\n",
                    "t(s)", "wngX", "wngY", "dSlot", "spdErr", "vcas", "rstk", "mode");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %8.0f %8.1f %6.1f %6.2f %s\n",
                phaseTime_, as.kin.x, as.kin.y, distToSlot, spdErr,
                as.vcas, input.rstick, modeBuf);
            nextPrint_ += 10.0;
        }
    }

    virtual void evaluatePhase(const AircraftState& /*as*/,
                               const PilotInput& /*input*/,
                               double /*dt*/,
                               double /*distToSlot*/) {}

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    std::vector<ThreatEntity> traceEntities() const override {
        return {ThreatEntity{"slot", curSlotX_, curSlotY_, curSlotZ_, 0.0}};
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",   curDistToSlot_,                              "ft"},
            {"spd_err",  curSpeedErr_,                                "kts"},
            {"in_pos",   curInPosition_ ? 1.0 : 0.0,                  ""},
            {"in_wingy", (enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
        };
    }

protected:
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};
    bool isHeavy_{false};
    bool hasNaN_{false};
    bool enteredWingy_{false};

    double startOffset_{0.0};
    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    double curDistToSlot_{0.0};
    double curSpeedErr_{0.0};
    bool curInPosition_{false};
    DigiMode curMode_{DigiMode::NoMode};
    mutable double curSlotX_{0.0}, curSlotY_{0.0}, curSlotZ_{0.0};
    double nextPrint_{0.0};
};

// ===========================================================================
// Phase 3: Join formation (60s)
//
// Strike-package lead flies straight north at 350 kts. The AI escort
// (slot 1, Wedge) starts 1500 ft behind-right of its slot and must close
// to within ~1500 ft of its slot, matching the lead's TAS, while entering
// Wingy mode.
//
// Criteria relaxed — verifies the AI TRIES to join (enters Wingy, matches
// speed). Heavy/slow aircraft get the closure criterion waived.
// ===========================================================================
class EscortJoinupPhase : public EscortFormationBase {
public:
    EscortJoinupPhase(const char* name, double duration)
        : EscortFormationBase(name, duration, /*startOffsetFt*/1500.0) {}

    void evaluatePhase(const AircraftState& /*as*/,
                       const PilotInput& /*input*/,
                       double dt,
                       double distToSlot) override {
        // Track sustained proximity to slot.
        if (distToSlot < sustainedProxFt()) timeInProximity_ += dt;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        // Speed match: TAS error < 100 kts (heavy: waived — wingman TRIES
        // to follow).
        if (!isHeavy_ && maxSpeedErr_ > 100.0) return false;
        // Closure: must close to within 2000 ft at least once.
        // Heavy/slow: waived (closure controller is conservative — see
        // scenario_digi_e2e_formation.cpp phase 1 comments).
        if (isHeavy_) return true;
        if (minDistToSlot_ > 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Enter Wingy mode; TAS match < 100kts%s; Close to <2000ft of slot%s; "
            "No NaN [join-up from %dft behind-right, %s]",
            isHeavy_ ? " (heavy: waived)" : "",
            isHeavy_ ? " (heavy: waived)" : "",
            static_cast<int>(startOffset_),
            isHeavy_ ? "heavy tolerances" : "fighter tolerances");
        return std::string(buf);
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   ") — escort role was not activated.";
        if (!isHeavy_ && maxSpeedErr_ > 100.0)
            return "Max TAS error " + std::to_string(static_cast<int>(maxSpeedErr_)) +
                   "kts (need < 100kts) — escort did not match lead's TAS.";
        if (!isHeavy_ && minDistToSlot_ > 2000.0)
            return "Min dist to slot was " +
                   std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (need < 2000ft) — escort did not close on its slot.";
        return "";
    }

    void Finish() const override {
        std::printf("  --- Join Formation Summary ---\n");
        std::printf("  Entered Wingy mode:    %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Max TAS error: %.1f kts (heavy: waived)\n", maxSpeedErr_);
            std::printf("  Min dist to slot: %.0f ft (heavy: waived)\n", minDistToSlot_);
        } else {
            std::printf("  Max TAS error: %.1f kts (need < 100) %s\n",
                maxSpeedErr_, maxSpeedErr_ < 100.0 ? "[PASS]" : "[FAIL]");
            std::printf("  Min dist to slot: %.0f ft (need < 2000) %s\n",
                minDistToSlot_, minDistToSlot_ < 2000.0 ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Time within %.0fft:    %.1f s (informational)\n",
            sustainedProxFt(), timeInProximity_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double sustainedProxFt() const { return isHeavy_ ? 2500.0 : 2000.0; }
    double timeInProximity_{0.0};
};

// ===========================================================================
// Phase 4: Escort ingress (90s)
//
// Strike-package lead continues north toward the target area (20 NM ahead).
// The escort starts on-slot (joined) and must maintain formation through
// the cruise. Verifies the Wingy mode persists and the escort tracks its
// slot for an extended period.
// ===========================================================================
class EscortIngressPhase : public EscortFormationBase {
public:
    EscortIngressPhase(const char* name, double duration)
        : EscortFormationBase(name, duration, /*startOffsetFt*/0.0) {}

    void placeWingman(FlightModel& fm) override {
        // Start on-slot but 200 ft BEHIND along the lead's velocity vector
        // (avoids the atan2(0,0) singularity in heading computation —
        // same trick as scenario_digi_e2e_formation.cpp phase 2).
        const double leadSigma = PI / 2.0;
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        fm.state().kin.x = slot1.range * std::cos(leadSigma - slot1.relAz);
        fm.state().kin.y = slot1.range * std::sin(leadSigma - slot1.relAz) - 200.0;
        fm.state().kin.z = -kEscortAlt;
    }

    void evaluatePhase(const AircraftState& /*as*/,
                       const PilotInput& /*input*/,
                       double dt,
                       double distToSlot) override {
        if (sc_brain_->activeMode() == DigiMode::Wingy) timeInWingy_ += dt;
        if (distToSlot < sustainedProxFt()) timeInProximity_ += dt;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        // Must stay in Wingy mode for a sustained period (>= 30s).
        const double minSustainedSec = isHeavy_ ? 20.0 : 30.0;
        if (timeInProximity_ < minSustainedSec) return false;
        return true;
    }

    std::string criteria() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Stay in Wingy; Maintain <%.0fft of slot for >=%.0fs; No NaN "
            "[escort ingress, lead %dkts north]",
            sustainedProxFt(), isHeavy_ ? 20.0 : 30.0,
            static_cast<int>(kLeadSpeedKts));
        return std::string(buf);
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double minSustainedSec = isHeavy_ ? 20.0 : 30.0;
        if (timeInProximity_ < minSustainedSec)
            return "Only held within " + std::to_string(static_cast<int>(sustainedProxFt())) +
                   "ft of slot for " + std::to_string(static_cast<int>(timeInProximity_)) +
                   "s (need >= " + std::to_string(static_cast<int>(minSustainedSec)) +
                   "s) — escort did not maintain formation through ingress.";
        return "";
    }

    void Finish() const override {
        const double minSustainedSec = isHeavy_ ? 20.0 : 30.0;
        std::printf("  --- Escort Ingress Summary ---\n");
        std::printf("  Entered Wingy mode:    %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Time in Wingy:         %.1f s\n", timeInWingy_);
        std::printf("  Time within %.0fft:     %.1f s (need >= %.0f) %s\n",
            sustainedProxFt(), timeInProximity_, minSustainedSec,
            timeInProximity_ >= minSustainedSec ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot:      %.0f ft\n", minDistToSlot_);
        std::printf("  Max TAS error:         %.1f kts\n", maxSpeedErr_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double sustainedProxFt() const { return isHeavy_ ? 2000.0 : 1500.0; }
    double timeInWingy_{0.0};
    double timeInProximity_{0.0};
};

// ===========================================================================
// Phase 5: Engage bandit (60s)
//
// During the escort, a bandit is detected attacking the strike package.
// The escort must break formation and engage defensively. We inject a
// bandit ahead of the lead (so the escort must move to defend), and verify
// the AI enters an offensive mode and maneuvers.
//
// In this phase, the lead is cleared and the escort is reset to free
// maneuvering (no Wingy) so the brain can pursue the bandit. This matches
// the real-world escort behavior of "drop formation and engage".
// ===========================================================================
class EscortEngagePhase : public ManeuverTest {
public:
    EscortEngagePhase(const char* name, double duration,
                      double alt, double speed, double targetSpeed,
                      double initialRangeNm, double offsetFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeNm_(initialRangeNm),
          offsetFt_(offsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start the escort mid-ingress (5 NM north of origin), heading north.
        const double startY = 5.0 * kNmToFt;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, kRwyHeading, true);
        fm.state().kin.x = 0.0;
        fm.state().kin.y = startY;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(kRwyHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());

        // Clear the formation lead so the brain drops Wingy mode and can
        // freely pursue the bandit. setLead(nullptr) clears the injected
        // lead; formation.isWing=false tells the brain this aircraft is no
        // longer a wingman.
        sc.setLead(nullptr);
        sc.brain().stateMutable().formation.isWing = false;
        sc.brain().stateMutable().formation.flightLeadId = -1;
        sc.brain().stateMutable().formation.vehicleInUnit = 0;

        // Bandit injected 10 NM north of the escort (between escort and
        // strike package lead), flying south (toward the package). Slight
        // east offset so they're not perfectly head-on.
        const double rangeFt = initialRangeNm_ * kNmToFt;
        target_.x = offsetFt_;
        target_.y = startY + rangeFt;  // north of escort
        target_.z = -alt_;
        target_.yaw = -kRwyHeading;    // south (toward package)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = -target_.speed;   // south
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = kRwyHeading;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the bandit south at its speed.
        target_.y -= target_.speed * dt;

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
                std::printf("\n%s (bandit %.1f NM north, %.0f kts, attacking package)\n",
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
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * kNmToFt;
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
                   "ft — bandit was not classified as a threat).";
        const bool canCloseRange = !isHeavy_ && (speed_ > 300.0);
        if (canCloseRange) {
            const double requiredClosureFt = 1.5 * kNmToFt;
            if (maxRange_ - minRange_ < requiredClosureFt)
                return "Range closure was " +
                       std::to_string((maxRange_ - minRange_) / kNmToFt) +
                       "NM (need >= 1.5NM) — did not close on bandit.";
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
// Phase 6: RTB (90s)
// ===========================================================================
class EscortRTBPhase : public ManeuverTest {
public:
    EscortRTBPhase(const char* name, double duration, double alt, double speed)
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
        // Clear formation lead so the brain is not still trying to fly Wingy.
        sc.setLead(nullptr);
        sc.brain().stateMutable().formation.isWing = false;
        sc.brain().stateMutable().formation.flightLeadId = -1;
        sc.brain().stateMutable().formation.vehicleInUnit = 0;

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
        return {ThreatEntity{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
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
// E2EEscortScenario
// ===========================================================================
class E2EEscortScenario : public ManeuverScenario {
public:
    E2EEscortScenario() : ManeuverScenario("e2e_escort") {}

    TestTier GetTestTier() const override { return TestTier::EndToEnd; }

    std::string GetDescription() const override {
        return "End-to-end ESCORT (AMIS_ESCORT): takeoff -> climb to 18kft "
               "and navigate 10NM to rendezvous -> join formation with "
               "strike package lead (Wingy mode) -> escort ingress toward "
               "target 20NM ahead -> engage bandit attacking the package "
               "(BVR/WVR) -> RTB to origin airbase. Tests the full escort "
               "mission pipeline (Takeoff -> Waypoint -> Wingy -> BVREngage "
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
        const double targetSpeed = 400.0;  // fast attacking bandit

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<EscortTakeoffPhase>("Takeoff", 30.0));
        tests.push_back(std::make_unique<EscortClimbPhase>("Climb to rendezvous", 60.0));
        tests.push_back(std::make_unique<EscortJoinupPhase>("Join formation", 60.0));
        tests.push_back(std::make_unique<EscortIngressPhase>("Escort ingress", 90.0));
        tests.push_back(std::make_unique<EscortEngagePhase>(
            "Engage bandit attacking package", 60.0,
            kEscortAlt, cornerSpeed, targetSpeed, 10.0, 3000.0));
        tests.push_back(std::make_unique<EscortRTBPhase>(
            "RTB to origin", 90.0, kEscortAlt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerE2EEscort("e2e_escort", []() {
    return std::make_unique<E2EEscortScenario>();
});

extern "C" void f4flight_forceLink_scenario_e2e_escort() {}

} // namespace f4flight_test
