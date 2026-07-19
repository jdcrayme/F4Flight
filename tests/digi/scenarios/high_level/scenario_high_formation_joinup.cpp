// f4flight - scenarios/scenario_high_formation_joinup.cpp
//
// HIGH-LEVEL scenario: formation join-up chain Join-up -> Formation type
// transition -> Formation turn. Composes three low-level formation behaviors
// into a realistic wingman join-up + station-keeping + maneuver sequence.
//
// Each phase re-inits the FlightModel to a deterministic starting condition.
// Pass criteria are intentionally relaxed.
//
// Pass criteria (per phase):
//   1. Join-up          : enter Wingy mode; close to within 1500ft of slot;
//                         speed error < 90kts; no NaN
//   2. Formation type   : with new formation type, enter Wingy mode; close to
//                         within 1500ft of new slot; no NaN
//   3. Formation turn   : lead turns 90deg; wingman enters Wingy; wingman's
//                         heading converges to lead's new heading (within 45deg)
//                         or shows heading change > 30deg (heavy: 20deg); no NaN
//
// Tier: HighLevel. Registered as "high_formation_joinup" — referenced by the
// cascade mapping tables g_highToLow["high_formation_joinup"] and g_e2eToHigh
// for e2e_escort / digi_e2e_formation.
//
// Task ID: 22

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/formation/formation_geometry.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

constexpr double kFormAlt    = 10000.0;

// Custom formation definitions for the high-level chain.
// relAz uses CW-from-nose convention (positive = right, 180° = behind).
inline formation::Formation highWedgeFormation() {
    formation::Formation f{};
    f[0] = {0.0,                       0.0, 0.0};
    f[1] = {135.0 * M_PI / 180.0,      0.0, 1000.0};   // right wing, behind-right
    f[2] = {-135.0 * M_PI / 180.0,     0.0, 1000.0};   // left wing, behind-left
    f[3] = {180.0 * M_PI / 180.0,      0.0, 2000.0};   // trail
    return f;
}

inline formation::Formation highEchelonFormation() {
    formation::Formation f{};
    f[0] = {0.0,                       0.0, 0.0};
    f[1] = {135.0 * M_PI / 180.0,      0.0, 1000.0};   // right echelon, 1000 ft
    f[2] = {135.0 * M_PI / 180.0,      0.0, 2000.0};   // right echelon, 2000 ft
    f[3] = {135.0 * M_PI / 180.0,      0.0, 3000.0};   // right echelon, 3000 ft
    return f;
}

// ===========================================================================
// Phase 1: Join-up
//
// AI wingman starts 2 NM behind the flight lead, lead flying north at corner
// speed. The AI should enter Wingy mode and close to the wedge slot.
// ===========================================================================
class HighJoinUpPhase : public ManeuverTest {
public:
    HighJoinUpPhase(const char* name, double duration, double alt, double speed,
                    double startBehindNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startBehindNm_(startBehindNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);
        // Place AI wingman startBehindNm_ behind the lead's initial position.
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -startBehindNm_ * 6076.0;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        formation::FormationTable::defaultInstance().registerFormation(
            formation::FormationType::Wedge, highWedgeFormation());

        const double leadVt = speed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        sc.setWingman(1, 1);
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);
        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        lead_.y += lead_.speed * dt;
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
        const double wingTasKts = as.kin.vt / KNOTS_TO_FTPSEC;
        const double leadTasKts = lead_.speed / KNOTS_TO_FTPSEC;
        maxSpeedErr_ = std::max(maxSpeedErr_, std::fabs(wingTasKts - leadTasKts));

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curDistToSlot_ = distToSlot;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        if (minDistToSlot_ > 1500.0) return false;
        if (maxSpeedErr_ > 90.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy mode; Close to <1500ft of wedge slot; "
               "TAS error < 90kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minDistToSlot_ > 1500.0)
            return "Min dist to slot " + std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (need < 1500ft).";
        if (maxSpeedErr_ > 90.0)
            return "Max TAS error " + std::to_string(static_cast<int>(maxSpeedErr_)) +
                   "kts (need < 90kts).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",  curDistToSlot_, "ft"},
            {"spd_err", maxSpeedErr_,   "kts"},
            {"in_wingy",(enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Join-up Summary ---\n");
        std::printf("  Entered Wingy:    %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot: %.0f ft (need < 1500) %s\n",
            minDistToSlot_, minDistToSlot_ < 1500.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max TAS error:    %.1f kts (need < 90) %s\n",
            maxSpeedErr_, maxSpeedErr_ < 90.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, startBehindNm_;
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};
    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    bool enteredWingy_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curDistToSlot_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 2: Formation type transition
//
// Lead transitions from Wedge to Echelon. AI wingman must re-position to the
// new slot. We re-init the AI close to the wedge slot, then register the
// echelon formation and command the brain to use it.
// ===========================================================================
class HighFormationTypePhase : public ManeuverTest {
public:
    HighFormationTypePhase(const char* name, double duration, double alt,
                           double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        formation::FormationTable::defaultInstance().registerFormation(
            formation::FormationType::Wedge, highWedgeFormation());
        formation::FormationTable::defaultInstance().registerFormation(
            formation::FormationType::Echelon, highEchelonFormation());

        // Start the AI wingman in its OLD wedge slot (which becomes wrong
        // once the formation transitions to echelon).
        const double leadVt = speed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        // Position AI at the wedge slot 1 (right wing, behind-right).
        const auto wedgeSlot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        const double wsX = lead_.x + wedgeSlot1.range * std::cos(lead_.yaw - wedgeSlot1.relAz);
        const double wsY = lead_.y + wedgeSlot1.range * std::sin(lead_.yaw - wedgeSlot1.relAz);
        fm.state().kin.x = wsX;
        fm.state().kin.y = wsY;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Command echelon formation — AI must transition to the new slot.
        sc.setWingman(1, 1);
        sc.setFormation(static_cast<int>(formation::FormationType::Echelon));
        sc.setLead(&lead_);
        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        lead_.y += lead_.speed * dt;
        const double leadSigma = lead_.yaw;
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Echelon, 1);
        const double desX = lead_.x + slot1.range * std::cos(leadSigma - slot1.relAz);
        const double desY = lead_.y + slot1.range * std::sin(leadSigma - slot1.relAz);
        const double desZ = lead_.z;
        const double dx = desX - as.kin.x;
        const double dy = desY - as.kin.y;
        const double dz = desZ - as.kin.z;
        const double distToSlot = std::sqrt(dx * dx + dy * dy + dz * dz);
        minDistToSlot_ = std::min(minDistToSlot_, distToSlot);

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curDistToSlot_ = distToSlot;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        if (minDistToSlot_ > 1500.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy mode; Close to <1500ft of new echelon slot; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minDistToSlot_ > 1500.0)
            return "Min dist to new slot " + std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (need < 1500ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",  curDistToSlot_, "ft"},
            {"in_wingy",(enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Formation-Type Summary ---\n");
        std::printf("  Entered Wingy:    %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot: %.0f ft (need < 1500) %s\n",
            minDistToSlot_, minDistToSlot_ < 1500.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};
    double minDistToSlot_{1e9};
    bool enteredWingy_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    double curDistToSlot_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Phase 3: Formation turn
//
// Lead starts at origin heading north, then turns 90deg to heading east after
// a short delay. AI wingman must follow the lead's turn. We verify the
// wingman's heading changes significantly (matching the lead's turn).
// ===========================================================================
class HighFormationTurnPhase : public ManeuverTest {
public:
    HighFormationTurnPhase(const char* name, double duration, double alt,
                           double speed, double turnDelayS)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          turnDelayS_(turnDelayS) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);
        formation::FormationTable::defaultInstance().registerFormation(
            formation::FormationType::Wedge, highWedgeFormation());

        const double leadVt = speed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        // AI wingman in the wedge slot 1.
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        const double sX = lead_.x + slot1.range * std::cos(lead_.yaw - slot1.relAz);
        const double sY = lead_.y + slot1.range * std::sin(lead_.yaw - slot1.relAz);
        fm.state().kin.x = sX;
        fm.state().kin.y = sY;
        fm.state().kin.z = -alt_;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        sc.setWingman(1, 1);
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);
        sc_brain_ = &sc.brain();
        isHeavy_ = isHeavy(fm.config());
        leadTurned_ = false;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // After turnDelayS_, the lead turns 90deg to the east (heading = 0).
        if (!leadTurned_ && phaseTime_ >= turnDelayS_) {
            lead_.yaw = 0.0;  // east
            lead_.vx = lead_.speed;
            lead_.vy = 0.0;
            lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);
            leadTurned_ = true;
            leadTurnedHeading_ = 0.0;
        }
        // Advance lead.
        lead_.x += lead_.vx * dt;
        lead_.y += lead_.vy * dt;

        // Track the wingman's heading change.
        if (!initialWingHeadingInit_) {
            initialWingHeading_ = as.kin.sigma;
            initialWingHeadingInit_ = true;
        }
        double dh = as.kin.sigma - initialWingHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(dh));

        // After lead turns, track wingman convergence to lead's new heading.
        if (leadTurned_) {
            double dh2 = as.kin.sigma - leadTurnedHeading_;
            while (dh2 >  PI) dh2 -= 2.0 * PI;
            while (dh2 < -PI) dh2 += 2.0 * PI;
            minAbsHeadingToLead_ = std::min(minAbsHeadingToLead_, std::fabs(dh2));
        }

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curHdgChg_ = maxHeadingChange_ * RTD;
        curMode_ = sc_brain_->activeMode();
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        if (!leadTurned_) return false;
        // Wingman must maneuver: heading change > 30deg (heavy: 20deg).
        const double hdgThreshold = isHeavy_ ? 20.0 : 30.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy mode; Lead executes 90deg turn; Wingman heading change > 30deg "
               "(20deg heavy); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!leadTurned_)
            return "Lead never turned (turnDelayS=" + std::to_string(turnDelayS_) + "s).";
        const double hdgThreshold = isHeavy_ ? 20.0 : 30.0;
        if (maxHeadingChange_ < hdgThreshold * DTR)
            return "Wingman heading change was " + std::to_string(maxHeadingChange_ * RTD) +
                   "deg (need > " + std::to_string(hdgThreshold) + "deg).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_chg",  curHdgChg_, "deg"},
            {"in_wingy", (enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 20.0 : 30.0;
        std::printf("  --- Formation-Turn Summary ---\n");
        std::printf("  Entered Wingy:        %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Lead turned:          %s\n", leadTurned_ ? "[PASS]" : "[FAIL]");
        std::printf("  Wingman hdg change:   %.1f deg (need > %.0f) %s\n",
            maxHeadingChange_ * RTD, hdgThreshold,
            maxHeadingChange_ > hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, turnDelayS_;
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};
    bool enteredWingy_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    bool leadTurned_{false};
    bool initialWingHeadingInit_{false};
    double initialWingHeading_{0.0};
    double leadTurnedHeading_{0.0};
    double maxHeadingChange_{0.0};
    double minAbsHeadingToLead_{1e9};
    double curHdgChg_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// HighFormationJoinUpScenario
// ===========================================================================
class HighFormationJoinUpScenario : public ManeuverScenario {
public:
    HighFormationJoinUpScenario() : ManeuverScenario("high_formation_joinup") {}

    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "HIGH: formation join-up chain Join-up -> Formation-type "
               "transition -> Formation turn. AI wingman joins a flight lead, "
               "transitions to echelon, then follows the lead through a 90deg "
               "turn. Relaxed per-phase criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, kFormAlt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<HighJoinUpPhase>(
            "Join-up (1NM behind lead)", 90.0, kFormAlt, speed, 1.0));
        tests.push_back(std::make_unique<HighFormationTypePhase>(
            "Wedge -> Echelon transition", 80.0, kFormAlt, speed));
        tests.push_back(std::make_unique<HighFormationTurnPhase>(
            "Lead 90deg turn", 80.0, kFormAlt, speed, 10.0));
        return tests;
    }
};

static RegisterScenario g_registerHighFormationJoinUp("high_formation_joinup", []() {
    return std::make_unique<HighFormationJoinUpScenario>();
});

extern "C" void f4flight_forceLink_scenario_high_formation_joinup() {}

} // namespace f4flight_test
