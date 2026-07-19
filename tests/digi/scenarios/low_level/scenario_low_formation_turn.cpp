// f4flight - scenarios/scenario_low_formation_turn.cpp
//
// LOW-LEVEL scenario: single 90° formation turn behavior.
//
// Split out of low_level/scenario_digi_formation_maneuver.cpp
// (FormationManeuverPhase). Wraps a SINGLE 90° standard-rate right turn by
// the flight lead — the wingman must bank to follow the slot through the
// arc. The parent scenario covers a full racetrack (N→E→S with 2 turns);
// this test isolates just ONE turn so failures localize to "can the wingman
// follow a single turn".
//
// Pass criteria is RELAXED vs the parent scenario: drop the SUSTAINED
// proximity time requirement (just require closing to the slot at some
// point + entering Wingy mode + reasonable speed match). The parent
// requires 15s of sustained proximity; we don't.
//
// Tier: LowLevel. Registered as "low_formation_turn" — referenced by the
// cascade mapping table g_highToLow["high_formation_joinup"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/formation/formation_geometry.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowFormationTurnPhase — single 90° turn by the lead
// ===========================================================================
class LowFormationTurnPhase : public ManeuverTest {
public:
    LowFormationTurnPhase(const char* name, double duration,
                          double alt, double leadSpeed, double startOffsetFt)
        : ManeuverTest(name, duration), alt_(alt), leadSpeed_(leadSpeed),
          startOffset_(startOffsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        const double wingmanInitKts = fm.config().geometry.cornerVcas_kts > 0
            ? fm.config().geometry.cornerVcas_kts : 330.0;
        fm.init(fm.config(), alt_, wingmanInitKts * KNOTS_TO_FTPSEC,
                initialHeading, true);

        fm.state().kin.x = startOffset_;
        fm.state().kin.y = -startOffset_;
        fm.state().kin.z = -(alt_ - 200.0);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // Lead at origin, flying north.
        const double leadVt = leadSpeed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;
        lead_.pitch = 0.0;
        lead_.roll = 0.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        sc.setWingman(1, 1);
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);

        isHeavy_ = isHeavy(fm.config());
        sc_brain_ = &sc.brain();
    }

    void updateLeadTrajectory(double dt) {
        // Phase boundaries for a SINGLE 90° turn:
        //   0 – 10 s  : straight north
        //   10 – 40 s : turn right 90° at standard rate (3°/s, 30 s for 90°)
        //   40 – 60 s : straight east
        constexpr double kLeg1End  = 10.0;
        constexpr double kTurn1End = 40.0;
        constexpr double kTurnRate = PI / 60.0;  // 3°/s

        const double leadBankRad =
            (lead_.speed > 1.0)
            ? std::atan(kTurnRate * lead_.speed / 32.174)
            : 0.0;

        if (phaseTime_ < kLeg1End) {
            lead_.yaw = PI / 2.0;
            lead_.roll = 0.0;
            curLeadPhase_ = "NORTH";
        } else if (phaseTime_ < kTurn1End) {
            const double tIntoTurn = phaseTime_ - kLeg1End;
            lead_.yaw = PI / 2.0 - tIntoTurn * kTurnRate;
            lead_.roll = -leadBankRad;
            curLeadPhase_ = "TURN->E";
        } else {
            lead_.yaw = 0.0;
            lead_.roll = 0.0;
            curLeadPhase_ = "EAST";
        }

        lead_.vx = lead_.speed * std::cos(lead_.yaw);
        lead_.vy = lead_.speed * std::sin(lead_.yaw);
        lead_.x += lead_.vx * dt;
        lead_.y += lead_.vy * dt;
        lead_.dcm = dcmFromEuler(lead_.yaw, lead_.pitch, lead_.roll);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        updateLeadTrajectory(dt);

        // Compute the wingman's desired slot position.
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
        const double spdErr = std::fabs(wingTasKts - leadTasKts);
        maxSpeedErr_ = std::max(maxSpeedErr_, spdErr);

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curDistToSlot_ = distToSlot;
        curMode_ = sc_brain_->activeMode();
        curWingBank_ = as.kin.phi;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (single 90deg turn, AI=slot 1 Wedge)\n", testName_.c_str());
                std::printf("%6s %6s %8s %8s %6s %6s %s\n",
                    "t(s)", "phase", "wngX", "wngY", "dSlot", "wbnk", "mode");
            }
            std::printf("%6.1f %6s %8.0f %8.0f %8.0f %6.1f %s\n",
                phaseTime_, curLeadPhase_.c_str(),
                as.kin.x, as.kin.y, distToSlot,
                curWingBank_ * 180.0 / PI, digiModeName(curMode_));
            nextPrint_ += 5.0;
        }
    }

    std::vector<ThreatEntity> traceEntities() const override {
        ThreatEntity s;
        s.type = "slot";
        s.name = "Slot 1";
        s.x = curSlotX_; s.y = curSlotY_; s.z = curSlotZ_;
        return {s};
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        // RELAXED: parent requires min dist < 500ft + 15s sustained proximity
        // + TAS match < 70kts. We only require closing to within 1000ft at
        // some point (heavy: 1500ft) + reasonable speed match (heavy: 100kts).
        const double closeThresh = isHeavy_ ? 1500.0 : 1000.0;
        if (minDistToSlot_ > closeThresh) return false;
        const double spdThresh = isHeavy_ ? 100.0 : 80.0;
        if (maxSpeedErr_ > spdThresh) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy mode; Close to slot within 1000ft (1500 heavy); "
               "TAS match < 80kts (100 heavy); No NaN [single 90deg turn]";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredWingy_)
            return "Never entered Wingy mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        const double closeThresh = isHeavy_ ? 1500.0 : 1000.0;
        if (minDistToSlot_ > closeThresh)
            return "Min dist to slot was " +
                   std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (needed < " + std::to_string(static_cast<int>(closeThresh)) + "ft).";
        const double spdThresh = isHeavy_ ? 100.0 : 80.0;
        if (maxSpeedErr_ > spdThresh)
            return "Max TAS error " + std::to_string(static_cast<int>(maxSpeedErr_)) +
                   "kts (needed < " + std::to_string(static_cast<int>(spdThresh)) + "kts).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",    curDistToSlot_, "ft"},
            {"spd_err",   maxSpeedErr_,   "kts"},
            {"in_wingy",  (enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
            {"wing_bank", curWingBank_ * 180.0 / PI, "deg"},
        };
    }

    void Finish() const override {
        const double closeThresh = isHeavy_ ? 1500.0 : 1000.0;
        const double spdThresh = isHeavy_ ? 100.0 : 80.0;
        std::printf("  --- Summary (single 90deg turn) ---\n");
        std::printf("  Entered Wingy mode:  %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot:    %.0f ft (need < %.0f) %s\n",
            minDistToSlot_, closeThresh,
            minDistToSlot_ < closeThresh ? "[PASS]" : "[FAIL]");
        std::printf("  Max TAS error:       %.1f kts (need < %.0f) %s\n",
            maxSpeedErr_, spdThresh,
            maxSpeedErr_ < spdThresh ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double leadSpeed_{0.0};
    double startOffset_{0.0};
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};
    bool isHeavy_{false};
    bool enteredWingy_{false};
    bool hasNaN_{false};
    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    double nextPrint_{0.0};
    double curDistToSlot_{0.0};
    double curWingBank_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    mutable double curSlotX_{0.0}, curSlotY_{0.0}, curSlotZ_{0.0};
    std::string curLeadPhase_{"NORTH"};
};

// ===========================================================================
// LowFormationTurnScenario
// ===========================================================================
class LowFormationTurnScenario : public ManeuverScenario {
public:
    LowFormationTurnScenario() : ManeuverScenario("low_formation_turn") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: single 90° formation turn. AI wingman (slot 1, Wedge) "
               "follows a flight lead that flies north, turns right 90° to "
               "east, then flies east. Single phase, relaxed pass criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 10000.0;
        const double cornerVcas = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, alt, cornerVcas * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowFormationTurnPhase>(
            "Single 90deg formation turn", 60.0, alt, cornerVcas, 1000.0));
        return tests;
    }
};

static RegisterScenario g_registerLowFormationTurn("low_formation_turn", []() {
    return std::make_unique<LowFormationTurnScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_formation_turn() {}

} // namespace f4flight_test
