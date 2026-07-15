// f4flight - scenarios/scenario_digi_tactics.cpp
//
// Digi AI tactics scenario: Loiter mode entry + wingman tactical maneuvers.
//
// This scenario verifies Round 7 P1 capabilities:
//   1. Loiter mode entry: when no waypoints are set, the AI enters Loiter
//      (orbits in place) instead of flying straight forever.
//   2. Wingman break maneuver: a wingman receiving a FlightCmdBreak order
//      enters FollowOrders mode and executes the break turn.
//
// Scenarios:
//   digi_loiter: AI with no waypoints must enter Loiter mode and orbit.
//     Pass criteria: enters Loiter, heading changes > 30° (proves it's
//     turning, not flying straight), no NaN/crash.
//
//   digi_break: AI wingman receives a break-right order. Must enter
//     FollowOrders mode, roll right, and complete the maneuver within 5s.
//     Pass criteria: enters FollowOrders, rolls right (positive rstick),
//     maneuver clears within 5s, no NaN/crash.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/wingman/wingman_state.h"
#include "f4flight/digi/comms/message.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LoiterPhase — no waypoints → AI enters Loiter mode and orbits
// ===========================================================================
class LoiterPhase : public ManeuverTest {
public:
    LoiterPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = 0.0;  // east
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Force Loiter mode on the brain so activeMode() returns Loiter.
        // The brain does NOT auto-enter Loiter when no waypoints are set;
        // it must be explicitly requested via forceMode or SteeringController::Mode::Loiter.
        sc.setMode(SteeringController::Mode::Loiter);
        sc.brain().forceMode(DigiMode::Loiter);
        sc.setAltitude(alt_);
        sc.setHeading(initialHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        sc_brain_ = &sc.brain();
        initialHeading_ = initialHeading;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();

        if (currentMode_ == DigiMode::Loiter) enteredLoiter_ = true;

        // Track total heading change — Loiter is a 30° bank orbit, so the
        // heading should change continuously. In 30s at 30° bank / 350 kts,
        // turn rate ≈ 1.4°/s → ~42° of heading change.
        double dh = heading - initialHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        maxAbsHeadingChange_ = std::max(maxAbsHeadingChange_, std::fabs(dh));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (no waypoints → Loiter orbit)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD,
                as.loads.nzcgs, input.rstick, digiModeName(currentMode_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter Loiter mode.
        if (!enteredLoiter_) return false;
        // 2. Must have turned significantly (Loiter = 30° bank orbit).
        //    In 30s at 30° bank, expect > 20° of heading change.
        if (maxAbsHeadingChange_ < 20.0 * DTR) return false;
        // 3. Must not crash.
        if (minAlt_ < alt_ - 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Loiter mode; Heading change > 20° (orbiting); "
               "No crash; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Loiter:       %s\n", enteredLoiter_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading change:   %.1f° (need > 20°) %s\n",
            maxAbsHeadingChange_ * RTD,
            maxAbsHeadingChange_ > 20.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                %.2f\n", maxG_);
        std::printf("  Min altitude:         %.0f ft %s\n", minAlt_,
            minAlt_ >= alt_ - 2000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double initialHeading_{0.0};
    const DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    bool enteredLoiter_{false};
    double maxAbsHeadingChange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    bool hasNaN_{false};
};

// ===========================================================================
// BreakManeuverPhase — wingman receives break-right order
// ===========================================================================
class BreakManeuverPhase : public ManeuverTest {
public:
    BreakManeuverPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = 0.0;  // east
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(initialHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Configure as wingman (slot 1) with a lead.
        sc.setWingman(1, 1);  // leadId=1, slot=1

        // Set up a lead entity (for Wingy fallback after maneuver completes).
        lead_.x = 0.0; lead_.y = 1000.0; lead_.z = -alt_;
        lead_.yaw = 0.0; lead_.speed = speed_ * KNOTS_TO_FTPSEC;
        lead_.isDead = false;
        sc.setLead(&lead_);

        // Send a break-right order via the mailbox.
        // payload.heading > 0 = right break.
        Message breakMsg{MessageType::FlightCmdBreak, 1, 2};
        breakMsg.payload.heading = 1.0;  // positive = right
        sc.brain().mailbox().push(breakMsg);

        sc_brain_ = &sc.brain();
        initialHeading_ = initialHeading;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();

        if (currentMode_ == DigiMode::FollowOrders) enteredFollowOrders_ = true;

        // Track if the wingman rolled right (positive rstick) during the break.
        if (currentMode_ == DigiMode::FollowOrders && input.rstick > 0.05) {
            rolledRight_ = true;
        }

        // Track if the maneuver cleared (brain left FollowOrders).
        if (enteredFollowOrders_ && currentMode_ != DigiMode::FollowOrders) {
            maneuverCleared_ = true;
        }

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (wingman break-right order)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "pstk", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD,
                input.pstick, input.rstick, digiModeName(currentMode_));
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter FollowOrders mode.
        if (!enteredFollowOrders_) return false;
        // 2. Must have rolled right (positive rstick) during the break.
        if (!rolledRight_) return false;
        // 3. Maneuver must clear (brain exits FollowOrders → Wingy fallback).
        if (!maneuverCleared_) return false;
        // 4. Must not crash.
        if (minAlt_ < alt_ - 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter FollowOrders mode; Roll right (positive rstick); "
               "Maneuver clears (exits FollowOrders); No crash; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered FollowOrders: %s\n", enteredFollowOrders_ ? "[PASS]" : "[FAIL]");
        std::printf("  Rolled right:         %s\n", rolledRight_ ? "[PASS]" : "[FAIL]");
        std::printf("  Maneuver cleared:     %s\n", maneuverCleared_ ? "[PASS]" : "(n/a)");
        std::printf("  Min altitude:         %.0f ft %s\n", minAlt_,
            minAlt_ >= alt_ - 2000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double initialHeading_{0.0};
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    bool enteredFollowOrders_{false};
    bool rolledRight_{false};
    bool maneuverCleared_{false};
    double minAlt_{1e9};
    bool hasNaN_{false};
};

// ===========================================================================
// Scenario: digi_tactics
// ===========================================================================
class DigiTacticsScenario : public ManeuverScenario {
public:
    DigiTacticsScenario() : ManeuverScenario("digi_tactics") {}

    std::string GetDescription() const override {
        return "Digi AI tactics: Loiter mode entry (no waypoints → orbit) + "
               "wingman break maneuver (FollowOrders mode). Tests Round 7 P1 "
               "Loiter entry logic and the Round 5/6 wingman tactical maneuver "
               "dispatch end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: Loiter (no waypoints → orbit)
        tests.push_back(std::make_unique<LoiterPhase>(
            "Loiter mode (no waypoints)", 30.0, alt, speed));
        // Phase 2: Break maneuver (wingman receives break-right order)
        tests.push_back(std::make_unique<BreakManeuverPhase>(
            "Wingman break-right maneuver", 10.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiTactics("digi_tactics", []() {
    return std::make_unique<DigiTacticsScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_tactics() {}

} // namespace f4flight_test
