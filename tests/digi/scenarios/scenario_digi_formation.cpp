// f4flight - scenarios/scenario_digi_formation.cpp
//
// Digi AI formation following test: a wingman AI follows a flight lead.
//
// This scenario verifies the AiFollowLead implementation:
//   1. A flight lead flies a straight course at constant altitude + speed
//   2. A wingman (the AI aircraft) starts offset from its formation slot
//   3. The wingman AI (Wingy mode) should:
//      - Enter Wingy mode
//      - Fly toward its formation slot (relative to the lead)
//      - Maintain formation position (stay within a tolerance of the slot)
//      - Match the lead's speed and heading
//
// The lead is a simple kinematic entity (no flight model) — it just flies
// straight at a constant velocity. The wingman uses the full flight model
// + digi AI to follow.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/formation/formation_geometry.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// FormationFollowPhase — wingman follows a lead flying straight
// ===========================================================================
class FormationFollowPhase : public ManeuverTest {
public:
    FormationFollowPhase(const char* name, double duration,
                         double alt, double speed, double startOffsetFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffset_(startOffsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Init the wingman (the AI aircraft) at the specified altitude + speed,
        // offset from the lead's position.
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Place the wingman behind + below the lead's desired slot position.
        // The lead is at (0, 0, -alt). The wingman starts at (startOffset_, 0, -alt+200)
        // — offset to the right and slightly below, so the AI has to maneuver
        // to reach its slot.
        fm.state().kin.x = startOffset_;
        fm.state().kin.y = -startOffset_;  // behind the lead
        fm.state().kin.z = -(alt_ - 200.0);  // 200 ft below

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // Set up the lead entity. The lead flies north at constant speed/alt.
        const double leadVt = speed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;  // north (sigma = velocity heading)
        lead_.pitch = 0.0;
        lead_.roll = 0.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        // Configure the wingman: slot 1 (first wingman), Wedge formation.
        sc.setWingman(1, 1);  // leadId=1, slot=1
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the lead forward at its speed
        lead_.y += lead_.speed * dt;

        // Re-inject the lead each frame (the brain reads frameInputs_.injectedLead)
        // — the SteeringController caches the pointer, but the lead's position
        // changes, so we need to ensure the brain sees the updated entity.
        // Since lead_ is a member, the pointer is stable; the brain reads
        // the current position each frame.

        // Track the wingman's distance to its desired formation slot.
        // The slot for Wedge slot 1 is: relAz=30°, range=1000 ft, relEl=0.
        // Desired world position = leadPos + 1000*cos(30°+leadSigma),
        //                                       1000*sin(30°+leadSigma)
        const double slotRange = 1000.0;
        const double slotAz = 30.0 * DTR;
        const double leadSigma = lead_.yaw;
        const double desX = lead_.x + slotRange * std::cos(slotAz + leadSigma);
        const double desY = lead_.y + slotRange * std::sin(slotAz + leadSigma);
        const double desZ = lead_.z;

        const double dx = desX - as.kin.x;
        const double dy = desY - as.kin.y;
        const double dz = desZ - as.kin.z;
        const double distToSlot = std::sqrt(dx * dx + dy * dy + dz * dz);

        minDistToSlot_ = std::min(minDistToSlot_, distToSlot);

        maxSpeedErr_ = std::max(maxSpeedErr_, std::fabs(as.vcas - speed_));

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (sc_brain_->state().formation.wingman.inPosition) inPosition_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (wingman follows lead, Wedge slot 1)\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "wngX", "wngY", "dSlot", "vcas", "pstk", "rstk", "mode");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %8.0f %8.0f %6.1f %6.2f %6.2f %6s\n",
                phaseTime_, as.kin.x, as.kin.y, distToSlot, as.vcas,
                input.pstick, input.rstick, modeBuf);
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered Wingy mode
        if (!enteredWingy_) return false;
        // 2. Must have gotten close to the formation slot (< 800 ft at some point)
        if (minDistToSlot_ > 800.0) return false;
        // 3. Must have reached "in position" (within 800 ft of slot)
        if (!inPosition_) return false;
        // 4. Speed error should be reasonable (< 90 kts — matching lead speed
        //    within a reasonable band). The wingman uses throttle-only speed
        //    control which has lag; high-T/W fighters overshoot speed during
        //    the initial closure and the dive-to-turn geometry adds speed.
        //    90 kts is ~20% of typical fighter cruise speed.
        if (maxSpeedErr_ > 90.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy mode; Get within 800ft of slot; "
               "Reach in-position (< 800ft); Speed error < 90kts; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Wingy mode:   %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot:     %.0f ft (need < 800) %s\n",
            minDistToSlot_, minDistToSlot_ < 800.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Reached in-position:  %s\n", inPosition_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max speed error:      %.1f kts (need < 90) %s\n",
            maxSpeedErr_, maxSpeedErr_ < 90.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double startOffset_{0.0};
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};

    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    bool enteredWingy_{false};
    bool inPosition_{false};
    bool hasNaN_{false};
};

// ===========================================================================
// Scenario: digi_formation
// ===========================================================================
class DigiFormationScenario : public ManeuverScenario {
public:
    DigiFormationScenario() : ManeuverScenario("digi_formation") {}

    std::string GetDescription() const override {
        return "Digi AI formation following: wingman follows a flight lead "
               "in Wedge formation. Tests AiFollowLead end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 10000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Phase 1: wingman starts 1000 ft offset, must close to formation slot.
        // 60 seconds gives enough time for all aircraft types to converge.
        // The 1000 ft offset is small enough for low-T/W aircraft (A-10, C-5)
        // to close, while still testing the formation-following logic.
        tests.push_back(std::make_unique<FormationFollowPhase>(
            "Formation follow (Wedge, 1000ft offset)", 60.0, alt, speed, 1000.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiFormation("digi_formation", []() {
    return std::make_unique<DigiFormationScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_formation() {}

} // namespace f4flight_test

// end
