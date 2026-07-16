// f4flight - scenarios/scenario_digi_formation.cpp
//
// Digi AI formation following test: 4-ship formation.
//
// This scenario verifies the AiFollowLead implementation with a 4-ship
// wedge formation:
//   - Slot 0: Flight lead (kinematic — flies straight at constant speed)
//   - Slot 1: Wingman 1 (AI aircraft with full flight model) — right wing
//   - Slot 2: Ghost wingman 2 (kinematic — flies perfect formation) — left wing
//   - Slot 3: Ghost wingman 3 (kinematic — flies perfect formation) — trail
//
// The AI wingman (slot 1) starts offset from its formation slot and must
// close to the slot and maintain formation. The ghost wingmen fly perfect
// formation (kinematic, no flight model) to show what a 4-ship looks like.
//
// Visualization (in the HTML report):
//   - Green track + circle: flight lead
//   - Blue diamond: desired slot position for the AI wingman
//   - Cyan tracks + circles: ghost wingmen (slots 2, 3)
//   - White track: AI wingman (the aircraft under test)

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
// FormationFollowPhase — 4-ship formation following
// ===========================================================================
class FormationFollowPhase : public ManeuverTest {
public:
    FormationFollowPhase(const char* name, double duration,
                         double alt, double speed, double startOffsetFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffset_(startOffsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Place the AI wingman (slot 1) offset from its desired position.
        fm.state().kin.x = startOffset_;
        fm.state().kin.y = -startOffset_;
        fm.state().kin.z = -(alt_ - 200.0);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // --- Set up the 4-ship formation ---
        // Lead (slot 0) flies north at constant speed/altitude.
        const double leadVt = speed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;  // north
        lead_.pitch = 0.0;
        lead_.roll = 0.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        // Ghost wingmen (slots 2, 3) start in their perfect formation positions.
        // Slot 2: left wing (relAz=-135°, range=1000) — behind-left
        // Slot 3: trail (relAz=180°, range=2000) — directly behind
        // Uses the ADAPTED formula (sigma - relAz) matching wingman_ai.cpp.
        const double leadSigma = lead_.yaw;
        const auto slot2 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 2);
        const auto slot3 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 3);
        ghost2_.x = lead_.x + slot2.range * std::cos(leadSigma - slot2.relAz);
        ghost2_.y = lead_.y + slot2.range * std::sin(leadSigma - slot2.relAz);
        ghost2_.z = lead_.z;
        ghost2_.speed = leadVt;
        ghost2_.yaw = leadSigma;
        ghost3_.x = lead_.x + slot3.range * std::cos(leadSigma - slot3.relAz);
        ghost3_.y = lead_.y + slot3.range * std::sin(leadSigma - slot3.relAz);
        ghost3_.z = lead_.z;
        ghost3_.speed = leadVt;
        ghost3_.yaw = leadSigma;

        // Configure the AI wingman: slot 1, Wedge formation.
        sc.setWingman(1, 1);  // leadId=1, slot=1
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the lead forward
        lead_.y += lead_.speed * dt;

        // Move ghost wingmen to maintain perfect formation relative to lead.
        // Uses the ADAPTED formula (sigma - relAz) matching wingman_ai.cpp.
        const double leadSigma = lead_.yaw;
        const auto slot2 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 2);
        const auto slot3 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 3);
        ghost2_.x = lead_.x + slot2.range * std::cos(leadSigma - slot2.relAz);
        ghost2_.y = lead_.y + slot2.range * std::sin(leadSigma - slot2.relAz);
        ghost2_.z = lead_.z;
        ghost3_.x = lead_.x + slot3.range * std::cos(leadSigma - slot3.relAz);
        ghost3_.y = lead_.y + slot3.range * std::sin(leadSigma - slot3.relAz);
        ghost3_.z = lead_.z;

        // Track the AI wingman's distance to its desired formation slot (slot 1).
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
        // Speed error: compare TRUE airspeeds (wingman TAS vs lead TAS), not
        // CAS vs cornerVcas. At altitude CAS < TAS, so the old metric reported
        // a huge phantom error even when the wingman matched the lead's speed.
        const double wingTasKts = as.kin.vt / KNOTS_TO_FTPSEC;
        const double leadTasKts = lead_.speed / KNOTS_TO_FTPSEC;
        maxSpeedErr_ = std::max(maxSpeedErr_, std::fabs(wingTasKts - leadTasKts));

        // Track SUSTAINED in-position time (not just momentary). The wingman
        // must hold formation, not just sweep through the slot once.
        if (distToSlot < kSustainedProximityFt) {
            timeInProximity_ += dt;
        }
        // Track final-window proximity (last 10 seconds).
        if (phaseTime_ > maxTime_ - 10.0) {
            finalWindowTime_ += dt;
            if (distToSlot < kSustainedProximityFt) finalWindowInPos_ += dt;
        }

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (sc_brain_->state().formation.wingman.inPosition) inPosition_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        // Store current slot position for traceEntities()
        curSlotX_ = desX;
        curSlotY_ = desY;
        curSlotZ_ = desZ;
        // Per-frame sample data (for trace)
        curDistToSlot_ = distToSlot;
        curSpeedErr_ = std::fabs(wingTasKts - leadTasKts);
        curInPosition_ = sc_brain_->state().formation.wingman.inPosition;
        curMode_ = sc_brain_->activeMode();

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (4-ship Wedge, AI=slot 1, start %dft offset)\n",
                    testName_.c_str(), static_cast<int>(startOffset_));
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

    // Provide custom trace entities for visualization:
    //   - "lead" (green): the flight lead — auto-extracted by the framework
    //     via frameInputs().injectedLead (set by sc.setLead(&lead_) in Init).
    //     Not duplicated here.
    //   - "slot" (blue diamond): the AI wingman's desired formation slot
    //   - "wingman" (cyan): the two ghost wingmen
    std::vector<ThreatEntity> traceEntities() const override {
        return {
            {"slot",    curSlotX_, curSlotY_, curSlotZ_, 0.0},
            {"wingman", ghost2_.x, ghost2_.y, ghost2_.z, ghost2_.speed},
            {"wingman", ghost3_.x, ghost3_.y, ghost3_.z, ghost3_.speed},
        };
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        // Must close to the slot at least once.
        if (minDistToSlot_ > 300.0) return false;
        // Must reach the brain's in-position flag at least once.
        if (!inPosition_) return false;
        // Must SUSTAIN proximity: hold within 500 ft for >= 15 seconds total.
        // This catches the oscillation bug — a wingman that sweeps through
        // the slot once and then oscillates 1000+ ft away does NOT pass.
        if (timeInProximity_ < 15.0) return false;
        // Must be stable in the final 10 seconds: >= 50% of the final window
        // within 500 ft. This proves the wingman has settled, not just passed
        // through briefly at the end.
        if (finalWindowTime_ > 0.0 &&
            finalWindowInPos_ / finalWindowTime_ < 0.5) return false;
        // Speed match (TAS vs TAS, not CAS vs cornerVcas).
        if (maxSpeedErr_ > 30.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Wingy; Close to <300ft of slot; Reach in-position; "
               "Sustain <500ft for >=15s; >=50% of final 10s within 500ft; "
               "TAS match <30kts; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredWingy_) {
            return "Never entered Wingy mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   ") — formation role was not activated.";
        }
        if (minDistToSlot_ > 300.0) {
            return "Min distance to slot was " +
                   std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (needed < 300ft) — wingman never closed to formation position.";
        }
        if (!inPosition_) {
            return "Never reached in-position flag (got within " +
                   std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft of slot, but brain never set inPosition=true).";
        }
        if (timeInProximity_ < 15.0) {
            return "Only held within 500ft of slot for " +
                   std::to_string(static_cast<int>(timeInProximity_)) +
                   "s (needed >= 15s) — wingman is oscillating, not holding formation.";
        }
        if (finalWindowTime_ > 0.0 &&
            finalWindowInPos_ / finalWindowTime_ < 0.5) {
            return "In final 10s, only " +
                   std::to_string(static_cast<int>(
                       100.0 * finalWindowInPos_ / finalWindowTime_)) +
                   "% of time within 500ft (needed >= 50%) — wingman did not settle.";
        }
        if (maxSpeedErr_ > 30.0) {
            return "Max TAS error was " + std::to_string(static_cast<int>(maxSpeedErr_)) +
                   "kts (needed < 30kts) — wingman did not match lead's true airspeed.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",     curDistToSlot_, "ft"},
            {"spd_err",    curSpeedErr_,   "kts"},
            {"in_pos",     curInPosition_ ? 1.0 : 0.0, ""},
            {"in_wingy",   (enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Wingy mode:   %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot:     %.0f ft (need < 300) %s\n",
            minDistToSlot_, minDistToSlot_ < 300.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Reached in-position:  %s\n", inPosition_ ? "[PASS]" : "[FAIL]");
        std::printf("  Time within 500ft:    %.1f s (need >= 15) %s\n",
            timeInProximity_, timeInProximity_ >= 15.0 ? "[PASS]" : "[FAIL]");
        if (finalWindowTime_ > 0.0) {
            std::printf("  Final 10s in-pos:     %.0f%% (need >= 50) %s\n",
                100.0 * finalWindowInPos_ / finalWindowTime_,
                finalWindowInPos_ / finalWindowTime_ >= 0.5 ? "[PASS]" : "[FAIL]");
        }
        std::printf("  Max TAS error:        %.1f kts (need < 30) %s\n",
            maxSpeedErr_, maxSpeedErr_ < 30.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    static constexpr double kSustainedProximityFt = 500.0;

    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double startOffset_{0.0};
    DigiEntity lead_;
    DigiEntity ghost2_;  // slot 2 (left wing, kinematic)
    DigiEntity ghost3_;  // slot 3 (trail, kinematic)
    const DigiBrain* sc_brain_{nullptr};

    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    double timeInProximity_{0.0};
    double finalWindowTime_{0.0};
    double finalWindowInPos_{0.0};
    bool enteredWingy_{false};
    bool inPosition_{false};
    bool hasNaN_{false};

    // Current desired slot position (for traceEntities)
    mutable double curSlotX_{0.0}, curSlotY_{0.0}, curSlotZ_{0.0};

    // Per-frame sample data (updated in Evaluate, read in traceSamples)
    double curDistToSlot_{0.0};
    double curSpeedErr_{0.0};
    bool curInPosition_{false};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// Scenario: digi_formation
// ===========================================================================
class DigiFormationScenario : public ManeuverScenario {
public:
    DigiFormationScenario() : ManeuverScenario("digi_formation") {}

    std::string GetDescription() const override {
        return "Digi AI 4-ship formation following: AI wingman (slot 1) follows "
               "a flight lead in Wedge formation. Ghost wingmen (slots 2, 3) "
               "fly perfect formation for visualization. Tests AiFollowLead "
               "end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 10000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<FormationFollowPhase>(
            "4-ship Wedge formation (AI=slot 1)", 90.0, alt, speed, 1000.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiFormation("digi_formation", []() {
    return std::make_unique<DigiFormationScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_formation() {}

} // namespace f4flight_test
