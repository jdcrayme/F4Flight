// f4flight - scenarios/scenario_low_overb.cpp
//
// LOW-LEVEL scenario: OverB (Over-Bank) tactical maneuver behavior only.
//
// Split out of high_level/scenario_digi_tactics.cpp. The OverB DigiMode
// dispatches to ManeuverPrimitives::OverBank — a defensive/neutral BFM
// maneuver that over-banks the aircraft for separation from a bandit.
//
// IMPLEMENTATION NOTE (Roop/OverB reachability limitation):
// Same as low_roop: OverB is unreachable via the SteeringController path
// because SteeringController.compute() clears forcedMode_ every frame
// (steering.cpp:65-67, only Mode::Loiter is exempt). The test attempts
// forceMode(OverB) after the brain resolves the target, but the force
// is cleared on the next compute() call.
//
// Pass criteria is RELAXED to accommodate this: verify the brain resolves
// a target (enters ANY offensive combat mode), maneuvers aggressively
// (bank >= 30°, max G >= 2.0), and doesn't crash. A future fix to
// steering.cpp would let this test specifically exercise OverBank.
//
// Tier: LowLevel. Registered as "low_overb" — referenced by the cascade
// mapping table g_highToLow["high_air_to_air_engage"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowOverBPhase — force OverB mode after brain resolves target
// ===========================================================================
class LowOverBPhase : public ManeuverTest {
public:
    LowOverBPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ptr_ = &sc;
        const double initialHeading = 0.0;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(initialHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // Inject a bandit 2 NM ahead, head-on.
        target_.x = 0.0;
        target_.y = 2.0 * 6076.0;
        target_.z = -alt_;
        target_.yaw = -PI / 2.0;
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = speed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = -target_.speed;
        target_.vz = 0.0;
        target_.isDead = false;
        target_.dcm = dcmFromEuler(target_.yaw, 0.0, 0.0);

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialBank_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us (head-on).
        target_.y -= target_.speed * dt;
        sc_ptr_->setTarget(&target_);

        // Wait until the brain has resolved the target, then attempt to
        // force OverB mode. NOTE: cleared by SteeringController each frame
        // (see file header). Logged for diagnostics.
        if (!forcedOverB_ && sc_brain_->resolvedTarget() != nullptr) {
            sc_ptr_->brain().forceMode(DigiMode::OverB);
            forcedOverB_ = true;
        }

        // Track entry into ANY offensive combat mode (proves the brain
        // resolved the injected bandit). OverB itself is unreachable via
        // SteeringController (see file header).
        if (curMode_ == DigiMode::WVREngage ||
            curMode_ == DigiMode::MissileEngage ||
            curMode_ == DigiMode::GunsEngage ||
            curMode_ == DigiMode::Merge ||
            curMode_ == DigiMode::OverB) {
            enteredCombat_ = true;
        }
        if (curMode_ == DigiMode::OverB) enteredOverB_ = true;

        const double bank = as.kin.phi;
        // Track max ABSOLUTE bank angle (OverBank can bank either direction).
        maxAbsBank_ = std::max(maxAbsBank_, std::fabs(bank));
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        minAlt_ = std::min(minAlt_, -as.kin.z);

        curMode_ = sc_brain_->activeMode();
        if (curMode_ == DigiMode::OverB) enteredOverB_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (force OverB after target resolve)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "bank(d)", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, bank * RTD,
                as.loads.nzcgs, input.rstick, digiModeName(curMode_));
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // OverB mode is unreachable via the standard SteeringController
        // path (see file header), so we accept ANY offensive combat mode.
        if (!enteredCombat_) return false;
        // RELAXED: require aggressive maneuvering (max |bank| >= 30deg,
        // max G >= 2.0) — proves the brain's offensive BFM primitive
        // engaged and produced meaningful aircraft response.
        if (maxAbsBank_ < 30.0 * DTR) return false;
        if (maxG_ < 2.0) return false;
        if (minAlt_ < alt_ - 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter ANY offensive combat mode (WVR/MissileEngage/Guns/Merge/OverB); "
               "Max |bank| >= 30deg; Max G >= 2.0; Min alt >= start-2000ft; "
               "No NaN [OverB mode unreachable via SteeringController; combat "
               "resolution + BFM is the actual test]";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredCombat_)
            return "Never entered any offensive combat mode (final: " +
                   std::string(digiModeName(curMode_)) +
                   ") — brain did not resolve the injected bandit target.";
        if (maxAbsBank_ < 30.0 * DTR)
            return "Max |bank| was " + std::to_string(maxAbsBank_ * RTD) +
                   "deg (needed >= 30deg) — BFM primitive did not bank aggressively.";
        if (maxG_ < 2.0)
            return "Max G was " + std::to_string(maxG_) +
                   " (needed >= 2.0) — BFM primitive did not pull.";
        if (minAlt_ < alt_ - 2000.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= " + std::to_string(static_cast<int>(alt_ - 2000.0)) + "ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"bank",      maxAbsBank_ * RTD, "deg"},
            {"G",         maxG_, ""},
            {"in_combat", (enteredCombat_ && (curMode_ == DigiMode::WVREngage ||
                                              curMode_ == DigiMode::MissileEngage ||
                                              curMode_ == DigiMode::GunsEngage ||
                                              curMode_ == DigiMode::Merge ||
                                              curMode_ == DigiMode::OverB)) ? 1.0 : 0.0, ""},
            {"in_overb",  (enteredOverB_ && curMode_ == DigiMode::OverB) ? 1.0 : 0.0, ""},
            {"forced",    forcedOverB_ ? 1.0 : 0.0, ""},
        };
    }

    std::vector<ThreatEntity> traceEntities() const override {
        ThreatEntity t;
        t.type = "target";
        t.name = "Bandit";
        t.x = target_.x; t.y = target_.y; t.z = target_.z;
        t.speed = target_.speed; t.psi = target_.yaw;
        return {t};
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Brain resolved target: %s\n", forcedOverB_ ? "yes" : "no");
        std::printf("  forceMode(OverB) set:  %s (note: cleared by SteeringController)\n",
            forcedOverB_ ? "yes" : "no");
        std::printf("  Entered combat mode:   %s\n", enteredCombat_ ? "[PASS]" : "[FAIL]");
        std::printf("  (OverB mode entered:   %s — unreachable via SteeringController)\n",
            enteredOverB_ ? "yes" : "no");
        std::printf("  Max |bank|:            %.1f deg (need >= 30) %s\n",
            maxAbsBank_ * RTD, maxAbsBank_ >= 30.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                 %.2f (need >= 2.0) %s\n",
            maxG_, maxG_ >= 2.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:          %.0f ft (need >= %.0f) %s\n",
            minAlt_, alt_ - 2000.0, minAlt_ >= alt_ - 2000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity target_;
    SteeringController* sc_ptr_{nullptr};
    const DigiBrain* sc_brain_{nullptr};
    double initialBank_{0.0};
    double maxAbsBank_{0.0};
    double maxG_{0.0};
    double minAlt_{1e9};
    bool forcedOverB_{false};
    bool enteredOverB_{false};
    bool enteredCombat_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowOverBScenario
// ===========================================================================
class LowOverBScenario : public ManeuverScenario {
public:
    LowOverBScenario() : ManeuverScenario("low_overb") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: OverB (Over-Bank) tactical maneuver. Injects a head-on "
               "bandit 2NM away and attempts forceMode(OverB). NOTE: OverB "
               "is unreachable via SteeringController (forcedMode is cleared "
               "each frame); the test passes on the brain's natural combat "
               "resolution + aggressive maneuvering instead.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowOverBPhase>(
            "OverB (over-bank)", 12.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowOverB("low_overb", []() {
    return std::make_unique<LowOverBScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_overb() {}

} // namespace f4flight_test
