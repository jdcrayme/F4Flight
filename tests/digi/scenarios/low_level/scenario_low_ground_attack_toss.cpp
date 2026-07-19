// f4flight - scenarios/scenario_low_ground_attack_toss.cpp
//
// LOW-LEVEL scenario: toss (loft) bombing ground attack profile only.
//
// Split out of high_level/scenario_digi_ground_attack_profiles.cpp
// (GroundAttackProfilePhase with AgAttackProfile::TossBomb). Wraps a single
// toss-bomb attack run — approach → pull-up → release at 45° pitch / 3000ft
// AGL → egress.
//
// Pass criteria is RELAXED vs the parent scenario: drop the egress-after-
// release check (just verify enter GroundMnvr + release weapon + don't
// crash).
//
// Tier: LowLevel. Registered as "low_ground_attack_toss" — referenced by
// the cascade mapping table g_highToLow["high_air_to_ground"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowGroundAttackTossPhase — single toss-bomb profile
// ===========================================================================
class LowGroundAttackTossPhase : public ManeuverTest {
public:
    LowGroundAttackTossPhase(const char* name, double duration,
                             double alt, double speed, double startOffsetNm)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffsetNm_(startOffsetNm) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        fm.state().kin.x = 0.0;
        fm.state().kin.y = -startOffsetNm_ * 6076.0;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(60.0);  // toss needs steep pull-up

        groundTarget_.x = 0.0;
        groundTarget_.y = 0.0;
        groundTarget_.z = 0.0;
        groundTarget_.vx = groundTarget_.vy = groundTarget_.vz = 0.0;
        groundTarget_.yaw = groundTarget_.pitch = groundTarget_.roll = 0.0;
        groundTarget_.speed = 0.0;
        groundTarget_.isDead = false;
        groundTarget_.dcm = dcmFromEuler(0.0, 0.0, 0.0);

        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
        fi.injectedAgProfile = AgAttackProfile::TossBomb;
        sc.brain().setFrameInputs(fi);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double dx = groundTarget_.x - as.kin.x;
        const double dy = groundTarget_.y - as.kin.y;
        const double distToTarget = std::sqrt(dx * dx + dy * dy);
        minDistToTarget_ = std::min(minDistToTarget_, distToTarget);

        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);

        if (sc_brain_->activeMode() == DigiMode::GroundMnvr) enteredGroundMnvr_ = true;
        if (input.releaseConsent) weaponReleased_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curAlt_ = altAGL;
        curDist_ = distToTarget;
        curMode_ = sc_brain_->activeMode();

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (toss profile, start %.1fNM south at %.0fft)\n",
                    testName_.c_str(), startOffsetNm_, alt_);
                std::printf("%6s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "dTgt", "vcas", "pstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.0f %6.1f %6.2f %6s\n",
                phaseTime_, altAGL, distToTarget, as.vcas,
                input.pstick, digiModeName(curMode_));
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (weaponReleased_ && phaseTime_ > 10.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredGroundMnvr_) return false;
        if (!weaponReleased_) return false;
        if (minAlt_ < 100.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GroundMnvr; Release weapon; Min alt > 100ft (no crash); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_)
            return "Never entered GroundMnvr mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!weaponReleased_)
            return "Never released weapon — toss profile did not reach release.";
        if (minAlt_ < 100.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed > 100ft) — aircraft crashed.";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",       curAlt_,   "ft"},
            {"d_target",  curDist_,  "ft"},
            {"in_ground", (enteredGroundMnvr_ && curMode_ == DigiMode::GroundMnvr) ? 1.0 : 0.0, ""},
            {"released",  weaponReleased_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- TossBomb Summary ---\n");
        std::printf("  Entered GroundMnvr:  %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:     %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.0f ft (need > 100) %s\n",
            minAlt_, minAlt_ > 100.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:        %.0f ft\n", maxAlt_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    double startOffsetNm_{0.0};
    DigiEntity groundTarget_;
    const DigiBrain* sc_brain_{nullptr};
    double minDistToTarget_{1e9};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    bool enteredGroundMnvr_{false};
    bool weaponReleased_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double curAlt_{0.0};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowGroundAttackTossScenario
// ===========================================================================
class LowGroundAttackTossScenario : public ManeuverScenario {
public:
    LowGroundAttackTossScenario() : ManeuverScenario("low_ground_attack_toss") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: toss (loft) bombing ground attack profile. Aircraft starts "
               "8NM south at 1500ft AGL, pulls up into 4G climb, releases at "
               "the apex. Single phase, relaxed pass criteria.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double tossAlt = 1500.0;
        const double tossSpeed = 400.0;
        fm.init(ctx.cfg, tossAlt, tossSpeed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowGroundAttackTossPhase>(
            "Toss bombing profile", 110.0, tossAlt, tossSpeed, 8.0));
        return tests;
    }
};

static RegisterScenario g_registerLowGroundAttackToss("low_ground_attack_toss", []() {
    return std::make_unique<LowGroundAttackTossScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_ground_attack_toss() {}

} // namespace f4flight_test
