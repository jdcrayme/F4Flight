// f4flight - scenarios/scenario_digi_ground_attack_profiles.cpp
//
// Digi AI ground attack test: three delivery profiles in one scenario.
//
// Task ID: 15-a
//
// Expands the Task 11 dive-bomb scenario into a 3-phase scenario that
// exercises all three A/G attack profiles implemented in runGroundAttack():
//
//   Phase 1 "DiveBomb"      : the original Task 11 dive-bomb profile
//                             (approach -> dive -> pullout -> egress)
//   Phase 2 "LevelDelivery" : Task 15-a level bombing
//                             (approach -> level -> release -> egress)
//   Phase 3 "TossBomb"      : Task 15-a toss (loft) bombing
//                             (approach -> pull-up -> release -> egress)
//
// Each phase injects a ground target at the origin AND selects the profile
// via FrameInputs.injectedAgProfile. We verify that the brain:
//   * Enters GroundMnvr mode
//   * Releases the weapon (releaseConsent goes true at least once)
//   * Does NOT crash (min altitude > 500 ft for dive, > 200 ft for level/toss)
//   * Starts egressing (distance to target increases after release)
//
// The DiveBomb phase mirrors the original digi_ground_attack scenario (so the
// two scenarios overlap on phase 1 by design — the new scenario exists to
// verify the Level + Toss profiles and the dispatcher's backward compat).

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// GroundAttackProfilePhase — one A/G delivery profile, parameterized.
// ===========================================================================
class GroundAttackProfilePhase : public ManeuverTest {
public:
    GroundAttackProfilePhase(const char* name, double duration,
                              double alt, double speed,
                              double startOffsetNm,
                              AgAttackProfile profile,
                              double minAltFloor,
                              const char* profileLabel)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffsetNm_(startOffsetNm), profile_(profile),
          minAltFloor_(minAltFloor), profileLabel_(profileLabel) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at cruise altitude, heading north toward the target.
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Position the aircraft south of the target.
        const double startX = 0.0;
        const double startY = -startOffsetNm_ * 6076.0;  // startOffsetNm NM south
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);   // allow steep banks for attack maneuvers
        sc.setMaxGamma(60.0);  // allow steep dive/climb (toss needs 45 deg)

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

        // Inject the ground target + select the attack profile. The brain's
        // setFrameInputs() commits both into state_.ag.
        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
        fi.injectedAgProfile = profile_;
        sc.brain().setFrameInputs(fi);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Track distance to target.
        const double dx = groundTarget_.x - as.kin.x;
        const double dy = groundTarget_.y - as.kin.y;
        const double distToTarget = std::sqrt(dx * dx + dy * dy);
        minDistToTarget_ = std::min(minDistToTarget_, distToTarget);

        // Track altitude.
        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);

        // Track mode entry.
        if (sc_brain_->activeMode() == DigiMode::GroundMnvr) enteredGroundMnvr_ = true;

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
        curMode_ = sc_brain_->activeMode();
        curPhase_ = sc_brain_->state().ag.agApproach;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (%s profile, start %.1fNM south at %.0fft)\n",
                    testName_.c_str(), profileLabel_.c_str(),
                    startOffsetNm_, alt_);
                std::printf("%6s %8s %8s %6s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "dTgt", "vcas", "pstk", "thrt", "mode", "phase");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %8.0f %6.1f %6.2f %6.2f %6s %6d\n",
                phaseTime_, altAGL, distToTarget, as.vcas,
                input.pstick, input.throttle, modeBuf,
                sc_brain_->state().ag.agApproach);
            nextPrint_ += 10.0;
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
        // 3. Must not crash — min altitude above the profile's floor.
        if (minAlt_ < minAltFloor_) return false;
        // 4. Must start egressing (distance increasing after release).
        if (!egressing_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GroundMnvr; Release weapon; Min alt > " +
               std::to_string(static_cast<int>(minAltFloor_)) +
               "ft (no crash); Egress after release; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_) {
            return "Never entered GroundMnvr mode (final mode: " +
                   std::string(digiModeName(curMode_)) + ").";
        }
        if (!weaponReleased_) {
            return "Never released weapon — " + std::string(profileLabel_) +
                   " profile did not reach the release condition.";
        }
        if (minAlt_ < minAltFloor_) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed > " + std::to_string(static_cast<int>(minAltFloor_)) +
                   "ft) — aircraft crashed.";
        }
        if (!egressing_) {
            return "Never started egressing — aircraft did not fly away after release.";
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
            {"ag_profile", static_cast<double>(static_cast<int>(profile_)), ""},
        };
    }

    void Finish() const override {
        std::printf("  --- %s Summary ---\n", profileLabel_.c_str());
        std::printf("  Entered GroundMnvr:  %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:     %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        if (weaponReleased_) {
            std::printf("  Release altitude:    %.0f ft\n", releaseAlt_);
            std::printf("  Release distance:    %.0f ft\n", releaseDist_);
        }
        std::printf("  Min altitude:        %.0f ft (need > %.0f) %s\n",
            minAlt_, minAltFloor_, minAlt_ >= minAltFloor_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:        %.0f ft\n", maxAlt_);
        std::printf("  Egressing:           %s\n", egressing_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double startOffsetNm_{0.0};
    AgAttackProfile profile_{AgAttackProfile::DiveBomb};
    double minAltFloor_{200.0};
    std::string profileLabel_;

    DigiEntity groundTarget_;
    const DigiBrain* sc_brain_{nullptr};

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
// Scenario: digi_ground_attack_profiles — three delivery profiles in one run.
// ===========================================================================
class DigiGroundAttackProfilesScenario : public ManeuverScenario {
public:
    DigiGroundAttackProfilesScenario()
        : ManeuverScenario("digi_ground_attack_profiles") {}

    std::string GetDescription() const override {
        return "Digi AI ground attack — three delivery profiles: dive-bomb, "
               "level delivery, and toss (loft) bombing. Each phase injects a "
               "ground target + selects the profile via FrameInputs."
               "injectedAgProfile. Verifies runGroundAttack() dispatches on "
               "the profile and each profile releases the weapon and "
               "egresses without crashing.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        // Phase 1: DiveBomb — the Task 11 profile (12k ft start, 350 kts,
        // 6 NM south, dive-bomb release at ~4000 ft AGL). Min alt > 500 ft
        // to match the existing digi_ground_attack scenario's tolerance.
        const double diveAlt = 12000.0;
        const double diveSpeed = cornerSpeed > 0 ? cornerSpeed : 350.0;

        // Phase 2: LevelDelivery — 8000 ft approach, 400 kts, 8 NM south,
        // level run at 500 ft AGL, release over target. Min alt > 200 ft.
        const double levelAlt = 8000.0;
        const double levelSpeed = 400.0;

        // Phase 3: TossBomb — 1500 ft AGL approach (above ground-avoid
        // threshold), 400 kts, 8 NM south, 4G pull-up at 3 NM, release at
        // 45° pitch / 3000 ft AGL. Min alt > 200 ft.
        const double tossAlt = 1500.0;
        const double tossSpeed = 400.0;

        fm.init(ctx.cfg, diveAlt, diveSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<GroundAttackProfilePhase>(
            "Dive-bomb profile", 90.0,
            diveAlt, diveSpeed, 6.0,
            AgAttackProfile::DiveBomb, 500.0, "DiveBomb"));
        tests.push_back(std::make_unique<GroundAttackProfilePhase>(
            "Level delivery profile", 90.0,
            levelAlt, levelSpeed, 8.0,
            AgAttackProfile::LevelDelivery, 200.0, "LevelDelivery"));
        tests.push_back(std::make_unique<GroundAttackProfilePhase>(
            "Toss bombing profile", 110.0,
            tossAlt, tossSpeed, 8.0,
            AgAttackProfile::TossBomb, 200.0, "TossBomb"));
        return tests;
    }
};

static RegisterScenario g_registerDigiGroundAttackProfiles(
    "digi_ground_attack_profiles", []() {
    return std::make_unique<DigiGroundAttackProfilesScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_ground_attack_profiles() {}

} // namespace f4flight_test
