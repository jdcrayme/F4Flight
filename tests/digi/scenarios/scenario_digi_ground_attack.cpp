// f4flight - scenarios/scenario_digi_ground_attack.cpp
//
// Digi AI ground attack test: dive-bomb profile.
//
// Tests the runGroundAttack() implementation with a single ground target.
// The AI aircraft starts at cruise altitude, flies toward the target, dives
// to the release altitude, releases the weapon, pulls out, and egresses.
//
// Visualization (in the HTML report):
//   - Purple track: the ground target (auto-extracted as "target" threat type)
//   - White track: AI aircraft (the aircraft under test)

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// GroundAttackPhase — dive-bomb attack on a ground target
// ===========================================================================
class GroundAttackPhase : public ManeuverTest {
public:
    GroundAttackPhase(const char* name, double duration,
                      double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at cruise altitude, heading north toward the target.
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Position the aircraft 6 NM south of the target.
        const double startX = 0.0;
        const double startY = -6.0 * 6076.0;  // 6 NM south
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);  // allow steep banks for attack maneuvers
        sc.setMaxGamma(45.0); // allow steep dive/climb

        // --- Set up the ground target ---
        // Target at origin (0, 0), at ground level (z = 0).
        groundTarget_.x = 0.0;
        groundTarget_.y = 0.0;
        groundTarget_.z = 0.0;  // ground level
        groundTarget_.vx = 0.0;
        groundTarget_.vy = 0.0;
        groundTarget_.vz = 0.0;
        groundTarget_.yaw = 0.0;
        groundTarget_.pitch = 0.0;
        groundTarget_.roll = 0.0;
        groundTarget_.speed = 0.0;
        groundTarget_.isDead = false;
        groundTarget_.dcm = dcmFromEuler(0.0, 0.0, 0.0);

        // Inject the ground target.
        FrameInputs fi;
        fi.injectedGroundTarget = &groundTarget_;
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
                std::printf("\n%s (dive-bomb on ground target, start 6NM south)\n",
                    testName_.c_str());
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
            nextPrint_ += 5.0;
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
        // 3. Must release at a reasonable altitude (1000-6000 ft AGL).
        if (releaseAlt_ < 1000.0 || releaseAlt_ > 6000.0) return false;
        // 4. Must pull out (min altitude > 500 ft — didn't crash).
        if (minAlt_ < 500.0) return false;
        // 5. Must start egressing (distance increasing after release).
        if (!egressing_) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter GroundMnvr; Release weapon; Release alt 1000-6000ft; "
               "Min alt > 500ft (no crash); Egress after release; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_) {
            return "Never entered GroundMnvr mode (final mode: " +
                   std::string(digiModeName(curMode_)) + ").";
        }
        if (!weaponReleased_) {
            return "Never released weapon — dive profile did not reach release altitude.";
        }
        if (releaseAlt_ < 1000.0 || releaseAlt_ > 6000.0) {
            return "Release altitude was " + std::to_string(static_cast<int>(releaseAlt_)) +
                   "ft (needed 1000-6000ft).";
        }
        if (minAlt_ < 500.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed > 500ft) — aircraft crashed during pullout.";
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
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered GroundMnvr:  %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:     %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        if (weaponReleased_) {
            std::printf("  Release altitude:    %.0f ft (need 1000-6000) %s\n",
                releaseAlt_,
                (releaseAlt_ >= 1000.0 && releaseAlt_ <= 6000.0) ? "[PASS]" : "[FAIL]");
            std::printf("  Release distance:    %.0f ft\n", releaseDist_);
        }
        std::printf("  Min altitude:        %.0f ft (need > 500) %s\n",
            minAlt_, minAlt_ >= 500.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Egressing:           %s\n", egressing_ ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity groundTarget_;
    const DigiBrain* sc_brain_{nullptr};

    double minDistToTarget_{1e9};
    double minAlt_{1e9};
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
// Scenario: digi_ground_attack
// ===========================================================================
class DigiGroundAttackScenario : public ManeuverScenario {
public:
    DigiGroundAttackScenario() : ManeuverScenario("digi_ground_attack") {}

    std::string GetDescription() const override {
        return "Digi AI ground attack: dive-bomb profile. The AI aircraft "
               "starts at cruise altitude 6NM south of a ground target, dives "
               "to the release altitude, releases the weapon, pulls out, and "
               "egresses. Tests runGroundAttack() end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 12000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 350.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<GroundAttackPhase>(
            "Dive-bomb ground attack", 90.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiGroundAttack("digi_ground_attack", []() {
    return std::make_unique<DigiGroundAttackScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_ground_attack() {}

} // namespace f4flight_test
