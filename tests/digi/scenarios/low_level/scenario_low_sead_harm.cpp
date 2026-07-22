// f4flight - scenarios/scenario_low_sead_harm.cpp
//
// LOW-LEVEL scenario: Suppression of Enemy Air Defenses (SEAD) using AGM-88 HARM.
// Tests all three HTS modes: Pre-Briefed (PB), Target-Of-Opportunity (TOO),
// and Self-Protect (SP).
//
// Tier: LowLevel. Registered as "low_sead_harm".

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/ground/ag_attack_phase.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>
#include <limits>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowSeadHarmPhase — single HARM/SEAD attack run parameterized by HtsMode.
// ===========================================================================
class LowSeadHarmPhase : public ManeuverTest {
public:
    LowSeadHarmPhase(const char* name, double duration,
                     double alt, double speed, double startOffsetNm, HtsMode mode)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          startOffsetNm_(startOffsetNm), htsMode_(mode) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north (pointing +Y)
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        const double startX = 0.0;
        const double startY = -startOffsetNm_ * 6076.0;
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(60.0);

        // Setup RWR truth and active SAM site
        truth_.clear();
        sam_.x = 0.0;
        sam_.y = 0.0;
        sam_.z = 0.0;
        sam_.vx = 0.0;
        sam_.vy = 0.0;
        sam_.vz = 0.0;
        sam_.yaw = 0.0;
        sam_.pitch = 0.0;
        sam_.roll = 0.0;
        sam_.speed = 0.0;
        sam_.isDead = false;
        sam_.isRadarEmitting = true;
        sam_.seekerType = DigiEntity::SeekerType::None;
        sam_.dcm = dcmFromEuler(0.0, 0.0, 0.0);

        // In SP mode, let the SAM site point South (toward us) so we are spiked!
        if (htsMode_ == HtsMode::SelfProtect) {
            sam_.yaw = -PI / 2.0; // South
        }

        truth_.entities.push_back(sam_);
        truth_.ids.push_back(101);
        truth_.firing.push_back(false);

        FrameInputs fi;
        fi.truth = &truth_;
        fi.injectedAgProfile = AgAttackProfile::SeadHarm;
        fi.injectedHtsMode = htsMode_;

        // In PB mode, we also inject a ground target coordinate directly
        if (htsMode_ == HtsMode::PreBriefed) {
            fi.injectedGroundTarget = &sam_;
        }

        // Setup some fuel / weapons state
        fi.fuelLbs = 5000.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2000.0;
        fi.fumesFuelLbs = 500.0;
        fi.winchester = false;

        // Reset the A/G attack phase counter to 0 to prevent state leakage from prior scenario phases!
        sc.brain().stateMutable().ag.agApproach = 0;

        sc.brain().setFrameInputs(fi);
        sc_brain_ = &sc.brain();

        // Setup simple SMS loadout with AGM-88 HARM
        sms_ = std::make_unique<StoresManagementSystem>();
        sms_->addHardpoint(3, WeaponType::Agm88, 2);
        sc.brain().setSMS(sms_.get());
    }

    ~LowSeadHarmPhase() override {
        if (sc_brain_) {
            sc_brain_->setSMS(nullptr);
        }
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double dx = 0.0 - as.kin.x;
        const double dy = 0.0 - as.kin.y;
        const double distToTarget = std::sqrt(dx * dx + dy * dy);
        minDistToTarget_ = std::min(minDistToTarget_, distToTarget);

        const double altAGL = -as.kin.z;
        minAlt_ = std::min(minAlt_, altAGL);

        const double pitchDeg = as.kin.theta * RTD;
        // Only track maxPitch_ during loft (Phase 1)
        if (sc_brain_ && sc_brain_->state().ag.agApproach == 1) {
            maxPitch_ = std::max(maxPitch_, pitchDeg);
        }

        const double G = as.loads.nzcgb;
        maxG_ = std::max(maxG_, G);

        if (sc_brain_->activeMode() == DigiMode::GroundMnvr) enteredGroundMnvr_ = true;
        if (input.releaseConsent) weaponReleased_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        curAlt_ = altAGL;
        curDist_ = distToTarget;
        curMode_ = sc_brain_->activeMode();
        curHeading_ = as.kin.sigma;

        // Keep RWR truth active each frame
        FrameInputs fi = sc_brain_->frameInputs();
        fi.truth = &truth_;
        sc_brain_->setFrameInputs(fi);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (HTS %s, start %.1fNM south at %.0fft)\n",
                    testName_.c_str(), htsModeName(htsMode_), startOffsetNm_, alt_);
                std::printf("%6s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "dTgt", "vcas", "pitch", "G", "mode");
            }
            std::printf("%6.1f %8.0f %8.0f %6.1f %6.1f %6.1f %6s\n",
                phaseTime_, altAGL, distToTarget, as.vcas,
                pitchDeg, G, digiModeName(curMode_));
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (weaponReleased_ && phaseTime_ > 10.0 && curMode_ == DigiMode::Waypoint);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredGroundMnvr_) return false;
        if (!weaponReleased_) return false;
        if (minAlt_ < 100.0) return false;

        // Mode-specific validation:
        if (htsMode_ == HtsMode::SelfProtect) {
            // SP: immediate level launch, followed by defensive beam / G pull
            if (maxPitch_ > 12.0) return false; // shouldn't do a full 20° loft
            if (maxG_ < 1.5) return false;      // should pull some Gs turning away
        } else {
            // PB and TOO: should perform a loft maneuver (> 15° pitch)
            if (maxPitch_ < 14.0) return false;
        }

        return true;
    }

    std::string criteria() const override {
        if (htsMode_ == HtsMode::SelfProtect) {
            return "Enter GroundMnvr; Release weapon; Min alt > 100ft; Max pitch < 12deg (level); Max G > 1.5 (evasion)";
        } else {
            return "Enter GroundMnvr; Release weapon; Min alt > 100ft; Max pitch > 14deg (loft)";
        }
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredGroundMnvr_)
            return "Never entered GroundMnvr mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!weaponReleased_)
            return "Never released weapon — SEAD profile did not reach release.";
        if (minAlt_ < 100.0)
            return "Min altitude " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed > 100ft) — aircraft crashed.";
        if (htsMode_ == HtsMode::SelfProtect) {
            if (maxPitch_ > 12.0) return "SP lofted " + std::to_string(maxPitch_) + "deg — expected level launch.";
            if (maxG_ < 1.5) return "SP evasion max G was " + std::to_string(maxG_) + " — expected > 1.5G.";
        } else {
            if (maxPitch_ < 14.0) return "Loft pitch was " + std::to_string(maxPitch_) + "deg — expected > 14deg.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"alt",       curAlt_,   "ft"},
            {"d_target",  curDist_,  "ft"},
            {"pitch",     maxPitch_, "deg"},
            {"G",         maxG_,     "g"},
            {"released",  weaponReleased_ ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        std::printf("  --- SEAD %s Summary ---\n", htsModeName(htsMode_));
        std::printf("  Entered GroundMnvr:  %s\n", enteredGroundMnvr_ ? "[PASS]" : "[FAIL]");
        std::printf("  Weapon released:     %s\n", weaponReleased_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.0f ft (need > 100) %s\n",
            minAlt_, minAlt_ > 100.0 ? "[PASS]" : "[FAIL]");
        if (htsMode_ == HtsMode::SelfProtect) {
            std::printf("  Evasion max G:       %.2f G (need > 1.5) %s\n",
                maxG_, maxG_ > 1.5 ? "[PASS]" : "[FAIL]");
            std::printf("  Max pitch:           %.1f deg (need < 12) %s\n",
                maxPitch_, maxPitch_ < 12.0 ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Loft max pitch:      %.1f deg (need > 14) %s\n",
                maxPitch_, maxPitch_ > 14.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    double startOffsetNm_{0.0};
    HtsMode htsMode_;
    TruthState truth_;
    DigiEntity sam_;
    std::unique_ptr<StoresManagementSystem> sms_;
    DigiBrain* sc_brain_{nullptr};
    double minDistToTarget_{1e9};
    double minAlt_{1e9};
    double maxPitch_{0.0};
    double maxG_{0.0};
    bool enteredGroundMnvr_{false};
    bool weaponReleased_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double curAlt_{0.0};
    double curDist_{0.0};
    double curHeading_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowSeadHarmScenario
// ===========================================================================
class LowSeadHarmScenario : public ManeuverScenario {
public:
    LowSeadHarmScenario() : ManeuverScenario("low_sead_harm") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: Suppression of Enemy Air Defenses (SEAD) using AGM-88 HARM. "
               "Tests Pre-Briefed (PB), Target-Of-Opportunity (TOO), and Self-Protect (SP) modes.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double seadAlt = 20000.0;
        const double seadSpeed = cornerSpeed > 0 ? cornerSpeed : 350.0;
        fm.init(ctx.cfg, seadAlt, seadSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowSeadHarmPhase>(
            "SEAD Pre-Briefed Mode", 90.0, seadAlt, seadSpeed, 30.0, HtsMode::PreBriefed));
        tests.push_back(std::make_unique<LowSeadHarmPhase>(
            "SEAD Target-Of-Opportunity Mode", 90.0, seadAlt, seadSpeed, 40.0, HtsMode::TargetOfOpportunity));
        tests.push_back(std::make_unique<LowSeadHarmPhase>(
            "SEAD Self-Protect Mode", 90.0, seadAlt, seadSpeed, 15.0, HtsMode::SelfProtect));
        return tests;
    }
};

static RegisterScenario g_registerLowSeadHarm("low_sead_harm", []() {
    return std::make_unique<LowSeadHarmScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_sead_harm() {}

} // namespace f4flight_test
