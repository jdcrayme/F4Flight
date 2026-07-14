// f4flight - scenarios/scenario_ai_basic.cpp
//
// AI steering test: level flight, staged climb, level, staged descent.
// Uses sc.compute() — exercises the full digi AI steering path:
//   HeadingAndAltitudeHold → AltitudeHold/LevelTurn → GammaHold → SetPstick
//   MachHold (throttle)
//
// Per-aircraft adaptation:
//   - Target speed = cfg.geometry.cornerVcas_kts (NOT hardcoded 380 kts)
//   - Climb is STAGED (1500 → 5000 → 10000 → 15000) not one 10,000-ft jump.
//     FreeFalcon's AI navigates at cruise altitude; a 60° gamma command from
//     a 10,000-ft altitude error saturates GammaHold and the aircraft zooms,
//     bleeds airspeed, and stalls. Staged climbs keep the gamma command in
//     the linear range.
//   - maxGamma = 15° (sustained climb envelope, vs. FreeFalcon's 60° which
//     is for the attitude-hold autopilot, not navigation)

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <string>

using namespace f4flight;

namespace manuver_test {

// ===========================================================================
// AILevelPhase — hold altitude + heading, let the AI fly it
// ===========================================================================
class AILevelPhase : public ManeuverTest {
protected:
    double nextPrint_{0.0};
    double ALT_TOL{150.0};
    double SPD_TOL{25.0};
    double HDG_TOL{10.0};  // degrees — heading drift tolerance
    const double kSettle{30.0};

    std::vector<std::pair<double,double>> altSamples_;
    std::vector<std::pair<double,double>> spdSamples_;
    std::vector<std::pair<double,double>> hdgSamples_;

    double targetAlt_{0.0};
    double targetSpd_{0.0};
    double targetHdg_{0.0};
    double startAlt_{0.0};
    double altCaptureTime_{0.0};
    double speedCaptureTime_{0.0};
    bool isFirstFrame_{true};
    bool isHeavy_{false};

    void windowMinMax(const std::vector<std::pair<double,double>>& samples,
                      double tEnd, double window,
                      double& outMin, double& outMax) const {
        outMin = std::numeric_limits<double>::max();
        outMax = std::numeric_limits<double>::lowest();
        const double tStart = tEnd - window;
        for (const auto& s : samples) {
            if (s.first < tStart) continue;
            outMin = std::min(outMin, s.second);
            outMax = std::max(outMax, s.second);
        }
        if (outMin > outMax) { outMin = outMax = samples.empty() ? 0.0 : samples.back().second; }
    }

    bool checkAltPass() const {
        if (altCaptureTime_ == 0.0) return false;
        double mn, mx;
        windowMinMax(altSamples_, phaseTime_, kSettle, mn, mx);
        return std::fabs(mx - targetAlt_) < ALT_TOL &&
               std::fabs(mn - targetAlt_) < ALT_TOL;
    }
    bool checkSpdPass() const {
        if (speedCaptureTime_ == 0.0) return false;
        double mn, mx;
        windowMinMax(spdSamples_, phaseTime_, kSettle, mn, mx);
        return std::fabs(mx - targetSpd_) < SPD_TOL &&
               std::fabs(mn - targetSpd_) < SPD_TOL;
    }
    bool checkHdgPass() const {
        // Heading must stay within HDG_TOL of target over the settle window.
        // Catches a regression where the AI holds alt+speed but drifts heading.
        double mn, mx;
        windowMinMax(hdgSamples_, phaseTime_, kSettle, mn, mx);
        // Wrap-aware comparison: convert heading error to [-180, 180].
        auto hdgErr = [](double a, double b) {
            double e = a - b;
            while (e >  180.0) e -= 360.0;
            while (e < -180.0) e += 360.0;
            return std::fabs(e);
        };
        return hdgErr(mn, targetHdg_) < HDG_TOL &&
               hdgErr(mx, targetHdg_) < HDG_TOL;
    }

    void printRow(const AircraftState& as, const PilotInput& input) const {
        double alt = -as.kin.z;
        double altErr = targetAlt_ - alt;
        double spdErr = targetSpd_ - as.vcas;
        std::printf("%6.0f %8.0f %8.0f %8.1f %8.1f %8.2f %8.2f %6.1f %6.1f %5.2f\n",
            phaseTime_, alt, altErr,
            as.vcas, spdErr,
            input.throttle, input.pstick,
            as.kin.phi * RTD, as.kin.theta * RTD,
            as.loads.nzcgs);
    }

public:
    AILevelPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(0.0);
        // Use the aircraft's corner speed as the MachHold target, matching
        // what FreeFalcon's AI does (af->CornerVcas()).
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        // Gamma envelope scales with aircraft class. Heavy aircraft (B-52,
        // C-130) have lower T/W at altitude and cannot sustain speed at 15°
        // gamma — they bleed to 160-170 kts and stall. A 10° envelope keeps
        // climbs sustainable (~750 ft/min at 250 kts) while still allowing
        // descents to dissipate speed without overspeeding. Fighters retain
        // the 15° navigation envelope.
        const bool heavy = isHeavy(fm.config());
        isHeavy_ = heavy;
        sc.setMaxGamma(heavy ? 10.0 : 15.0);
        sc.setTurnG(heavy ? 1.3 : 2.0);
        // Heavy aircraft (C-130, B-52H) have low T/W at altitude and bleed
        // speed during sustained climbs. The C-130 at 15000 ft / 250 kts
        // can't sustain level flight — thrust < drag. Accept a wider speed
        // band (50 kts vs 25 kts) so the test doesn't fail on physics. The
        // altitude tolerance stays tight — that's what the AI is for.
        SPD_TOL = heavy ? 50.0 : 25.0;
    }

    virtual bool IsFinished() const {
        return phaseTime_ >= maxTime_ ||
            (altCaptureTime_ > 0 && speedCaptureTime_ > 0 &&
             phaseTime_ > altCaptureTime_ + 60.0 && phaseTime_ > speedCaptureTime_ + 60.0);
    }

    virtual bool IsPassed() const { return checkAltPass() && checkSpdPass() && checkHdgPass(); }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        double alt = -as.kin.z;
        double spd = as.vcas;

        if (isFirstFrame_) {
            startAlt_ = alt;
            isFirstFrame_ = false;
        }

        if (altCaptureTime_ == 0.0) {
            if (std::fabs(startAlt_ - targetAlt_) < ALT_TOL ||
                (startAlt_ < targetAlt_ && alt >= targetAlt_) ||
                (startAlt_ > targetAlt_ && alt <= targetAlt_)) {
                altCaptureTime_ = phaseTime_;
            }
        }

        if (speedCaptureTime_ == 0.0 && altCaptureTime_ > 0) {
            if (std::fabs(spd - targetSpd_) < SPD_TOL) {
                speedCaptureTime_ = phaseTime_;
            }
        }

        altSamples_.emplace_back(phaseTime_, alt);
        spdSamples_.emplace_back(phaseTime_, spd);
        hdgSamples_.emplace_back(phaseTime_, as.kin.sigma * RTD);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %8s %8s %8s %6s %6s %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "spdErr",
                    "throt", "pstick", "bank(d)", "pitch(d)", "G");
            }
            printRow(as, input);
            nextPrint_ += 10.0;
        }
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        if (altCaptureTime_ > 0.0) {
            double altMin, altMax;
            windowMinMax(altSamples_, phaseTime_, kSettle, altMin, altMax);
            std::printf("  Altitude capture at T+%.2f \t", altCaptureTime_);
            std::printf("  ALT (last %.0fs): +%.0f ft, -%.0f ft %s\n",
                kSettle,
                std::fabs(altMax - targetAlt_),
                std::fabs(altMin - targetAlt_),
                checkAltPass() ? "[PASS]" : "[FAIL]");
            if (speedCaptureTime_ > 0.0) {
                double spdMin, spdMax;
                windowMinMax(spdSamples_, phaseTime_, kSettle, spdMin, spdMax);
                std::printf("  Speed capture at T+%.2f \t", speedCaptureTime_);
                std::printf("  SPD (last %.0fs): +%.1f kts, -%.1f kts %s\n",
                    kSettle,
                    std::fabs(spdMax - targetSpd_),
                    std::fabs(spdMin - targetSpd_),
                    checkSpdPass() ? "[PASS]" : "[FAIL]");
            } else {
                std::printf("  Speed capture NOT achieved.  [FAIL]\n");
            }
            double hdgMin, hdgMax;
            windowMinMax(hdgSamples_, phaseTime_, kSettle, hdgMin, hdgMax);
            std::printf("  HDG (last %.0fs): %.1f..%.1f deg (target %.1f, tol ±%.0f) %s\n",
                kSettle, hdgMin, hdgMax, targetHdg_, HDG_TOL,
                checkHdgPass() ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Altitude capture NOT achieved.  [FAIL]\n");
        }
    }
};

// ===========================================================================
// AIBasicScenario — staged level/climb/level/descent using per-aircraft speed
// ===========================================================================
class AIBasicScenario : public ManeuverScenario {
public:
    AIBasicScenario() : ManeuverScenario("ai_basic") {}

    std::string GetDescription() const override {
        return "AI steering: level, staged climb, level, staged descent. "
               "Per-aircraft corner speed. Exercises GammaHold + MachHold.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        // Per-aircraft: use the aircraft's own corner speed.
        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double cruiseAlt = 5000;
        // highAlt not needed: scenario climbs directly to 15000 ft via the
        // staged level phase below. Kept the 5000 -> 10000 -> 15000 ladder
        // to avoid saturating GammaHold on heavy / low-thrust aircraft.

        fm.init(ctx.cfg, cruiseAlt, cornerSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        const bool heavy = isHeavy(ctx.cfg);
        if (heavy) {
            // Heavy aircraft (C-130, B-52H) have low T/W at altitude. The
            // C-130 at 15000 ft / 250 kts can't sustain level flight (thrust
            // < drag). Cap the climb ladder at 10000 ft where the aircraft
            // can still maintain speed. Same 6-phase structure (level →
            // staged climb → level → staged descent), just lower altitudes.
            tests.push_back(std::make_unique<AILevelPhase>("Level hold 5000ft",   90.0,  5000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Climb to 7500ft",    150.0,  7500.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Climb to 10000ft",   150.0, 10000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Level hold 10000ft",  90.0, 10000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Descend to 7500ft",  150.0,  7500.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Descend to 5000ft",  150.0,  5000.0, cornerSpeed));
        } else {
            // Staged: level at 5000 → climb to 10000 → climb to 15000 → level → descend to 10000 → descend to 5000
            tests.push_back(std::make_unique<AILevelPhase>("Level hold 5000ft",   90.0,  5000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Climb to 10000ft",   150.0, 10000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Climb to 15000ft",   150.0, 15000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Level hold 15000ft",  90.0, 15000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Descend to 10000ft", 150.0, 10000.0, cornerSpeed));
            tests.push_back(std::make_unique<AILevelPhase>("Descend to 5000ft",  150.0,  5000.0, cornerSpeed));
        }
        return tests;
    }
};

static RegisterScenario g_registerAIBasic("ai_basic", []() {
    return std::make_unique<AIBasicScenario>();
});

extern "C" void f4flight_forceLink_scenario_ai_basic() {}

} // namespace manuver_test
