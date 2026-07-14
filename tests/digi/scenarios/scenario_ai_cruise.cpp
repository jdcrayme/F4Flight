// f4flight - scenarios/scenario_ai_cruise.cpp
//
// AI steering test: sustained level flight at high altitude. Uses
// sc.compute() — exercises HeadingAndAltitudeHold + MachHold at a cruise
// condition where Mach-dependent aero tables matter.
//
// This is NOT combat (ACM/weapons delivery is future work). It's a
// high-altitude cruise hold that exposes any thrust/drag imbalance at
// the aircraft's cruise Mach.
//
// Per-aircraft adaptation:
//   - Target speed = cfg.geometry.cornerVcas_kts (or 80% of maxStoreSpeed
//     if cornerVcas is unset)
//   - Altitude = 25000 ft (high-altitude cruise)
//   - maxGamma = 15°

#include "f4flight/flight/f4flight.h"
#include "f4flight/flight/atmosphere.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;

namespace f4flight_test {

class AICruisePhase : public ManeuverTest {
public:
    AICruisePhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), targetAlt_(alt), targetSpd_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(targetAlt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(targetSpd_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        // Cruise is level flight, so maxGamma rarely bites — but keep the
        // class-aware scaling consistent with ai_basic / ai_flightplan so
        // heavy aircraft don't get an aggressive gamma command during the
        // initial altitude capture transient.
        const bool heavy = isHeavy(fm.config());
        sc.setMaxGamma(heavy ? 10.0 : 15.0);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        const double alt = -as.kin.z;
        const double spd = as.vcas;
        altSamples_.emplace_back(phaseTime_, alt);
        spdSamples_.emplace_back(phaseTime_, spd);
        // Only count altitude capture if speed is ALSO close (within 50 kts).
        // Previously altCaptureTime was set immediately because the aircraft
        // starts at the target altitude, but the speed is still far from
        // target — the settling window then included the speed transient.
        if (altCaptureTime_ == 0.0 &&
            std::fabs(alt - targetAlt_) < 200.0 &&
            std::fabs(spd - targetSpd_) < 50.0) {
            altCaptureTime_ = phaseTime_;
        }
        if (altCaptureTime_ > 0.0 && std::fabs(spd - targetSpd_) < 25.0 &&
            speedCaptureTime_ == 0.0) {
            speedCaptureTime_ = phaseTime_;
        }
        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s\n", testName_.c_str());
                std::printf("  %6s  %8s  %8s  %8s  %8s  %6s  %6s  %5s\n",
                    "t(s)", "alt(ft)", "altErr", "vcas", "spdErr",
                    "bank(d)", "pitch(d)", "G");
            }
            std::printf("  %6.0f  %8.0f  %8.0f  %8.1f  %8.1f  %6.1f  %6.1f  %5.2f\n",
                phaseTime_, alt, targetAlt_ - alt, spd, targetSpd_ - spd,
                as.kin.phi * RTD, as.kin.theta * RTD, as.loads.nzcgs);
            nextPrint_ += 10.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ ||
               (altCaptureTime_ > 0.0 && speedCaptureTime_ > 0.0 &&
                phaseTime_ > std::max(altCaptureTime_, speedCaptureTime_) + 60.0);
    }

    bool IsPassed() const override {
        if (altCaptureTime_ == 0.0 || speedCaptureTime_ == 0.0) return false;
        // Settling window: last 30 s of phase. Tolerances:
        //   ALT_TOL = 200 ft  (high-altitude cruise, Mach-dependent aero)
        //   SPD_TOL = 25 kts
        const double tEnd = phaseTime_;
        const double tStart = tEnd - 30.0;
        double altMin = 1e9, altMax = -1e9, spdMin = 1e9, spdMax = -1e9;
        for (const auto& s : altSamples_) if (s.first >= tStart) { altMin = std::min(altMin, s.second); altMax = std::max(altMax, s.second); }
        for (const auto& s : spdSamples_) if (s.first >= tStart) { spdMin = std::min(spdMin, s.second); spdMax = std::max(spdMax, s.second); }
        // Also require the phase to have run at least 60 s past capture, so
        // the settling window is actually a SETTLING window (not the capture
        // transient itself). Previously the phase could finish at T+30 if
        // both captures happened at T+0.02, scoring the capture transient.
        if (phaseTime_ < std::max(altCaptureTime_, speedCaptureTime_) + 60.0)
            return false;
        return std::fabs(altMax - targetAlt_) < 200.0 && std::fabs(altMin - targetAlt_) < 200.0 &&
               std::fabs(spdMax - targetSpd_) < 25.0  && std::fabs(spdMin - targetSpd_) < 25.0;
    }

    std::string criteria() const override {
        return "Altitude capture achieved; Speed capture achieved; 60s past last capture; "
               "Altitude ±200ft over last 30s; Speed ±25kts over last 30s; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        if (altCaptureTime_ > 0.0) {
            std::printf("  Altitude capture at T+%.2f\n", altCaptureTime_);
            if (speedCaptureTime_ > 0.0) {
                std::printf("  Speed capture at T+%.2f\n", speedCaptureTime_);
            } else {
                std::printf("  Speed capture NOT achieved.  [FAIL]\n");
            }
        } else {
            std::printf("  Altitude capture NOT achieved.  [FAIL]\n");
        }
        std::printf("  Result: %s\n", IsPassed() ? "[PASS]" : "[FAIL]");
    }

private:
    double targetAlt_, targetSpd_;
    double nextPrint_{0.0};
    double altCaptureTime_{0.0};
    double speedCaptureTime_{0.0};
    std::vector<std::pair<double,double>> altSamples_;
    std::vector<std::pair<double,double>> spdSamples_;
};

class AICruiseScenario : public ManeuverScenario {
public:
    AICruiseScenario() : ManeuverScenario("ai_cruise") {}

    std::string GetDescription() const override {
        return "AI high-altitude cruise hold (25000 ft). Per-aircraft corner "
               "speed. Exercises Mach-dependent aero + thrust/drag balance.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cornerSpeed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        const double alt = 25000.0;

        // At altitude, VCAS < VT (true airspeed). The flight model's init()
        // takes true airspeed, but the AI targets VCAS. If we init at
        // cornerSpeed * KNOTS_TO_FTPSEC (treating it as true), the actual
        // VCAS will be much lower and MachHold will spend the first 30-60 s
        // accelerating. Instead, convert target VCAS to true airspeed at the
        // target altitude so the aircraft starts already at the target.
        // Approximate: VT = VCAS / sqrt(rho/rho0) for subsonic.
        // (Exact inverse of the VCAS formula; this is close enough for init.)
        double vt_ftps = cornerSpeed * KNOTS_TO_FTPSEC;
        {
            // Use the atmosphere model to get rho at altitude
            AtmosphereOutput a = computeAtmosphere(alt, vt_ftps,
                ctx.cfg.geometry.area_ft2, 1.0);
            const double rhoRatio = a.rho / RHOASL;
            if (rhoRatio > 0.01) {
                vt_ftps = cornerSpeed * KNOTS_TO_FTPSEC / std::sqrt(rhoRatio);
            }
        }

        fm.init(ctx.cfg, alt, vt_ftps, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<AICruisePhase>(
            "High-alt cruise hold 25000ft", 180.0, alt, cornerSpeed));
        return tests;
    }
};

static RegisterScenario g_registerAICruise("ai_cruise", []() {
    return std::make_unique<AICruiseScenario>();
});

extern "C" void f4flight_forceLink_scenario_ai_cruise() {}

} // namespace f4flight_test
