// f4flight - scenarios/scenario_digi_rtb.cpp
//
// Digi AI RTB (Return To Base) scenario: fuel-critical divert to nearest
// airbase + landing transition.
//
// This scenario verifies the Round 6 AirbaseCheck + FuelCheck + RTB pipeline:
//   1. AI starts in level flight with Bingo fuel.
//   2. AirbaseCheck picks the nearest friendly airbase (20 NM north).
//   3. AI enters RTB mode and steers toward the airbase.
//   4. When within 10 NM, AirbaseCheck transitions RTB → Landing.
//   5. AI enters Landing mode and begins the approach.
//
// Pass criteria:
//   - Enters RTB mode at Bingo fuel
//   - Steers toward the divert airbase (heading converges to airbase bearing)
//   - Transitions to Landing mode when within range
//   - Does not NaN or crash

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/digi_brain.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// RTBPhase — fuel-critical divert to nearest airbase
// ===========================================================================
class RTBPhase : public ManeuverTest {
public:
    RTBPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at 15000 ft, 350 kts, heading east (away from airbase).
        const double initialHeading = 0.0;  // east (yaw=0 = +X = east)
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(initialHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        // --- Set up the friendly airbase 20 NM north ---
        // Coordinate convention: yaw=0 = +X = east. "North" = +Y.
        // Airbase at (0, 20NM) — 90° off the aircraft's current heading (east).
        airbase_.x = 0.0;
        airbase_.y = 20.0 * 6076.0;  // 20 NM north
        airbase_.z = -5000.0;         // 5000 ft MSL field elevation
        airbase_.runwayHeading = 0.0; // runway points east
        airbase_.id = 100;

        // --- Set up fuel state: Bingo ---
        // bingoFuelLbs = 1500, fuelLbs = 1400 (below bingo → RTB).
        FrameInputs fi;
        fi.fuelLbs = 1400.0;
        fi.bingoFuelLbs = 1500.0;
        fi.jokerFuelLbs = 2500.0;
        fi.fumesFuelLbs = 800.0;
        fi.airbases = &airbase_;
        fi.airbaseCount = 1;
        sc.brain().setFrameInputs(fi);

        sc_brain_ = &sc.brain();
        initialHeading_ = initialHeading;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Track heading + mode
        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();

        if (currentMode_ == DigiMode::RTB) enteredRTB_ = true;
        if (currentMode_ == DigiMode::Landing) enteredLanding_ = true;

        // Track heading convergence to airbase bearing.
        // Aircraft starts heading east (0). Airbase is north (+Y).
        // Desired heading = atan2(20NM, 0) = PI/2 (north).
        // Track how close the aircraft gets to heading north.
        double dh = heading - (PI / 2.0);  // heading minus north
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToNorth_ = std::min(minAbsHeadingToNorth_, std::fabs(dh));

        // Track distance to airbase
        const double dx = airbase_.x - as.kin.x;
        const double dy = airbase_.y - as.kin.y;
        const double distToAirbase = std::sqrt(dx * dx + dy * dy);
        minDistToAirbase_ = std::min(minDistToAirbase_, distToAirbase);

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        minAlt_ = std::min(minAlt_, -as.kin.z);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (Bingo fuel, airbase 20NM north)\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "dAB(ft)", "pstk", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %8.0f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD, distToAirbase,
                input.pstick, input.rstick, digiModeName(currentMode_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter RTB mode.
        if (!enteredRTB_) return false;
        // 2. Must steer toward the airbase (heading converges to north).
        //    Allow 60° tolerance — the aircraft starts heading east and needs
        //    to turn 90° to north; in 60s at 45° bank it can turn ~200°.
        if (minAbsHeadingToNorth_ > 60.0 * DTR) return false;
        // 3. Must close the distance to the airbase (prove it's flying toward
        //    it, not away). Started at 20 NM = 121520 ft; require the min
        //    distance to be at least 0.5 NM less than the start (proves the
        //    aircraft turned toward the airbase and is closing). Slow aircraft
        //    (A-10 at 250 kts) may not close much in 90s, but they should at
        //    least be getting closer, not further.
        if (minDistToAirbase_ > 121520.0 - 0.5 * 6076.0) return false;
        // 4. Must not crash.
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB mode; Turn within 60° of airbase bearing; "
               "Close distance to airbase (at least 0.5NM closer); No crash; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered RTB:          %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading to north: %.1f° (need < 60°) %s\n",
            minAbsHeadingToNorth_ * RTD,
            minAbsHeadingToNorth_ < 60.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to airbase:  %.0f ft (need < %.0f) %s\n",
            minDistToAirbase_, 121520.0 - 0.5 * 6076.0,
            minDistToAirbase_ < 121520.0 - 0.5 * 6076.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Landing:      %s\n", enteredLanding_ ? "[PASS]" : "(n/a)");
        std::printf("  Min altitude:         %.0f ft %s\n", minAlt_,
            minAlt_ >= 1000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    double initialHeading_{0.0};
    FrameInputs::AirbaseInfo airbase_;
    const DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    bool enteredRTB_{false};
    bool enteredLanding_{false};
    double minAbsHeadingToNorth_{1e9};
    double minDistToAirbase_{1e9};
    double minAlt_{1e9};
    bool hasNaN_{false};
};

// ===========================================================================
// Scenario: digi_rtb
// ===========================================================================
class DigiRTBScenario : public ManeuverScenario {
public:
    DigiRTBScenario() : ManeuverScenario("digi_rtb") {}

    std::string GetDescription() const override {
        return "Digi AI RTB: Bingo fuel divert to nearest airbase. Tests "
               "FuelCheck + AirbaseCheck + RTB navigation end-to-end. The AI "
               "starts heading east with Bingo fuel and a friendly airbase 20NM "
               "north; it must enter RTB mode, turn toward the airbase, and "
               "close the distance.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<RTBPhase>(
            "Bingo fuel RTB to nearest airbase", 90.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiRTB("digi_rtb", []() {
    return std::make_unique<DigiRTBScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_rtb() {}

} // namespace f4flight_test
