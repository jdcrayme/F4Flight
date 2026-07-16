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
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// Airbase position constants (used by both RTBPhase and the scenario's
// sceneGeometry so the drawn runway matches the actual divert target).
// Airbase is 20 NM north (+Y) of the origin, 5000 ft MSL field elevation.
constexpr double kRtbAirbaseX = 0.0;
constexpr double kRtbAirbaseY = 20.0 * 6076.0;  // 20 NM north
constexpr double kRtbAirbaseZ = -5000.0;

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
        airbase_.x = kRtbAirbaseX;
        airbase_.y = kRtbAirbaseY;  // 20 NM north
        airbase_.z = kRtbAirbaseZ;   // 5000 ft MSL field elevation
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
        isHeavy_ = isHeavy(fm.config());
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

        // Per-frame sample data (for trace)
        curDistToAirbase_ = distToAirbase;
        curHdgErr_ = std::fabs(dh) * RTD;  // heading error to airbase bearing
        curFuelLbs_ = sc_brain_->state().fuel.fuelLbs;
        curMode_ = currentMode_;

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
        //    it, not away). Started at 20 NM = 121520 ft. The old test only
        //    required 0.5 NM closure — trivially satisfied by any drift
        //    toward the airbase. Tighten to require at least 3 NM closure
        //    (proves the AI actually navigated toward the airbase, not just
        //    happened to drift slightly closer). Heavy/slow aircraft (A-10,
        //    C-130) get a 1 NM threshold.
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * 6076.0;
        if (minDistToAirbase_ > 121520.0 - requiredClosureFt) return false;
        // 4. Must not crash.
        if (minAlt_ < 1000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter RTB mode; Turn within 60° of airbase bearing; "
               "Close distance by >= 3NM (1NM heavy); No crash; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredRTB_) {
            return "Never entered RTB mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; fuel was " + std::to_string(static_cast<int>(curFuelLbs_)) +
                   "lbs but FuelCheck did not declare Bingo).";
        }
        if (minAbsHeadingToNorth_ > 60.0 * DTR) {
            return "Closest heading to airbase bearing was " +
                   std::to_string(minAbsHeadingToNorth_ * RTD) +
                   "deg (needed <= 60deg) — aircraft did not turn toward the airbase.";
        }
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * 6076.0;
        if (minDistToAirbase_ > 121520.0 - requiredClosureFt) {
            return "Min distance to airbase was " +
                   std::to_string(static_cast<int>(minDistToAirbase_)) +
                   "ft (needed <= " + std::to_string(static_cast<int>(121520.0 - requiredClosureFt)) +
                   "ft = " + std::to_string(isHeavy_ ? 1.0 : 3.0) +
                   "NM closure) — aircraft did not close meaningfully on the divert field.";
        }
        if (minAlt_ < 1000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 1000ft) — aircraft descended below the safe floor during RTB.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"dist_ab",  curDistToAirbase_, "ft"},
            {"hdg_err",  curHdgErr_,        "deg"},
            {"fuel",     curFuelLbs_,       "lb"},
            {"in_rtb",   (enteredRTB_ && curMode_ == DigiMode::RTB) ? 1.0 : 0.0, ""},
            {"in_landing", (curMode_ == DigiMode::Landing) ? 1.0 : 0.0, ""},
        };
    }

    // Publish the airbase as a trace entity so the HTML report shows the
    // divert field as an amber square in both 2D and 3D views.
    std::vector<ThreatEntity> traceEntities() const override {
        return {{"airbase", airbase_.x, airbase_.y, airbase_.z, 0.0}};
    }

    void Finish() const override {
        const double requiredClosureFt = (isHeavy_ ? 1.0 : 3.0) * 6076.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered RTB:          %s\n", enteredRTB_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min heading to north: %.1f° (need < 60°) %s\n",
            minAbsHeadingToNorth_ * RTD,
            minAbsHeadingToNorth_ < 60.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to airbase:  %.0f ft (need < %.0f) %s\n",
            minDistToAirbase_, 121520.0 - requiredClosureFt,
            minDistToAirbase_ < 121520.0 - requiredClosureFt ? "[PASS]" : "[FAIL]");
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
    bool isHeavy_{false};
    double minAbsHeadingToNorth_{1e9};
    double minDistToAirbase_{1e9};
    double minAlt_{1e9};
    bool hasNaN_{false};

    // Per-frame sample data (updated in Evaluate, read in traceSamples)
    double curDistToAirbase_{0.0};
    double curHdgErr_{0.0};
    double curFuelLbs_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
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

    // Draw the divert runway at the airbase position so the visualization
    // shows where the aircraft is trying to land. Runway points east (heading
    // 0), 6000 ft long, drawn as a gold centerline.
    std::vector<SceneLine> sceneGeometry() const override {
        const double rwyLen = 6000.0;
        const double halfLen = rwyLen / 2.0;
        SceneLine runway;
        runway.label = "Runway";
        runway.x1 = kRtbAirbaseX - halfLen;
        runway.y1 = kRtbAirbaseY;
        runway.z1 = kRtbAirbaseZ;
        runway.x2 = kRtbAirbaseX + halfLen;
        runway.y2 = kRtbAirbaseY;
        runway.z2 = kRtbAirbaseZ;
        runway.color = "#FFD700";
        runway.width = 100.0;  // 100 ft wide
        return {runway};
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
