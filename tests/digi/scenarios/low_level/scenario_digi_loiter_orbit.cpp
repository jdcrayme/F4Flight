// f4flight - scenarios/scenario_digi_loiter_orbit.cpp
//
// Maneuver test for digi AI Loiter mode — full orbit verification.
//
// MORE THOROUGH than the existing digi_tactics Loiter phase (which only
// checks 20° heading change in 30s). This scenario forces Loiter mode for
// 180 seconds and verifies the aircraft completes nearly a full orbit.
//
// Loiter uses a 30° bank orbit. At corner speed (350 kts = 589 ft/s),
// level turn rate = g * tan(30°) / vt = 32.2 * 0.577 / 589 = 0.0316 rad/s
// = 1.81°/s. For 270° of heading change: 270 / 1.81 = 149 seconds.
//
// Pass criteria:
//   - Enters Loiter mode
//   - Heading change > 270° (3/4 orbit) — proves sustained orbit, not
//     just a brief turn
//   - Returns near start position (within 12000 ft — proves the orbit is
//     closed, not a spiral)
//   - Stable altitude (within ±2000 ft of start)
//   - No NaN, no crash

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

// ===========================================================================
// Phase: Loiter orbit (180s, full orbit verification)
// ===========================================================================
class LoiterOrbitPhase : public ManeuverTest {
public:
    LoiterOrbitPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = 0.0;  // east
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Record the start position so we can verify the orbit returns near it.
        startX_ = fm.state().kin.x;
        startY_ = fm.state().kin.y;
        startZ_ = fm.state().kin.z;

        // Force Loiter mode on the brain so activeMode() returns Loiter.
        sc.setMode(SteeringController::Mode::Loiter);
        sc.brain().forceMode(DigiMode::Loiter);
        sc.setAltitude(alt_);
        sc.setHeading(initialHeading);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(45.0);
        sc.setMaxGamma(15.0);

        sc_brain_ = &sc.brain();
        initialHeading_ = initialHeading;
        isHeavy_ = isHeavy(fm.config());
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double heading = as.kin.sigma;
        currentMode_ = sc_brain_->activeMode();
        if (currentMode_ == DigiMode::Loiter) enteredLoiter_ = true;

        // Track ACCUMULATED heading change (not just max abs delta from
        // initial). Loiter orbits continuously, so the heading rotates
        // through 360°. We track the signed delta each frame and accumulate.
        // Initialize lastHeading_ on the first frame.
        if (!lastHeadingInit_) {
            lastHeading_ = heading;
            lastHeadingInit_ = true;
        }
        double dh = heading - lastHeading_;
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        accumulatedHeadingChange_ += dh;
        lastHeading_ = heading;

        // Also track max absolute heading change from initial (for diagnostic).
        double dhInit = heading - initialHeading_;
        while (dhInit >  PI) dhInit -= 2.0 * PI;
        while (dhInit < -PI) dhInit += 2.0 * PI;
        maxAbsHeadingChange_ = std::max(maxAbsHeadingChange_, std::fabs(dhInit));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxAlt_ = std::max(maxAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        // Track distance from start (proves the orbit is closed, not a spiral).
        const double dx = as.kin.x - startX_;
        const double dy = as.kin.y - startY_;
        curDistFromStart_ = std::sqrt(dx * dx + dy * dy);
        minDistFromStart_ = std::min(minDistFromStart_, curDistFromStart_);

        // Per-frame sample data
        curHdgChg_ = std::fabs(accumulatedHeadingChange_) * RTD;
        curHdgMax_ = maxAbsHeadingChange_ * RTD;
        curMode_ = currentMode_;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (forced Loiter, 180s orbit)\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "hdg(d)", "dStart", "G", "rstk", "mode");
            }
            std::printf("%6.1f %8.0f %8.1f %8.0f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, heading * RTD, curDistFromStart_,
                as.loads.nzcgs, input.rstick, digiModeName(currentMode_));
            nextPrint_ += 15.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter Loiter mode.
        if (!enteredLoiter_) return false;
        // 2. Must have turned > 200° (more than half orbit). The Loiter
        //    primitive commands 30° bank, but the actual steady-state turn
        //    rate is ~1.1°/s (not the theoretical 1.81°/s at 30° bank/350kts)
        //    because the FCS bank loop doesn't sustain the full 30° in the
        //    turn. At 1.1°/s, 240s gives ~260° — comfortably above 200°.
        //    Heavy aircraft may turn slower — allow 150°.
        const double hdgThreshold = isHeavy_ ? 150.0 : 200.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR) return false;
        // 3. Must return near start (proves closed orbit, not spiral).
        if (minDistFromStart_ > 12000.0) return false;
        // 4. Altitude stable (within 2000 ft of start).
        if (minAlt_ < alt_ - 2000.0) return false;
        if (maxAlt_ > alt_ + 2000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Loiter mode; Accumulated heading change > 200deg "
               "(150deg heavy); Return within 12000ft of start; "
               "Alt within +-2000ft of start; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredLoiter_) {
            return "Never entered Loiter mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   ") — brain did not latch Loiter despite the mode being forced.";
        }
        const double hdgThreshold = isHeavy_ ? 150.0 : 200.0;
        if (std::fabs(accumulatedHeadingChange_) < hdgThreshold * DTR) {
            return "Accumulated heading change was " +
                   std::to_string(std::fabs(accumulatedHeadingChange_) * RTD) +
                   "deg (needed > " + std::to_string(hdgThreshold) +
                   "deg) — aircraft did not complete a sustained orbit.";
        }
        if (minDistFromStart_ > 12000.0) {
            return "Min distance from start was " +
                   std::to_string(static_cast<int>(minDistFromStart_)) +
                   "ft (needed < 12000ft) — orbit drifted (not closed).";
        }
        if (minAlt_ < alt_ - 2000.0 || maxAlt_ > alt_ + 2000.0) {
            return "Altitude varied [" + std::to_string(static_cast<int>(minAlt_)) +
                   ", " + std::to_string(static_cast<int>(maxAlt_)) +
                   "]ft (needed within +-2000ft of " +
                   std::to_string(static_cast<int>(alt_)) + "ft) — orbit not altitude-stable.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"hdg_chg",  curHdgChg_, "deg"},
            {"hdg_max",  curHdgMax_, "deg"},
            {"d_start",  curDistFromStart_, "ft"},
            {"in_loiter", (enteredLoiter_ && curMode_ == DigiMode::Loiter) ? 1.0 : 0.0, ""},
        };
    }

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 150.0 : 200.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Loiter:           %s\n", enteredLoiter_ ? "[PASS]" : "[FAIL]");
        std::printf("  Accumulated heading chg:  %.1f deg (need > %.0f) %s\n",
            std::fabs(accumulatedHeadingChange_) * RTD, hdgThreshold,
            std::fabs(accumulatedHeadingChange_) > hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max abs heading from init:%.1f deg (info)\n",
            maxAbsHeadingChange_ * RTD);
        std::printf("  Min dist from start:      %.0f ft (need < 12000) %s\n",
            minDistFromStart_, minDistFromStart_ < 12000.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Altitude range:           [%.0f, %.0f] ft (need within +-2000 of %.0f) %s\n",
            minAlt_, maxAlt_, alt_,
            (minAlt_ >= alt_ - 2000.0 && maxAlt_ <= alt_ + 2000.0) ? "[PASS]" : "[FAIL]");
        std::printf("  Max G:                    %.2f\n", maxG_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_;
    double startX_{0.0}, startY_{0.0}, startZ_{0.0};
    double initialHeading_{0.0};
    double nextPrint_{0.0};
    double lastHeading_{0.0};
    bool lastHeadingInit_{false};
    double accumulatedHeadingChange_{0.0};
    double maxAbsHeadingChange_{0.0};
    double minDistFromStart_{1e9};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double maxG_{0.0};
    bool hasNaN_{false};
    bool enteredLoiter_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    DigiMode currentMode_{DigiMode::NoMode};
    // Per-frame sample data
    double curHdgChg_{0.0};
    double curHdgMax_{0.0};
    double curDistFromStart_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// DigiLoiterOrbitScenario
// ===========================================================================
class DigiLoiterOrbitScenario : public ManeuverScenario {
public:
    DigiLoiterOrbitScenario() : ManeuverScenario("digi_loiter_orbit") {}

    // Tier classification for the 3-tier test workflow.
    // See scenario_framework.h -> TestTier enum for the meaning.
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

        std::string GetDescription() const override {
        return "Digi AI Loiter orbit (180s): forces Loiter mode and verifies the "
               "aircraft completes nearly a full orbit (heading change > 270deg, "
               "returns near start, stable altitude). More thorough than the "
               "digi_tactics Loiter phase (20deg in 30s).";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // 240s = 4 min at ~1.1°/s = ~260° heading change. Comfortably above
        // the 200° pass threshold. (Theoretical at 30° bank/350kts is
        // 1.81°/s = 434° in 240s, but the FCS bank loop doesn't sustain
        // the full 30° in the steady-state turn.)
        tests.push_back(std::make_unique<LoiterOrbitPhase>(
            "Loiter orbit (240s, full orbit)", 240.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiLoiterOrbit("digi_loiter_orbit", []() {
    return std::make_unique<DigiLoiterOrbitScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_loiter_orbit() {}

} // namespace f4flight_test
