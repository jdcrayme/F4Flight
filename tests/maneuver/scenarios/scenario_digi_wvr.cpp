// f4flight - scenarios/scenario_digi_wvr.cpp
//
// Maneuver test for digi AI offensive WVR engagement (Tier 2).
//
// This is an END-TO-END integration test that drives the full FlightModel +
// DigiBrain through a WVR dogfight scenario:
//
//   1. Set up a real aircraft in level flight
//   2. Inject a target (bandit) via DigiEntity
//   3. Run the simulation for several seconds, updating the target position
//   4. Verify the AI enters WVREngage mode, turns toward the target, and
//      doesn't crash or NaN
//
// Scenarios:
//   digi_wvr_chase: Target 3 NM ahead, same heading, slower. The AI should
//     enter WVREngage, chase the target, and close range.
//
//   digi_wvr_headon: Target 3 NM ahead, head-on. The AI should enter
//     WVREngage and turn to track the target.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace manuver_test {

// ===========================================================================
// Phase: WVR Chase
//
// Target 3 NM ahead, same heading, slower (400 kts). The AI should:
//   - Enter WVREngage mode
//   - Turn toward the target (small heading change since it's ahead)
//   - Not crash (stay above ground)
//   - Not NaN
// ===========================================================================
class WVRChasePhase : public ManeuverTest {
public:
    WVRChasePhase(const char* name, double duration,
                  double alt, double speed, double targetSpeed,
                  double initialRange)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRange_(initialRange) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // Target ahead, same heading, slower
        target_.x = 0.0;
        target_.y = initialRange_;
        target_.z = -alt_;
        target_.yaw = 0.0;  // same heading (chase)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = target_.speed;
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target forward at its speed
        target_.y += target_.speed * dt;

        // Track state
        const double range = std::sqrt(
            (target_.x - as.kin.x) * (target_.x - as.kin.x) +
            (target_.y - as.kin.y) * (target_.y - as.kin.y));
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);

        // "Turned toward target" check: target is at +Y (north, bearing π/2
        // from the aircraft at origin). Track the minimum angular distance
        // from the aircraft's heading to north — proves the AI turned toward
        // the target, not away.
        double dh = as.kin.sigma - (PI / 2.0);
        while (dh >  PI) dh -= 2.0 * PI;
        while (dh < -PI) dh += 2.0 * PI;
        minAbsHeadingToNorth_ = std::min(minAbsHeadingToNorth_, std::fabs(dh));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        if (sc_brain_->activeMode() == DigiMode::WVREngage) enteredWVREngage_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.0f NM ahead, %.0f kts)\n",
                    testName_.c_str(), initialRange_ / 6076.0, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "pstk", "rstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.pstick, input.rstick, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered WVREngage mode.
        if (!enteredWVREngage_) return false;
        // 2. Must have turned toward the target. Target is at bearing π/2
        //    (north); aircraft starts heading 0 (east). BFM involves a
        //    ~90° turn to put the nose on the target. Requiring the heading
        //    to get within 35° of north proves the AI turned the right
        //    direction (not away). Range closure is NOT asserted — BFM
        //    bleeds energy, so range may increase during the maneuver.
        if (minAbsHeadingToNorth_ > 35.0 * DTR) return false;
        // 3. Must have pulled G — BFM is a high-G maneuver. At maxBank=60°
        //    the level-turn G is 2.0; BFM pulls harder. Require > 2.0 to
        //    catch a regression where the AI doesn't actually maneuver.
        if (maxG_ < 2.0) return false;
        // 4. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered WVREngage: %s\n", enteredWVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Closest hdg to N:  %.1f deg (need <= 35) %s\n",
            minAbsHeadingToNorth_ * RTD, minAbsHeadingToNorth_ <= 35.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max G (need >= 2.0): %.2f %s\n",
            maxG_, maxG_ >= 2.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Range: %.0f..%.0f ft (info, BFM may not close)\n",
            minRange_, maxRange_);
        std::printf("  Min altitude: %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRange_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAbsHeadingToNorth_{std::numeric_limits<double>::max()};
    double minAlt_{1e9};
    double maxG_{0.0};
    bool hasNaN_{false};
    bool enteredWVREngage_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: WVR Head-On
//
// Target 3 NM ahead, head-on (heading toward us). The AI should:
//   - Enter WVREngage mode
//   - Track the target (heading change as target passes)
//   - Not crash
// ===========================================================================
class WVRHeadOnPhase : public ManeuverTest {
public:
    WVRHeadOnPhase(const char* name, double duration,
                   double alt, double speed, double targetSpeed,
                   double initialRange)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRange_(initialRange) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);

        // Target ahead, head-on (heading south = -y)
        target_.x = 0.0;
        target_.y = initialRange_;
        target_.z = -alt_;
        target_.yaw = PI;  // heading south (toward us)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = 0.0;
        target_.vy = -target_.speed;  // moving south (toward us)
        target_.vz = 0.0;
        target_.isDead = false;

        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us
        target_.y -= target_.speed * dt;
        // If target passes us, reset it to keep the engagement going
        if (target_.y < -2000.0) {
            target_.y = initialRange_;
        }

        // Track state
        const double range = std::sqrt(
            (target_.x - as.kin.x) * (target_.x - as.kin.x) +
            (target_.y - as.kin.y) * (target_.y - as.kin.y));
        minRange_ = std::min(minRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxHeadingChange_ = std::max(maxHeadingChange_, std::fabs(as.kin.sigma));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;
        if (sc_brain_->activeMode() == DigiMode::WVREngage) enteredWVREngage_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.0f NM head-on, %.0f kts)\n",
                    testName_.c_str(), initialRange_ / 6076.0, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, modeBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWVREngage_) return false;
        // Must have tracked the target through a significant heading
        // change. Target passes beam (90° bearing change) at ~3 NM closure;
        // the AI should rotate through at least 45° to track it. The
        // metric was already tracked and printed but never asserted.
        if (maxHeadingChange_ < 45.0 * DTR) return false;
        // Must have come close to the target at some point (proof the
        // AI turned toward it, not away).
        if (minRange_ > 0.5 * initialRange_) return false;
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered WVREngage: %s\n", enteredWVREngage_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min range:        %.0f ft (need <= %.0f) %s\n",
            minRange_, 0.5 * initialRange_,
            minRange_ <= 0.5 * initialRange_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading chg:  %.1f deg (need >= 45) %s\n",
            maxHeadingChange_ * RTD, maxHeadingChange_ >= 45.0 * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:     %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max G: %.2f\n", maxG_);
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRange_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double minRange_{1e9};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxHeadingChange_{0.0};
    bool hasNaN_{false};
    bool enteredWVREngage_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// DigiWVRScenario
// ===========================================================================
class DigiWVRScenario : public ManeuverScenario {
public:
    DigiWVRScenario() : ManeuverScenario("digi_wvr") {}

    std::string GetDescription() const override {
        return "Digi AI WVR engagement: chase and head-on. Tests RollAndPull "
               "offensive BFM end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Chase: target 3 NM ahead, same heading, 250 kts (slower)
        tests.push_back(std::make_unique<WVRChasePhase>(
            "WVR chase (3NM, 250kts)", 20.0, alt, speed, 250.0, 3.0 * 6076.0));
        // Head-on: target 3 NM ahead, head-on, 350 kts
        tests.push_back(std::make_unique<WVRHeadOnPhase>(
            "WVR head-on (3NM, 350kts)", 20.0, alt, speed, 350.0, 3.0 * 6076.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiWVR("digi_wvr", []() {
    return std::make_unique<DigiWVRScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_wvr() {}

} // namespace manuver_test
