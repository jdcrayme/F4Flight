// f4flight - scenarios/scenario_digi_collision.cpp
//
// Maneuver test for digi AI CollisionAvoid mode.
//
// Sets up a head-on collision course: AI and target on opposite headings,
// same altitude, closing fast. The AI should:
//   1. Detect the predicted collision via CollisionCheck (extrapolates both
//      velocity vectors; if they cross within kHRange=200ft, predict a
//      mid-air).
//   2. Enter CollisionAvoid mode.
//   3. Maneuver to avoid (set an evasion trackpoint at 45° off the nose).
//   4. Not crash or NaN.
//
// CollisionCheck triggers when:
//   - timeToImpact <= reactTime = (kGSLimit/maxGs) * kReactFact
//     = (9.0 / maxGs) * 0.55
//   - For an F-16 (maxGs=7.5), reactTime = 0.66s
//   - At 1170 ft/s closure, timeToImpact = (range - 200) / 1170
//   - Collision fires when range <= 200 + 1170*0.66 = ~972 ft
//
// To make the collision fire IMMEDIATELY (not require a long closure), we
// start the target at 500 ft head-on — timeToImpact = (500-200)/1170 = 0.26s
// < 0.66s, so CollisionAvoid fires on the first frame.
//
// Pass criteria:
//   - Enters CollisionAvoid mode at some point
//   - Maneuvers (heading change > 15° proves evasion)
//   - No NaN, no crash

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// Phase: CollisionAvoid (head-on collision course)
// ===========================================================================
class CollisionAvoidPhase : public ManeuverTest {
public:
    CollisionAvoidPhase(const char* name, double duration,
                        double alt, double speed, double targetSpeed,
                        double initialRangeFt)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed),
          targetSpeed_(targetSpeed), initialRangeFt_(initialRangeFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setAltitude(alt_);
        sc.setHeading(0.0);  // east
        sc.setCornerSpeed(speed_);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);
        sc.setMaxGamma(15.0);
        isHeavy_ = isHeavy(fm.config());
        maxGs_ = fm.config().geometry.maxGs;

        // Target ahead (+x), head-on (heading west, toward us), at close range.
        // Both at same altitude. This is a direct collision course.
        target_.x = initialRangeFt_;
        target_.y = 0.0;
        target_.z = -alt_;  // same altitude
        target_.yaw = PI;   // west (toward us)
        target_.pitch = 0.0;
        target_.roll = 0.0;
        target_.speed = targetSpeed_ * KNOTS_TO_FTPSEC;
        target_.vx = -target_.speed;  // moving west (toward us)
        target_.vy = 0.0;
        target_.vz = 0.0;
        target_.isDead = false;

        // Inject the target so CollisionCheck has something to extrapolate.
        sc.setTarget(&target_);
        sc_brain_ = &sc.brain();
        initialHeading_ = 0.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the target toward us.
        target_.x -= target_.speed * dt;
        // If target passes us, reset to keep the threat active.
        if (target_.x < -1000.0) target_.x = initialRangeFt_;

        // Per-frame state
        const double dx = target_.x - as.kin.x;
        const double dy = target_.y - as.kin.y;
        const double range = std::sqrt(dx * dx + dy * dy);
        minRange_ = std::min(minRange_, range);
        maxRange_ = std::max(maxRange_, range);

        minAlt_ = std::min(minAlt_, -as.kin.z);
        maxG_ = std::max(maxG_, as.loads.nzcgs);
        maxHeadingChange_ = std::max(maxHeadingChange_,
            std::fabs(as.kin.sigma - initialHeading_));

        // Track lateral separation from the collision course (x-axis). The
        // target is on the x-axis (y=0); the aircraft starts on the x-axis
        // (y=0). CollisionAvoid should move the aircraft OFF the x-axis
        // (lateral evasion). The old test only checked heading change —
        // which can be satisfied by turning in the wrong direction (toward
        // the collision) or by pitching up without lateral movement.
        maxLateralSep_ = std::max(maxLateralSep_, std::fabs(as.kin.y));

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        const DigiMode mode = sc_brain_->activeMode();
        if (mode == DigiMode::CollisionAvoid) enteredCollisionAvoid_ = true;
        curMode_ = mode;

        // Per-frame sample data
        curRange_ = range;
        curHdgChg_ = std::fabs(as.kin.sigma - initialHeading_) * RTD;
        curG_ = as.loads.nzcgs;
        curInCollision_ = (mode == DigiMode::CollisionAvoid);
        curLatSep_ = std::fabs(as.kin.y);

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (target %.0fft head-on, %.0f kts, same alt)\n",
                    testName_.c_str(), initialRangeFt_, targetSpeed_);
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "range", "hdg(d)", "G", "rstk", "mode");
            }
            const std::size_t bufSize = 16;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(mode));
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s\n",
                phaseTime_, -as.kin.z, range, as.kin.sigma * RTD,
                as.loads.nzcgs, input.rstick, modeBuf);
            nextPrint_ += 1.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered CollisionAvoid mode.
        if (!enteredCollisionAvoid_) return false;
        // 2. Must have maneuvered to avoid — track the max heading change
        //    from the initial heading. >15° proves the AI rolled away from
        //    the collision course. (Heavy aircraft may only manage 10°.)
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) return false;
        // 3. Must have achieved LATERAL separation from the collision course.
        //    The target is on the x-axis (y=0); the aircraft starts on the
        //    x-axis. CollisionAvoid should move the aircraft OFF the x-axis.
        //    The old test only checked heading change — a regression where
        //    the AI turns but doesn't actually move laterally (e.g., pitches
        //    up without rolling) would pass the heading check but fail this.
        //    Require >= 200 ft lateral separation (the evasion trackpoint is
        //    at 45° off the nose, which produces both x and y movement).
        const double latSepThreshold = isHeavy_ ? 100.0 : 200.0;
        if (maxLateralSep_ < latSepThreshold) return false;
        // 4. Must not have lawn-darted.
        if (minAlt_ < 5000.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter CollisionAvoid mode; Heading change > 15deg (10deg heavy); "
               "Lateral separation >= 200ft (100ft heavy); Min alt >= 5000ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredCollisionAvoid_) {
            // Compute the expected react time for diagnostics.
            const double reactTime = (9.0 / std::max(maxGs_, 1.0)) * 0.55;
            return "Never entered CollisionAvoid mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   "; target closed to " + std::to_string(static_cast<int>(minRange_)) +
                   "ft — CollisionCheck did not predict a mid-air (reactTime=" +
                   std::to_string(reactTime) + "s may be too short for this airframe).";
        }
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        if (maxHeadingChange_ < hdgThreshold * DTR) {
            return "Max heading change was " + std::to_string(curHdgChg_) +
                   "deg (needed > " + std::to_string(hdgThreshold) +
                   "deg) — aircraft did not maneuver to avoid the collision.";
        }
        const double latSepThreshold = isHeavy_ ? 100.0 : 200.0;
        if (maxLateralSep_ < latSepThreshold) {
            return "Max lateral separation from collision course was " +
                   std::to_string(static_cast<int>(maxLateralSep_)) +
                   "ft (needed >= " + std::to_string(static_cast<int>(latSepThreshold)) +
                   "ft) — aircraft turned but did not move laterally off the "
                   "collision line (evasion trackpoint not followed).";
        }
        if (minAlt_ < 5000.0) {
            return "Min altitude was " + std::to_string(static_cast<int>(minAlt_)) +
                   "ft (needed >= 5000ft) — collision evasion pulled the aircraft too low.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"range",        curRange_,         "ft"},
            {"hdg_chg",      curHdgChg_,        "deg"},
            {"G",            curG_,             ""},
            {"lat_sep",      curLatSep_,        "ft"},
            {"in_collision", curInCollision_ ? 1.0 : 0.0, ""},
        };
    }

    // The target is auto-extracted by the framework via brain.resolvedTarget().

    void Finish() const override {
        const double hdgThreshold = isHeavy_ ? 10.0 : 15.0;
        const double latSepThreshold = isHeavy_ ? 100.0 : 200.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Entered CollisionAvoid: %s\n",
            enteredCollisionAvoid_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max heading change:    %.1f deg (need > %.0f) %s\n",
            maxHeadingChange_ * RTD, hdgThreshold,
            maxHeadingChange_ > hdgThreshold * DTR ? "[PASS]" : "[FAIL]");
        std::printf("  Max lateral sep:       %.0f ft (need >= %.0f) %s\n",
            maxLateralSep_, latSepThreshold,
            maxLateralSep_ >= latSepThreshold ? "[PASS]" : "[FAIL]");
        std::printf("  Range: %.0f..%.0f ft (info)\n", minRange_, maxRange_);
        std::printf("  Max G:                 %.2f\n", maxG_);
        std::printf("  Min altitude:          %.0f ft (need >= 5000) %s\n",
            minAlt_, minAlt_ >= 5000.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_, speed_, targetSpeed_, initialRangeFt_;
    DigiEntity target_;
    double nextPrint_{0.0};
    double initialHeading_{0.0};
    double minRange_{1e9};
    double maxRange_{0.0};
    double minAlt_{1e9};
    double maxG_{0.0};
    double maxGs_{9.0};
    double maxHeadingChange_{0.0};
    double maxLateralSep_{0.0};
    bool hasNaN_{false};
    bool enteredCollisionAvoid_{false};
    bool isHeavy_{false};
    const DigiBrain* sc_brain_{nullptr};

    // Per-frame sample data
    double curRange_{0.0};
    double curHdgChg_{0.0};
    double curG_{0.0};
    double curLatSep_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    bool curInCollision_{false};
};

// ===========================================================================
// DigiCollisionScenario
// ===========================================================================
class DigiCollisionScenario : public ManeuverScenario {
public:
    DigiCollisionScenario() : ManeuverScenario("digi_collision") {}

    std::string GetDescription() const override {
        return "Digi AI CollisionAvoid: head-on target at close range, same "
               "altitude. Tests CollisionCheck (velocity extrapolation + "
               "reactTime) + the CollisionAvoid evasion trackpoint.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const double speed = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Target at 500 ft head-on — inside CollisionCheck's reactTime window
        // for any aircraft, so CollisionAvoid fires on the first frame.
        tests.push_back(std::make_unique<CollisionAvoidPhase>(
            "CollisionAvoid (500ft head-on)", 12.0, alt, speed, 350.0, 500.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiCollision("digi_collision", []() {
    return std::make_unique<DigiCollisionScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_collision() {}

} // namespace f4flight_test
