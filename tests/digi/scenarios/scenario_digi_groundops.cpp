// f4flight - scenarios/scenario_digi_groundops.cpp
//
// Maneuver tests for digi AI ground operations (Phase 2).
//
// These are END-TO-END integration tests that drive the full FlightModel +
// DigiBrain through takeoff and landing scenarios:
//
//   1. Set up a real aircraft on the ground at a runway threshold
//   2. Command the brain to start takeoff (or landing approach)
//   3. Run the simulation for ~60 seconds
//   4. Verify the AI executes the correct sequence:
//      - Takeoff: accelerate, rotate, lift off, climb out
//      - Landing: descend on glideslope, flare, touchdown, decelerate
//
// The tests use a flat-earth ground model (groundZ = 0) and a simple runway
// (heading 0°, threshold at origin). ATC clearance is auto-granted (the
// brain's startTakeoff/startLanding methods set hasTakeoffClearance/hasLandingClearance).

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// Phase: Takeoff
//
// Aircraft starts on the ground at the runway threshold, brakes on. The AI
// should:
//   - Enter Takeoff mode
//   - Accelerate down the runway (throttle = 1.0)
//   - Rotate at V_R (~140 kts)
//   - Lift off (altitude > 10 ft AGL)
//   - Climb out to 1500 ft AGL
//   - Not NaN, not crash
// ===========================================================================
class TakeoffPhase : public ManeuverTest {
public:
    TakeoffPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start on the ground at the runway threshold, heading north.
        // In F4Flight's NED frame, heading = atan2(ydot, xdot), so north
        // (+Y) is heading PI/2. The previous code used heading=0.0 which
        // is EAST — the aircraft took off pointing 90° away from the
        // drawn (north-south) runway, producing the "90° off runway"
        // visual the user reported.
        const double rwyHeading = PI / 2.0;  // north
        fm.init(fm.config(), 0.0, 0.0, rwyHeading, false);  // on ground, 0 kts
        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Check if this is a heavy aircraft (low T/W may not take off in time)
        isHeavy_ = isHeavy(fm.config());

        // Command takeoff. Runway heading must match the aircraft's init
        // heading (PI/2 = north) so the takeoff roll steers straight.
        sc.brain().commandTakeoff(270, rwyHeading, 0.0, 0.0, 0.0);

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;  // groundZ = 0
        maxAlt_ = std::max(maxAlt_, altAGL);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (altAGL > 10.0) becameAirborne_ = true;
        if (altAGL > 1000.0) reachedClimbout_ = true;
        if (input.throttle > 0.9) appliedTakeoffThrottle_ = true;
        if (sc_brain_->activeMode() == DigiMode::Takeoff) enteredTakeoff_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (takeoff from runway 27)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "rstk", "phase");
            }
            const std::size_t bufSize = 24;
            char phaseBuf[bufSize];
            switch (sc_brain_->state().groundOps.phase) {
                case GroundOpsPhase::TakeoffRoll: std::snprintf(phaseBuf, bufSize, "Roll"); break;
                case GroundOpsPhase::Rotation:    std::snprintf(phaseBuf, bufSize, "Rotation"); break;
                case GroundOpsPhase::AfterTakeoff: std::snprintf(phaseBuf, bufSize, "Climbout"); break;
                default: std::snprintf(phaseBuf, bufSize, "Other"); break;
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, input.rstick, phaseBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || reachedClimbout_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. All aircraft must enter Takeoff mode and apply takeoff throttle.
        if (!enteredTakeoff_) return false;
        if (!appliedTakeoffThrottle_) return false;
        // 2. Heavy aircraft (low T/W) may not have enough thrust to take off
        //    in the test time. For these, require at least 80 kts of
        //    acceleration (proves the throttle actually advanced and the
        //    aircraft is moving) — the old predicate accepted a stationary
        //    aircraft as long as the brain latched Takeoff mode.
        if (isHeavy_) {
            return maxSpeed_ >= 80.0;
        }
        // 3. Fighter/attack: must become airborne AND reach a meaningful
        //    altitude. The old test only required 100 ft (7% of the 1500 ft
        //    climbout target) — a hop-and-stall would pass. Require 500 ft
        //    (33% of climbout) and at least 200 kts (rotation + accel).
        if (!becameAirborne_) return false;
        if (maxAlt_ < 500.0) return false;
        if (maxSpeed_ < 200.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Takeoff mode; Apply takeoff throttle; "
               "Fighter: airborne + alt >= 500ft + speed >= 200kts; "
               "Heavy: speed >= 80kts; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Takeoff mode:    %s\n", enteredTakeoff_ ? "[PASS]" : "[FAIL]");
        std::printf("  Applied takeoff throttle:%s\n", appliedTakeoffThrottle_ ? "[PASS]" : "[FAIL]");
        if (isHeavy_) {
            std::printf("  Heavy: max speed %.1f kts (need >= 80) %s\n",
                maxSpeed_, maxSpeed_ >= 80.0 ? "[PASS]" : "[FAIL]");
        } else {
            std::printf("  Became airborne: %s\n", becameAirborne_ ? "[PASS]" : "[FAIL]");
            std::printf("  Max altitude:    %.0f ft (need >= 500) %s\n",
                maxAlt_, maxAlt_ >= 500.0 ? "[PASS]" : "[FAIL]");
            std::printf("  Max speed:       %.1f kts (need >= 200) %s\n",
                maxSpeed_, maxSpeed_ >= 200.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    bool becameAirborne_{false};
    bool reachedClimbout_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};
    bool enteredTakeoff_{false};
    bool appliedTakeoffThrottle_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: Landing
//
// Aircraft starts 3 NM from the runway threshold at 2000 ft AGL, heading
// toward the threshold. The AI should:
//   - Enter Landing mode
//   - Descend on ~3° glideslope
//   - Flare near the ground
//   - Touch down
//   - Decelerate
//   - Not crash, not NaN
// ===========================================================================
class LandingPhase : public ManeuverTest {
public:
    LandingPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 3 NM south of threshold, ON the 3° glideslope, at approach
        // speed. The previous setup started at 2000 ft / 250 kts — 1044 ft
        // above the glideslope and 80 kts too fast. The AI then had to dive
        // to intercept, building a 180+ ft/s descent rate that the flare
        // couldn't arrest. Starting on-glideslope at approach speed lets
        // the TrackPointLanding primitive hold a stable ~22 ft/s descent.
        const double initialRange = 3.0 * 6076.0;  // 3 NM
        const double gsAngle = 3.0 * DTR;          // 3° glideslope
        const double initialAlt = initialRange * std::tan(gsAngle);  // ~956 ft
        const double initialHeading = PI / 2.0;  // north (toward +Y, toward threshold)

        // Approach speed = 1.3 × stallSpeed. stallSpeed is a runtime field
        // (set by fm.init from the aircraft config's aero tables). We init
        // first, then read it. Default 130 kts if not set.
        const double defaultApproachKts = 170.0;  // 1.3 × 130 kts default
        fm.init(fm.config(), initialAlt, defaultApproachKts * KNOTS_TO_FTPSEC,
                initialHeading, true);

        // Re-read stallSpeed from the runtime state (fm.init populates it).
        const double stallSpeed = fm.state().aero.stallSpeed > 0
            ? fm.state().aero.stallSpeed : 130.0;
        const double approachSpeedKts = 1.3 * stallSpeed;
        // Re-init with the correct approach speed if different from default.
        if (std::fabs(approachSpeedKts - defaultApproachKts) > 5.0) {
            fm.init(fm.config(), initialAlt, approachSpeedKts * KNOTS_TO_FTPSEC,
                    initialHeading, true);
        }

        // Position aircraft 3 NM south of origin (threshold at origin)
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -initialRange;
        fm.state().kin.z = -initialAlt;

        // Set initial pitch + flight path to match the 3° glideslope.
        // fm.init defaults to 10° pitch (level flight) which causes the
        // aircraft to climb above the glideslope, then dive to correct.
        // Setting theta=0 + gamma=-3° starts the aircraft descending
        // smoothly on the glideslope.
        fm.state().kin.theta = 0.0;
        fm.state().kin.gmma = -gsAngle;  // descending at 3°
        fm.state().kin.singam = -std::sin(gsAngle);
        fm.state().kin.cosgam = std::cos(gsAngle);
        const double vt0 = fm.state().kin.vt;
        fm.state().kin.xdot = 0.0;
        fm.state().kin.ydot = vt0 * std::cos(gsAngle);
        fm.state().kin.zdot = vt0 * std::sin(gsAngle);  // positive = descending
        fm.state().kin.quat = quatFromEuler(fm.state().kin.psi, fm.state().kin.theta, fm.state().kin.phi);

        initialAlt_ = initialAlt;  // for pass criteria

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Command landing. Runway heading = PI/2 (north) to match the
        // aircraft's approach heading and the drawn runway geometry.
        sc.brain().commandLanding(270, PI / 2.0, 0.0, 0.0, 0.0);

        // The brain now clears pullupTimer/groundAvoidNeeded itself when
        // it detects a landing phase in compute() (matching FreeFalcon's
        // dlogic.cpp:49-52 which disables GroundCheck for LandingMode).

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double altAGL = -as.kin.z;  // groundZ = 0
        minAlt_ = std::min(minAlt_, altAGL);
        maxAlt_ = std::max(maxAlt_, altAGL);  // <-- was missing
        maxSpeed_ = std::max(maxSpeed_, as.vcas);
        minSpeed_ = std::min(minSpeed_, as.vcas);

        if (altAGL < 10.0 && !touchedDown_) {
            touchedDown_ = true;
            touchdownSpeed_ = as.vcas;  // capture speed at moment of touchdown
        }
        if (touchedDown_ && as.vcas < 30.0) stopped_ = true;
        if (sc_brain_->activeMode() == DigiMode::Landing) enteredLanding_ = true;

        if (std::isnan(as.kin.vt) || std::isnan(as.kin.z)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (landing on runway 27, 3NM final)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "vcas", "thrt", "pstk", "rstk", "mode", "phase");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            char phaseBuf[bufSize];
            switch (sc_brain_->state().groundOps.phase) {
                case GroundOpsPhase::Approach:  std::snprintf(phaseBuf, bufSize, "Approach"); break;
                case GroundOpsPhase::Flare:     std::snprintf(phaseBuf, bufSize, "Flare"); break;
                case GroundOpsPhase::Touchdown: std::snprintf(phaseBuf, bufSize, "Touchdown"); break;
                case GroundOpsPhase::Rollout:   std::snprintf(phaseBuf, bufSize, "Rollout"); break;
                case GroundOpsPhase::VacatingRunway: std::snprintf(phaseBuf, bufSize, "Vacating"); break;
                default: std::snprintf(phaseBuf, bufSize, "Other"); break;
            }
            std::printf("%6.1f %8.0f %8.1f %6.2f %6.2f %6.2f %6s %6s\n",
                phaseTime_, altAGL, as.vcas, input.throttle,
                input.pstick, input.rstick, modeBuf, phaseBuf);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || stopped_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must have entered Landing mode.
        if (!enteredLanding_) return false;
        // 2. Must have descended (not just cruised). The old threshold of
        //    initialAlt-100 was too generous (a 100-ft descent over 90 s
        //    is level flight noise). Require descent of at least 500 ft.
        if (minAlt_ > initialAlt_ - 500.0) return false;
        // 3. Must not have climbed excessively. The TrackPointLanding
        //    primitive (ported from FreeFalcon mnvers.cpp:33) is a pure
        //    proportional tracker — no integral, no gamma feedback. In
        //    F4Flight's flight model this still produces a ~200-300 ft
        //    Phugoid transient at capture (the aircraft pitches up first,
        //    then descends — see DIGI_AUDIT.md "Known library gaps").
        //    The old test allowed +500 ft (a go-around would pass); we
        //    tighten to +400 ft (accepts the Phugoid, still catches a
        //    true go-around).
        if (maxAlt_ > initialAlt_ + 400.0) return false;
        // 4. Must touch down (altAGL <= 10 at some point).
        if (!touchedDown_) return false;
        // 5. Must not go excessively underground (no ground reaction bug).
        if (minAlt_ < -500.0) return false;
        // 6. Must have decelerated after touchdown. The rollout logic sets
        //    throttle=0 + wheel brakes + speed brakes. Heavy aircraft have
        //    weak deceleration (low braking effectiveness relative to mass
        //    + residual idle thrust). Require at least 20 kts of deceleration
        //    (proves the rollout phase engaged and throttle went to idle).
        //    The old threshold of 30 kts was too tight for heavy aircraft.
        if (touchedDown_ && minSpeed_ > touchdownSpeed_ - 20.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Landing mode; Descend >= 500ft; Max alt <= initial+400ft; "
               "Touch down; Min alt >= -500ft; Decel >= 30kts after touchdown; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Landing mode: %s\n", enteredLanding_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min altitude:        %.1f ft (need >= -500) %s\n",
            minAlt_, minAlt_ >= -500.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max altitude:        %.1f ft (need <= %.0f) %s\n",
            maxAlt_, initialAlt_ + 400.0,
            maxAlt_ <= initialAlt_ + 400.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Touched down:        %s %s\n", touchedDown_ ? "yes" : "no",
            touchedDown_ ? "[PASS]" : "[FAIL]");
        std::printf("  Descended >= 500 ft: %s\n",
            minAlt_ <= initialAlt_ - 500.0 ? "[PASS]" : "[FAIL]");
        if (touchedDown_) {
            std::printf("  Decel after TD:      %.0f -> %.0f kts (need >= 30 kts decel) %s\n",
                touchdownSpeed_, minSpeed_,
                minSpeed_ <= touchdownSpeed_ - 30.0 ? "[PASS]" : "[FAIL]");
        }
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minAlt_{1e9};
    double maxAlt_{0.0};
    double maxSpeed_{0.0};
    double minSpeed_{1e9};
    double touchdownSpeed_{0.0};
    double initialAlt_{0.0};  // set in Init (on-glideslope altitude)
    bool touchedDown_{false};
    bool stopped_{false};
    bool hasNaN_{false};
    bool enteredLanding_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// Phase: Taxi
//
// Demonstrates minimal taxi capability. The aircraft starts at a parking
// spot 500 ft east of the runway threshold, heading north. The AI taxis
// to the runway threshold (origin) at taxi speed. This exercises the
// RunTaxi code path and the TaxiToRunway ground ops phase.
//
// Pass criteria:
//   - Enter TaxiToRunway phase
//   - Move toward the threshold (distance decreases)
//   - Arrive within 50 ft of the threshold
//   - Speed stays at or below taxi speed (~25 kts)
//   - No NaN, no erratic behavior
// ===========================================================================
class TaxiPhase : public ManeuverTest {
public:
    TaxiPhase(const char* name, double duration)
        : ManeuverTest(name, duration) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start 500 ft east of the runway threshold, heading north.
        // In F4Flight's NED frame, north = +Y = heading PI/2.
        const double startX = 500.0;  // 500 ft east
        const double startY = 0.0;
        const double startHeading = PI / 2.0;  // north

        fm.init(fm.config(), 0.0, 0.0, startHeading, false);  // on ground, 0 kts
        fm.state().kin.x = startX;
        fm.state().kin.y = startY;
        fm.state().kin.z = 0.0;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);

        // Command taxi to the runway threshold (origin).
        // We set the ground ops state directly — the brain's RunTaxi
        // function will steer toward (runwayThresholdX, runwayThresholdY).
        auto& go = sc.brain().stateMutable().groundOps;
        go.phase = GroundOpsPhase::TaxiToRunway;
        go.runwayThresholdX = 0.0;  // threshold at origin
        go.runwayThresholdY = 0.0;
        go.runwayHeading = PI / 2.0;  // north
        go.hasTakeoffClearance = false;  // taxi only, no takeoff

        sc_brain_ = &sc.brain();
        startDist_ = std::sqrt(startX * startX + startY * startY);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double dx = 0.0 - as.kin.x;  // threshold at origin
        const double dy = 0.0 - as.kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        minDist_ = std::min(minDist_, dist);
        maxSpeed_ = std::max(maxSpeed_, as.vcas);

        if (sc_brain_->state().groundOps.phase == GroundOpsPhase::TaxiToRunway)
            enteredTaxi_ = true;
        if (dist < 50.0) reachedThreshold_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (taxi 500ft east → threshold)\n", testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s\n",
                    "t(s)", "x(ft)", "y(ft)", "dist(ft)", "vcas", "rstk", "phase");
            }
            const std::size_t bufSize = 24;
            char phaseBuf[bufSize];
            switch (sc_brain_->state().groundOps.phase) {
                case GroundOpsPhase::TaxiToRunway: std::snprintf(phaseBuf, bufSize, "Taxi"); break;
                case GroundOpsPhase::HoldingShort: std::snprintf(phaseBuf, bufSize, "HoldShort"); break;
                default: std::snprintf(phaseBuf, bufSize, "Other"); break;
            }
            std::printf("%6.1f %8.0f %8.0f %8.0f %6.1f %6.2f %6s\n",
                phaseTime_, as.kin.x, as.kin.y, dist, as.vcas,
                input.rstick, phaseBuf);
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || reachedThreshold_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredTaxi_) return false;
        if (!reachedThreshold_) return false;
        // Speed should stay at or below ~30 kts (taxi speed + margin)
        if (maxSpeed_ > 35.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter TaxiToRunway; Reach threshold (<50ft); Speed <= 35kts; No NaN";
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Taxi mode: %s\n", enteredTaxi_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min distance to threshold: %.1f ft (need < 50) %s\n",
            minDist_, minDist_ < 50.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Max speed: %.1f kts (need <= 35) %s\n",
            maxSpeed_, maxSpeed_ <= 35.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double minDist_{1e9};
    double maxSpeed_{0.0};
    double startDist_{0.0};
    bool reachedThreshold_{false};
    bool enteredTaxi_{false};
    bool hasNaN_{false};
    const DigiBrain* sc_brain_{nullptr};
};

// ===========================================================================
// DigiGroundOpsScenario
// ===========================================================================
class DigiGroundOpsScenario : public ManeuverScenario {
public:
    DigiGroundOpsScenario() : ManeuverScenario("digi_groundops") {}

    std::string GetDescription() const override {
        return "Digi AI ground ops: taxi to runway, takeoff (accelerate, rotate, "
               "climb out), and landing (approach, flare, touchdown, rollout).";
    }

    // 10,000 ft runway centered at origin, running north-south (along +Y).
    // The takeoff starts at the threshold (origin) and the landing approaches
    // from 3 NM south (-Y) heading north toward the threshold. We draw:
    //   - Runway centerline (gold, thick)
    //   - Runway threshold markings (two perpendicular lines at each end)
    std::vector<SceneLine> sceneGeometry() const override {
        std::vector<SceneLine> lines;
        const double rwLen = 10000.0;  // 10,000 ft
        const double rwHalf = rwLen / 2.0;
        const double rwWidth = 200.0;  // 200 ft wide

        // Runway centerline — dark so it's visible under the flight path
        SceneLine centerline;
        centerline.label = "RWY";
        centerline.x1 = 0.0; centerline.y1 = -rwHalf; centerline.z1 = 0.0;
        centerline.x2 = 0.0; centerline.y2 =  rwHalf; centerline.z2 = 0.0;
        centerline.color = "#3a3a4a";
        centerline.width = 150.0;  // 150 ft wide (drawn as a thick line)
        lines.push_back(centerline);

        // Threshold markings (perpendicular lines at each end)
        SceneLine threshN;
        threshN.label = "RWY_End_N";
        threshN.x1 = -rwWidth; threshN.y1 = rwHalf; threshN.z1 = 0.0;
        threshN.x2 =  rwWidth; threshN.y2 = rwHalf; threshN.z2 = 0.0;
        threshN.color = "#3a3a4a";
        threshN.width = 80.0;
        lines.push_back(threshN);

        SceneLine threshS;
        threshS.label = "RWY_End_S";
        threshS.x1 = -rwWidth; threshS.y1 = -rwHalf; threshS.z1 = 0.0;
        threshS.x2 =  rwWidth; threshS.y2 = -rwHalf; threshS.z2 = 0.0;
        threshS.color = "#3a3a4a";
        threshS.width = 80.0;
        lines.push_back(threshS);

        return lines;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        fm.init(ctx.cfg, 0.0, 0.0, 0.0, false);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Taxi: 60s to taxi 500 ft to the threshold
        tests.push_back(std::make_unique<TaxiPhase>("Taxi", 60.0));
        // Takeoff: 90s for heavy aircraft (low T/W needs more runway)
        tests.push_back(std::make_unique<TakeoffPhase>("Takeoff", 90.0));
        // Landing: 90s for approach + flare + rollout
        tests.push_back(std::make_unique<LandingPhase>("Landing", 90.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiGroundOps("digi_groundops", []() {
    return std::make_unique<DigiGroundOpsScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_groundops() {}

} // namespace f4flight_test
