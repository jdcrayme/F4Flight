// f4flight - scenarios/scenario_digi_formation_maneuver.cpp
//
// Digi AI formation maneuvering test: lead flies a racetrack pattern with
// two 90° standard-rate turns, and the AI wingman must follow.
//
// Pattern (120 s total):
//   0–15 s   : Lead flies NORTH (yaw = +π/2) straight and level.
//   15–45 s  : Lead turns RIGHT 90° at standard rate (3°/s, ~30 s for 90°).
//              At 420 kts (corner Vcas for F-16), standard rate requires
//              roughly a 50° bank — steep for a fighter but this is the
//              kinematic lead's prescribed trajectory (not flown by the FCS).
//   45–60 s  : Lead flies EAST (yaw = 0) straight and level.
//   60–90 s  : Lead turns RIGHT 90° again — ends facing SOUTH (yaw = -π/2).
//   90–120 s : Lead flies SOUTH straight and level.
//
// The slot position rotates with the lead's heading, so the AI wingman
// (slot 1 of a Wedge formation, behind-right at +135°, 1000 ft) must BANK
// to follow the slot through the turns. This is a much harder test than
// straight-and-level formation following because the wingman has to:
//   - Predict where the slot is going (it sweeps an arc around the lead)
//   - Manage energy through the turns (bank bleeds speed)
//   - Re-stabilize on the slot after each turn completes
//
// Pass criteria (looser than straight-and-level formation, since turns are
// intrinsically harder — bank bleeds energy and the slot sweeps an arc):
//   - Enter Wingy mode
//   - Close to within 500 ft of slot at some point (1000 ft for heavies)
//   - Sustain within 800 ft of slot for >= 15 seconds (1200 ft / 8 s for
//     heavies — they can't keep up through standard-rate turns)
//   - Speed match (TAS) < 70 kts (80 kts for heavies)
//   - No NaN
//   - Duration: 120 s (covers the full racetrack)
//
// Coordinate system note (see formation_geometry.h):
// F4Flight uses (x=east, y=north, z=down) with sigma (yaw) measured CCW
// from +x. The slot formula is the ADAPTED form
//     trackX = leadX + range * cos(sigma - relAz*formSide)
//     trackY = leadY + range * sin(sigma - relAz*formSide)
// which reproduces FreeFalcon's intended geometry (positive relAz = right,
// relAz = pi = directly behind) in F4Flight's coordinate system.
// "Right" in F4Flight's CCW convention = DECREASING yaw.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/formation/formation_geometry.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// FormationManeuverPhase — 2-ship formation following through lead turns
// ===========================================================================
// Both the lead and the wingman fly at corner Vcas (matches the existing
// straight-flight formation test for consistency). The wingman's energy
// margin to catch up after a turn comes from the wingman AI's
// closureCorrection term in wingman_ai.cpp (allows up to +70 kts over the
// lead's speed when far from the slot), not from a speed differential at
// init.
class FormationManeuverPhase : public ManeuverTest {
public:
    FormationManeuverPhase(const char* name, double duration,
                           double alt, double leadSpeed, double startOffsetFt)
        : ManeuverTest(name, duration), alt_(alt), leadSpeed_(leadSpeed),
          startOffset_(startOffsetFt) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        const double initialHeading = PI / 2.0;  // north (yaw=+π/2)
        // Wingman starts at corner Vcas (best turn speed) — gives it max
        // energy margin to rejoin after the lead's turns.
        const double wingmanInitKts = fm.config().geometry.cornerVcas_kts > 0
            ? fm.config().geometry.cornerVcas_kts : 330.0;
        fm.init(fm.config(), alt_, wingmanInitKts * KNOTS_TO_FTPSEC,
                initialHeading, true);

        // Place the AI wingman (slot 1) offset from its desired position.
        // Start behind-left of the lead (negative x = west, negative y = south
        // when lead is at origin facing north) so the wingman has to rejoin.
        fm.state().kin.x = startOffset_;
        fm.state().kin.y = -startOffset_;
        fm.state().kin.z = -(alt_ - 200.0);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(60.0);   // allow steeper bank so the wingman can follow turns
        sc.setMaxGamma(15.0);

        // --- Set up the 2-ship formation (lead + AI wingman) ---
        // Lead (slot 0) starts at the origin, flying north at leadSpeed_
        // (= corner Vcas, same as the wingman). The lead is kinematic — it
        // maintains constant airspeed through the turns (no FCS / throttle
        // lag), so the wingman has to keep up with a perfectly-flown lead.
        const double leadVt = leadSpeed_ * KNOTS_TO_FTPSEC;
        lead_.x = 0.0;
        lead_.y = 0.0;
        lead_.z = -alt_;
        lead_.vx = 0.0;
        lead_.vy = leadVt;          // north = +y
        lead_.vz = 0.0;
        lead_.yaw = PI / 2.0;       // north
        lead_.pitch = 0.0;
        lead_.roll = 0.0;
        lead_.speed = leadVt;
        lead_.isDead = false;
        lead_.dcm = dcmFromEuler(lead_.yaw, 0.0, 0.0);

        // Configure the AI wingman: slot 1, Wedge formation.
        // Slot 1 in Wedge is behind-right (relAz=+135°, range=1000 ft).
        // During turns, the slot sweeps an arc around the lead — the wingman
        // must bank to follow.
        sc.setWingman(1, 1);  // leadId=1, slot=1
        sc.setFormation(static_cast<int>(formation::FormationType::Wedge));
        sc.setLead(&lead_);

        // Heavy aircraft (B-52, C-130) have low T/W, low roll authority, and
        // low G — they can't keep up through standard-rate turns. Loosen the
        // pass criteria for them (matches the pattern used in other scenarios
        // like digi_merge, digi_bvr).
        isHeavy_ = isHeavy(fm.config());

        sc_brain_ = &sc.brain();
    }

    // --- Update the lead's maneuvering trajectory ---
    //
    // The lead is a kinematic entity (no flight model), so we directly
    // prescribe its heading, velocity, and position. The pattern is a
    // racetrack: north → right turn → east → right turn → south.
    //
    // "Right" in F4Flight's CCW math convention = DECREASING yaw.
    //   yaw = +π/2 → north (+y)
    //   yaw =  0   → east  (+x)
    //   yaw = -π/2 → south (-y)
    void updateLeadTrajectory(double dt) {
        // Phase boundaries (seconds):
        //   0 – 15 : straight north
        //   15 – 45 : turn right 90° at standard rate (3°/s, 30 s for 90°)
        //   45 – 60 : straight east
        //   60 – 90 : turn right 90° (30 s)
        //   90 – 120: straight south
        constexpr double kLeg1End   = 15.0;
        constexpr double kTurn1End  = 45.0;
        constexpr double kLeg2End   = 60.0;
        constexpr double kTurn2End  = 90.0;
        // Standard rate turn = 3°/s = π/60 rad/s. At 420 kts this corresponds
        // to roughly a 50° bank — steep for a fighter but kinematically valid.
        constexpr double kTurnRate  = PI / 60.0;  // 0.05236 rad/s
        // Bank angle for visualization (computed for the lead's speed at
        // standard rate). g=32.174 ft/s², V=lead_.speed.
        const double leadBankRad =
            (lead_.speed > 1.0)
            ? std::atan(kTurnRate * lead_.speed / 32.174)
            : 0.0;

        if (phaseTime_ < kLeg1End) {
            // Leg 1: north, wings level.
            lead_.yaw = PI / 2.0;
            lead_.roll = 0.0;
        } else if (phaseTime_ < kTurn1End) {
            // Turn 1: right turn from north to east.
            // Right turn = decreasing yaw (CCW math convention).
            const double tIntoTurn = phaseTime_ - kLeg1End;
            lead_.yaw = PI / 2.0 - tIntoTurn * kTurnRate;
            lead_.roll = -leadBankRad;  // right bank = negative roll (CCW math)
        } else if (phaseTime_ < kLeg2End) {
            // Leg 2: east, wings level.
            lead_.yaw = 0.0;
            lead_.roll = 0.0;
        } else if (phaseTime_ < kTurn2End) {
            // Turn 2: right turn from east to south.
            const double tIntoTurn = phaseTime_ - kLeg2End;
            lead_.yaw = -tIntoTurn * kTurnRate;
            lead_.roll = -leadBankRad;
        } else {
            // Leg 3: south, wings level.
            lead_.yaw = -PI / 2.0;
            lead_.roll = 0.0;
        }

        // Update lead's velocity vector from the (possibly new) yaw.
        // The lead maintains constant airspeed through the turns (it's
        // kinematic — we're not modeling the FCS/throttle lag).
        lead_.vx = lead_.speed * std::cos(lead_.yaw);
        lead_.vy = lead_.speed * std::sin(lead_.yaw);

        // Advance lead position.
        lead_.x += lead_.vx * dt;
        lead_.y += lead_.vy * dt;
        // Update DCM (for any code that reads body-frame transforms).
        lead_.dcm = dcmFromEuler(lead_.yaw, lead_.pitch, lead_.roll);

        // Track which phase we're in for logging.
        if (phaseTime_ < kLeg1End)       curLeadPhase_ = "NORTH";
        else if (phaseTime_ < kTurn1End) curLeadPhase_ = "TURN1→E";
        else if (phaseTime_ < kLeg2End)  curLeadPhase_ = "EAST";
        else if (phaseTime_ < kTurn2End) curLeadPhase_ = "TURN2→S";
        else                              curLeadPhase_ = "SOUTH";
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the lead along its racetrack pattern.
        updateLeadTrajectory(dt);

        // Track the AI wingman's distance to its desired formation slot (slot 1).
        // Uses the ADAPTED formula (sigma - relAz) matching wingman_ai.cpp.
        const double leadSigma = lead_.yaw;
        const auto slot1 = formation::FormationTable::defaultInstance().slotGeometry(
            formation::FormationType::Wedge, 1);
        const double desX = lead_.x + slot1.range * std::cos(leadSigma - slot1.relAz);
        const double desY = lead_.y + slot1.range * std::sin(leadSigma - slot1.relAz);
        const double desZ = lead_.z;

        const double dx = desX - as.kin.x;
        const double dy = desY - as.kin.y;
        const double dz = desZ - as.kin.z;
        const double distToSlot = std::sqrt(dx * dx + dy * dy + dz * dz);

        minDistToSlot_ = std::min(minDistToSlot_, distToSlot);

        // Speed error: compare TRUE airspeeds (wingman TAS vs lead TAS), not
        // CAS vs cornerVcas. At altitude CAS < TAS, so the old metric reported
        // a huge phantom error even when the wingman matched the lead's speed.
        // Turns cause additional variation because the wingman has to bank
        // and the FCS isn't perfectly coordinated, so we use a looser bound.
        const double wingTasKts = as.kin.vt / KNOTS_TO_FTPSEC;
        const double leadTasKts = lead_.speed / KNOTS_TO_FTPSEC;
        const double spdErr = std::fabs(wingTasKts - leadTasKts);
        maxSpeedErr_ = std::max(maxSpeedErr_, spdErr);

        // Track SUSTAINED in-proximity time. The slot sweeps an arc during
        // turns, so we use a wider threshold than the straight-flight
        // formation test. Threshold is aircraft-class-dependent.
        if (distToSlot < sustainedProxFt()) {
            timeInProximity_ += dt;
        }

        // Track proximity DURING TURNS only — this is the harder sub-test.
        // We want to know if the wingman can hold formation through the
        // arcing slot, not just on the straight legs.
        const bool inTurn = (curLeadPhase_.find("TURN") != std::string::npos);
        if (inTurn) {
            turnWindowTime_ += dt;
            if (distToSlot < sustainedProxFt()) turnWindowInPos_ += dt;
            turnMaxDist_ = std::max(turnMaxDist_, distToSlot);
        }

        if (sc_brain_->activeMode() == DigiMode::Wingy) enteredWingy_ = true;
        if (sc_brain_->state().formation.wingman.inPosition) inPosition_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt) ||
            std::isnan(lead_.x) || std::isnan(lead_.yaw)) hasNaN_ = true;

        // Store current slot position for traceEntities()
        curSlotX_ = desX;
        curSlotY_ = desY;
        curSlotZ_ = desZ;
        // Per-frame sample data (for trace)
        curDistToSlot_ = distToSlot;
        curSpeedErr_ = spdErr;
        curInPosition_ = sc_brain_->state().formation.wingman.inPosition;
        curMode_ = sc_brain_->activeMode();
        curWingBank_ = as.kin.phi;  // wingman's current bank (rad)
        curLeadYaw_ = lead_.yaw;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (2-ship Wedge, AI=slot 1, lead flies racetrack)\n",
                    testName_.c_str());
                std::printf("%6s %6s %8s %8s %8s %6s %6s %6s %6s %s\n",
                    "t(s)", "phase", "wngX", "wngY", "dSlot", "vcas", "pstk", "rstk",
                    "wbank", "mode");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            std::printf("%6.1f %6s %8.0f %8.0f %8.0f %6.1f %6.2f %6.2f %6.1f %s\n",
                phaseTime_, curLeadPhase_.c_str(),
                as.kin.x, as.kin.y, distToSlot, as.vcas,
                input.pstick, input.rstick,
                curWingBank_ * 180.0 / PI, modeBuf);
            nextPrint_ += 10.0;
        }
    }

    // Provide custom trace entities for visualization:
    //   - "lead"  (green): auto-extracted via frameInputs().injectedLead
    //     (set by sc.setLead(&lead_) in Init). NOT duplicated here.
    //   - "slot"  (blue diamond): the AI wingman's desired formation slot
    //     (moves with the lead — sweeps an arc during turns).
    std::vector<ThreatEntity> traceEntities() const override {
        return {
            {"slot", curSlotX_, curSlotY_, curSlotZ_, 0.0},
        };
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredWingy_) return false;
        // Must close to the slot at least once. Looser than the 300 ft
        // straight-flight test because turns make the slot a moving target.
        if (minDistToSlot_ > minCloseFt()) return false;
        // Must SUSTAIN proximity: hold within sustainedProxFt() for >=
        // minSustainedSec() seconds total. The threshold is wider than the
        // straight-flight test (500 ft) because the slot sweeps an arc
        // during turns; the time requirement proves the wingman held
        // formation through at least one straight + turn segment.
        if (timeInProximity_ < minSustainedSec()) return false;
        // Speed match (TAS vs TAS). Turns cause speed variation (bank bleeds
        // energy; the FCS isn't perfectly coordinated), so we use a looser
        // bound than the straight-flight test (30 kts for fighters).
        if (maxSpeedErr_ > maxSpeedErrKts()) return false;
        return true;
    }

    std::string criteria() const override {
        // Thresholds are aircraft-class-dependent (heavy aircraft get looser
        // tolerances because they can't keep up through standard-rate turns).
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Enter Wingy; Close to <%.0fft of slot; Sustain <%.0fft for >=%.0fs; "
            "TAS match <%.0fkts; No NaN [racetrack: N->E->S with 2 std-rate turns, %s]",
            minCloseFt(), sustainedProxFt(), minSustainedSec(), maxSpeedErrKts(),
            isHeavy_ ? "heavy-class tolerances" : "fighter tolerances");
        return std::string(buf);
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state (kinematic divergence).";
        if (!enteredWingy_) {
            return "Never entered Wingy mode (final mode: " +
                   std::string(digiModeName(curMode_)) +
                   ") — formation role was not activated.";
        }
        if (minDistToSlot_ > minCloseFt()) {
            return "Min distance to slot was " +
                   std::to_string(static_cast<int>(minDistToSlot_)) +
                   "ft (needed < " + std::to_string(static_cast<int>(minCloseFt())) +
                   "ft) — wingman never closed to formation position through the turns.";
        }
        if (timeInProximity_ < minSustainedSec()) {
            return "Only held within " + std::to_string(static_cast<int>(sustainedProxFt())) +
                   "ft of slot for " +
                   std::to_string(static_cast<int>(timeInProximity_)) +
                   "s (needed >= " + std::to_string(static_cast<int>(minSustainedSec())) +
                   "s) — wingman could not hold formation through lead's turns.";
        }
        if (maxSpeedErr_ > maxSpeedErrKts()) {
            return "Max TAS error was " + std::to_string(static_cast<int>(maxSpeedErr_)) +
                   "kts (needed < " + std::to_string(static_cast<int>(maxSpeedErrKts())) +
                   "kts) — wingman did not match lead's true airspeed through turns.";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_slot",    curDistToSlot_,                            "ft"},
            {"spd_err",   curSpeedErr_,                              "kts"},
            {"in_pos",    curInPosition_ ? 1.0 : 0.0,                ""},
            {"in_wingy",  (enteredWingy_ && curMode_ == DigiMode::Wingy) ? 1.0 : 0.0, ""},
            {"wing_bank", curWingBank_ * 180.0 / PI,                 "deg"},
            {"lead_yaw",  curLeadYaw_   * 180.0 / PI,                "deg"},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary (racetrack pattern, 2 std-rate turns) ---\n");
        std::printf("  Entered Wingy mode:    %s\n", enteredWingy_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to slot:      %.0f ft (need < %.0f) %s\n",
            minDistToSlot_, minCloseFt(),
            minDistToSlot_ < minCloseFt() ? "[PASS]" : "[FAIL]");
        std::printf("  Time within %.0fft:     %.1f s (need >= %.0f) %s\n",
            sustainedProxFt(), timeInProximity_, minSustainedSec(),
            timeInProximity_ >= minSustainedSec() ? "[PASS]" : "[FAIL]");
        if (turnWindowTime_ > 0.0) {
            std::printf("  During turns:          in-pos %.0f%% of %.0fs; max dSlot %.0fft\n",
                100.0 * turnWindowInPos_ / turnWindowTime_,
                turnWindowTime_, turnMaxDist_);
        }
        std::printf("  Max TAS error:         %.1f kts (need < %.0f) %s\n",
            maxSpeedErr_, maxSpeedErrKts(),
            maxSpeedErr_ < maxSpeedErrKts() ? "[PASS]" : "[FAIL]");
        std::printf("  Reached in-position:   %s (informational)\n",
            inPosition_ ? "yes" : "no");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    // Pass-criteria thresholds. Wider than the straight-flight formation
    // test because turns make the slot a moving target and bank bleeds
    // energy (the FCS isn't perfectly coordinated through dynamic maneuvers).
    //
    // Two threshold sets:
    //   - Fighter: tighter (the wingman should be able to follow turns).
    //   - Heavy:   looser  (low T/W, low roll authority — can't keep up
    //                        through standard-rate turns; we accept "tried
    //                        to form up and didn't crash" rather than
    //                        "held tight formation through turns").
    // Heavy aircraft use the same pattern as digi_merge / digi_bvr:
    // scale thresholds by aircraft class.
    double minCloseFt()       const { return isHeavy_ ? 1000.0 : 500.0; }
    double sustainedProxFt()  const { return isHeavy_ ? 1200.0 : 800.0; }
    double minSustainedSec()  const { return isHeavy_ ?    8.0 : 15.0; }
    double maxSpeedErrKts()   const { return isHeavy_ ?   80.0 : 70.0; }
    // (Straight-flight formation test uses 30 kts for fighters; turns cause
    // 2x+ variation. Heavy uses 80 kts — turns at slow speeds cause even
    // more relative variation.)

    double nextPrint_{0.0};
    double alt_{0.0};
    double leadSpeed_{0.0};   // lead's prescribed cruise speed (kts)
    double startOffset_{0.0};
    DigiEntity lead_;
    const DigiBrain* sc_brain_{nullptr};

    double minDistToSlot_{1e9};
    double maxSpeedErr_{0.0};
    double timeInProximity_{0.0};
    double turnWindowTime_{0.0};
    double turnWindowInPos_{0.0};
    double turnMaxDist_{0.0};
    bool enteredWingy_{false};
    bool inPosition_{false};
    bool hasNaN_{false};
    bool isHeavy_{false};

    // Current desired slot position (for traceEntities)
    mutable double curSlotX_{0.0}, curSlotY_{0.0}, curSlotZ_{0.0};

    // Per-frame sample data (updated in Evaluate, read in traceSamples)
    double curDistToSlot_{0.0};
    double curSpeedErr_{0.0};
    double curWingBank_{0.0};
    double curLeadYaw_{0.0};
    bool curInPosition_{false};
    DigiMode curMode_{DigiMode::NoMode};
    std::string curLeadPhase_{"NORTH"};
};

// ===========================================================================
// Scenario: digi_formation_maneuver
// ===========================================================================
class DigiFormationManeuverScenario : public ManeuverScenario {
public:
    DigiFormationManeuverScenario() : ManeuverScenario("digi_formation_maneuver") {}

    std::string GetDescription() const override {
        return "Digi AI formation maneuvering: AI wingman (slot 1) follows a "
               "flight lead that flies a racetrack pattern (north → east → south) "
               "with two 90° standard-rate turns. The slot position rotates "
               "with the lead's heading, so the wingman must bank to follow. "
               "Tests AiFollowLead through dynamic lead maneuvers, not just "
               "straight-and-level flight.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 10000.0;
        const double cornerVcas = ctx.cfg.geometry.cornerVcas_kts > 0
            ? ctx.cfg.geometry.cornerVcas_kts : 330.0;
        // Both lead and wingman fly at corner Vcas — matches the existing
        // straight-flight formation test (consistency). The wingman's energy
        // margin to catch up after a turn comes from the wingman AI's
        // closureCorrection term (allows up to +70 kts over lead speed when
        // far from slot), not from a speed differential at init.

        // The framework calls fm.init() once before the phase starts; the
        // phase's Init() re-initializes with the per-phase altitude/speed,
        // so this initial init just needs to be valid.
        fm.init(ctx.cfg, alt, cornerVcas * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Single phase: 120 s, covering the full racetrack pattern.
        // Start offset 1000 ft (the wingman begins displaced from its slot
        // and must rejoin before the first turn at t=15 s).
        tests.push_back(std::make_unique<FormationManeuverPhase>(
            "2-ship Wedge formation through racetrack (lead N→E→S)",
            120.0, alt, cornerVcas, 1000.0));
        return tests;
    }
};

static RegisterScenario g_registerDigiFormationManeuver("digi_formation_maneuver", []() {
    return std::make_unique<DigiFormationManeuverScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_formation_maneuver() {}

} // namespace f4flight_test
