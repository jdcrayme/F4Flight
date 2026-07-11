// f4flight - scenarios/scenario_combat.cpp
//
// The "combat" scenario: air-combat maneuvering (ACM) and weapons delivery.
//
// STATUS: scaffold only. The scenario is registered so `maneuver_test --list`
// shows "combat" as a future capability, but running it currently produces
// a placeholder sequence (a high-G level turn at constant altitude) rather
// than real ACM/weapons behavior.
//
// Planned future work:
//   - Target tracking: maintain a steering goal that points the aircraft
//     at a maneuvering target. Needs a TargetState struct + a
//     TargetTrackBehavior that commands pitch/roll to keep the target
//     in the weapon envelope.
//   - Intercept geometry: lag pursuit, lead pursuit, pure pursuit. Needs
//     the target's velocity vector, not just its position.
//   - Weapon envelopes: each weapon type (gun, IR missile, radar missile)
//     has its own envelope (range, aspect, off-boresight). The steering
//     controller needs to know which envelope to fly to.
//   - Gun employment: point the nose at the target's predicted future
//     position with the right lead angle for gun range.
//   - Missile employment: cue the pilot to release within the weapon
//     envelope; for radar missiles, fly to a maneuvering-employment
//     zone and maintain radar track until PITOT BULL.
//   - Defensive maneuvering: break turns, beam maneuvers, chaff/flare
//     programs when under attack.
//
// The scaffold below demonstrates where each of these would slot in.

#include "f4flight/f4flight.h"
#include "maneuver_test.h"

#include <cmath>
#include <string>

using namespace f4flight;

namespace manuver_test{

// ---------------------------------------------------------------------------
// DummyCombat — placeholder 
// ---------------------------------------------------------------------------
class DummyCombat : public ManeuverTest {
public:
    DummyCombat()
        : ManeuverTest("Dummy", 60)
    {
        testName_ = "Dummy (placeholder)";
    }

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc.setVerticalBehavior(std::make_unique<AltitudeHold>(1500, 250));
        sc.setHorizontalBehavior(std::make_unique<HeadingHold>(0));
    }

    virtual bool IsPassed() const { return false; }
    
    bool IsFinished() const override { return true; }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
    }

    void Finish() const {
    }

private:
    double bankAngle_, heading_;
};

// ===========================================================================
// CombatScenario — ACM + weapons delivery
// ===========================================================================
class CombatScenario : public ManeuverScenario {
public:
    CombatScenario() : ManeuverScenario("combat") {}

    std::string GetDescription() const override {
        return "Air combat maneuvering and weapons delivery. (SCAFFOLD: "
               "target tracking, intercept geometry, weapon envelopes, and "
               "release cues are future work. Currently runs placeholder "
               "high-G turns and level passes.)";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double cruiseAlt = 1500;
        const double cruiseSpeed = 380;

        // Reset the flight model to a clean state for this scenario. Each test phase
        fm.init(ctx.cfg, cruiseAlt, cruiseSpeed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<DummyCombat>());
        return tests;
    }
};

static RegisterScenario g_registerCombat("combat", []() {
    return std::make_unique<CombatScenario>();
});

// Force-link symbol. See maneuver_test.h for the rationale.
extern "C" void f4flight_forceLink_scenario_combat() {}

} // namespace f4flight
