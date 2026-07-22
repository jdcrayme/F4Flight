// f4flight - scenarios/scenario_low_refuel.cpp
//
// LOW-LEVEL scenario: Air-to-Air Refueling (AAR) test.
// Tests the refueling state machine: Inbound → Precontact → Contact → Disconnect.
//
// Tier: LowLevel. Registered as "low_refuel".

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

class LowRefuelScenario : public ManeuverScenario {
public:
    LowRefuelScenario() : ManeuverScenario("low_refuel") {
        maxTime_ = 500.0;  // Allow enough time for full refueling cycle
    }

    std::string GetDescription() const override {
        return "Digi AI Air-to-Air Refueling: Tests the complete refueling mission "
               "including inbound approach, precontact positioning, contact/fuel transfer, "
               "and disconnect/departure phases.";
    }

    std::string GetTestGroup() const override { return "Refueling"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    void StartScenario(const std::string& defaultAircraftPath) override {
        const double targetAlt = 20000.0;  // Typical tanker altitude
        const double targetSpd = 300.0;    // Typical refueling speed

        // 1. Spawn the receiver aircraft (our AI-controlled F-16)
        auto receiver = CreateAircraft("F16", defaultAircraftPath);
        if (!receiver) return;

        // Configure autopilot limits via DigiBrain config
        digi::DigiConfig cfg;
        cfg.cornerSpeedKts = targetSpd;
        cfg.maxGs = receiver->fm.config().geometry.maxGs;
        cfg.maxBankDeg = 45.0;
        cfg.maxGammaDeg = 15.0;
        cfg.turnLoadFactor = 2.0;
        receiver->sc.brain().configure(cfg);

        // Set initial position: behind and below where tanker will be
        // Tanker will be at (0, 0, -targetAlt), so start receiver 5 NM behind and 500 ft below
        const double initialOffsetBack = 5.0 * 6076.0;  // 5 NM behind
        const double initialOffsetDown = 500.0;          // 500 ft below
        receiver->fm.state().kin.x = 0.0;
        receiver->fm.state().kin.y = -initialOffsetBack;
        receiver->fm.state().kin.z = -targetAlt + initialOffsetDown;
        receiver->fm.state().kin.sigma = 0.0;  // Heading north
        receiver->fm.state().vcas = targetSpd;

        // 2. Spawn the tanker aircraft (KC-135 flying straight and level)
        auto tanker = CreateAircraft("KC135", defaultAircraftPath);
        if (!tanker) return;

        // Tanker flies straight and level at refueling altitude/speed
        tanker->fm.state().kin.x = 0.0;
        tanker->fm.state().kin.y = 0.0;
        tanker->fm.state().kin.z = -targetAlt;
        tanker->fm.state().kin.sigma = 0.0;  // Heading north
        tanker->fm.state().vcas = targetSpd;

        // 3. Inject the tanker entity for refueling
        // The brain will automatically enter Refueling mode when injectedTanker is set
        digi::FrameInputs fi;
        fi.injectedTanker = &tanker->sc.getEntity();

        // Set frame inputs on receiver's brain
        receiver->sc.brain().setFrameInputs(fi);

        // 4. Setup declarative telemetries
        auto t_alt = CreateTelemetry("ReceiverAlt", [receiver]() { 
            return -receiver->fm.state().kin.z; 
        });
        auto t_spd = CreateTelemetry("ReceiverSpd", [receiver]() { 
            return receiver->fm.state().vcas; 
        });
        auto t_tanker_alt = CreateTelemetry("TankerAlt", [tanker]() { 
            return -tanker->fm.state().kin.z; 
        });
        
        // Track refueling phase
        auto t_refuel_phase = CreateTelemetry("RefuelPhase", [receiver]() {
            const auto& state = receiver->sc.brain().state();
            return static_cast<double>(state.refuel.phase);
        });

        // Track refuel completion flag
        auto t_refuel_complete = CreateTelemetry("RefuelComplete", [receiver]() {
            const auto& state = receiver->sc.brain().state();
            return state.refuel.refuelComplete ? 1.0 : 0.0;
        });

        // Track distance to boom position
        auto t_boom_dist = CreateTelemetry("BoomDistance", [receiver, tanker]() {
            const auto& state = receiver->sc.brain().state();
            const double dx = receiver->fm.state().kin.x - state.refuel.boomX;
            const double dy = receiver->fm.state().kin.y - state.refuel.boomY;
            const double dz = receiver->fm.state().kin.z - state.refuel.boomZ;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        });

        // 5. Setup declarative assertions (conditionals)
        // Verify receiver maintains approximately tanker altitude during contact
        CreateConditional<ConditionalValueRemainsInRange>(t_alt, targetAlt, 1000.0, 
            /*isRequired=*/true, "Altitude Hold", 
            "Receiver maintains altitude within ±1000 ft of tanker during refueling");

        // Verify receiver maintains approximately refueling speed
        CreateConditional<ConditionalValueRemainsInRange>(t_spd, targetSpd, 50.0, 
            /*isRequired=*/true, "Speed Hold", 
            "Receiver maintains speed within ±50 kts of target during refueling");

        // Verify refueling phase progression (should reach Contact phase = 3)
        CreateConditional<ConditionalValueReachesRange>(t_refuel_phase, 3.0, 0.1, 
            /*isRequired=*/true, "Contact Phase", 
            "Refueling reaches Contact phase (fuel transfer)");

        // Verify refueling completes (refuelComplete flag set)
        CreateConditional<ConditionalValueReachesRange>(t_refuel_complete, 1.0, 0.1, 
            /*isRequired=*/true, "Refuel Complete", 
            "Refueling completes successfully (disconnect phase finished)");

        // Verify proximity to boom during contact (within 200 ft)
        CreateConditional<ConditionalValueReachesRange>(t_boom_dist, 0.0, 200.0, 
            /*isRequired=*/true, "Boom Proximity", 
            "Receiver achieves close proximity to boom position (<200 ft)");
    }
};

static RegisterScenario g_registerRefuel("low_refuel", []() {
    return std::make_unique<LowRefuelScenario>();
});

} // namespace f4flight_test
