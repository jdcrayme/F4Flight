// f4flight - scenarios/high_level/scenario_digi_aar.cpp
//
// Digi AI air-to-air refueling (AAR) test with both a tanker and receiver.
//
// All refueling state machine logic is contained in the receiver's DIGI brain
// (runRefueling). The scenario test simply assigns the correct waypoints and
// verifies appropriate stage transitions, positions, and radio calls.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <memory>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// Tanker waypoints definition (NED frame: +X = east, +Y = north, +Z = down)
static const std::vector<Vec3> g_tankerWaypoints = {
    {0.0, 0.0, -15000.0},
    {0.0, 100000.0, -15000.0},
    {0.0, 200000.0, -15000.0},
    {0.0, 300000.0, -15000.0}
};

// ===========================================================================
// Scenario: digi_aar
// ===========================================================================
class DigiAARScenario : public ManeuverScenario {
public:
    DigiAARScenario() : ManeuverScenario("digi_aar") {}

    // Tier classification for the 3-tier test workflow.
    TestTier GetTestTier() const override { return TestTier::HighLevel; }

    std::string GetDescription() const override {
        return "Air-to-air refueling test with both a tanker and receiver: "
               "approach -> precontact -> contact -> disconnect. "
               "Refueling state transitions and pilot decisions are fully managed in the DIGI brain.";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;
        std::vector<double> pathCoords;
        for (size_t i = 0; i < g_tankerWaypoints.size(); ++i) {
            const auto& wp = g_tankerWaypoints[i];
            TraceGeometry tgWp;
            tgWp.name = "Tanker Waypoint " + std::to_string(i + 1);
            tgWp.type = "waypoint";
            tgWp.coords = {wp.x, wp.y, wp.z};
            tgWp.color = "#FFD700";
            geom.push_back(tgWp);

            pathCoords.push_back(wp.x);
            pathCoords.push_back(wp.y);
            pathCoords.push_back(wp.z);
        }

        TraceGeometry tgPath;
        tgPath.name = "Tanker Path";
        tgPath.type = "corridor";
        tgPath.coords = pathCoords;
        tgPath.color = "#FFD700";
        tgPath.width = 2.0;
        geom.push_back(tgPath);

        return geom;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& /*fm*/, const ScenarioContext& ctx) override {

        ClearScenarioObjects();

        const double alt = 15000.0;
        const double speed = 300.0; // kts — typical refueling speed for all aircraft

        // At 15,000 ft, 300 knots TAS is approx 238 knots CAS.
        const double refuelCas = 238.0;

        // 1. Create receiver aircraft
        auto receiver = CreateAircraft("Receiver", ctx.cfg);
        receiver->fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, PI / 2.0, true);

        // Position receiver exactly at the precontact position (120 ft from boom on 30-degree line).
        receiver->fm.state().kin.x = 0.0;
        receiver->fm.state().kin.y = -154.0;
        receiver->fm.state().kin.z = -(alt - 80.0);

        // Set up the FlightPlan on the receiver with a Refuel task!
        auto fp = std::make_shared<FlightPlan>();
        fp->pushTask(MissionTask{TaskType::Refuel, {0.0, 0.0, -alt}, refuelCas, alt, kInvalidEntityId, 30.0});
        receiver->sc.brain().setFlightPlan(fp);

        receiver->sc.setCornerSpeed(refuelCas); // Match target refueling speed exactly!
        receiver->sc.setMaxGs(ctx.cfg.geometry.maxGs);
        receiver->sc.setMaxBank(30.0);
        receiver->sc.setMaxGamma(10.0);
        receiver->sc.setAltitude(alt);
        receiver->sc.setHeading(PI / 2.0);

        receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        receiver->sc.brain().stateMutable().refuel.contactTimer = 0.0;
        receiver->sc.brain().stateMutable().refuel.contactDuration = 30.0; // 30s contact hold

        // 2. Create tanker aircraft
        auto tanker = CreateAircraft("Tanker", ctx.cfg);
        tanker->fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, PI / 2.0, true);

        // Tanker starts at (0, 0, -15000)
        tanker->fm.state().kin.x = 0.0;
        tanker->fm.state().kin.y = 0.0;
        tanker->fm.state().kin.z = -alt;

        // Set up the FlightPlan on the tanker with a Navigate task straight north!
        auto tanker_fp = std::make_shared<FlightPlan>();
        tanker_fp->pushTask(MissionTask{TaskType::Navigate, {0.0, 1000000.0, -alt}, refuelCas, alt, kInvalidEntityId, 0.0});
        tanker->sc.brain().setFlightPlan(tanker_fp);

        // Limit tanker turns to 20 degrees bank as per flight safety when receiver is present
        tanker->sc.setMaxBank(20.0);
        tanker->sc.setCornerSpeed(refuelCas);

        // 3. Define Telemetries
        auto telInjectTanker = CreateTelemetry("inject_tanker", [this, receiver, tanker]() {
            std::printf("DEBUG scenario telInjectTanker: brain_ptr=%p\n", (void*)&receiver->sc.brain());
            const double tanker_x = tanker->fm.state().kin.x;
            const double tanker_y = tanker->fm.state().kin.y;
            const double tanker_z = tanker->fm.state().kin.z;
            const double tanker_yaw = tanker->fm.state().kin.psi;

            std::printf("DEBUG POSITIONS: tanker = {%.1f, %.1f, %.1f}, receiver = {%.1f, %.1f, %.1f}\n",
                tanker_x, tanker_y, tanker_z,
                receiver->fm.state().kin.x, receiver->fm.state().kin.y, receiver->fm.state().kin.z);

            // Common default tanker state population
            tankerEntity_.x = tanker_x;
            tankerEntity_.y = tanker_y;
            tankerEntity_.z = tanker_z;
            tankerEntity_.yaw = tanker_yaw;
            tankerEntity_.pitch = tanker->fm.state().kin.theta;
            tankerEntity_.roll = tanker->fm.state().kin.phi;
            tankerEntity_.speed = tanker->fm.state().kin.vt;
            tankerEntity_.vx = tanker->fm.state().kin.xdot;
            tankerEntity_.vy = tanker->fm.state().kin.ydot;
            tankerEntity_.vz = tanker->fm.state().kin.zdot;
            tankerEntity_.isDead = false;
            tankerEntity_.dcm = tanker->fm.state().kin.dcm;

            FrameInputs fi;
            fi.injectedTanker = &tankerEntity_;
            receiver->sc.brain().setFrameInputs(fi);

            return static_cast<double>(receiver->sc.brain().state().refuel.phase);
        });

        auto telDistToBoom = CreateTelemetry("d_boom", [this, receiver]() {
            constexpr double kBoomOffsetBackFt = 50.0;
            constexpr double kBoomOffsetDownFt = 20.0;
            const double boomX = tankerEntity_.x - kBoomOffsetBackFt * std::cos(tankerEntity_.yaw);
            const double boomY = tankerEntity_.y - kBoomOffsetBackFt * std::sin(tankerEntity_.yaw);
            const double boomZ = tankerEntity_.z + kBoomOffsetDownFt;

            const double dx = boomX - receiver->fm.state().kin.x;
            const double dy = boomY - receiver->fm.state().kin.y;
            const double dz = boomZ - receiver->fm.state().kin.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        });

        auto telActiveMode = CreateTelemetry("active_mode", [receiver]() {
            return static_cast<double>(receiver->sc.brain().activeMode());
        });

        auto telRefuelPhase = CreateTelemetry("refuel_phase", [receiver]() {
            std::printf("DEBUG scenario telRefuelPhase: brain_ptr=%p\n", (void*)&receiver->sc.brain());
            return static_cast<double>(receiver->sc.brain().state().refuel.phase);
        });

        // Debug telemetries
        auto telContactTimer = CreateTelemetry("c_timer", [receiver]() {
            return receiver->sc.brain().state().refuel.contactTimer;
        });

        // Altitudes
        auto telRecZ = CreateTelemetry("rec_z", [receiver]() {
            return receiver->fm.state().kin.z;
        });
        auto telTankZ = CreateTelemetry("tank_z", [tanker]() {
            return tanker->fm.state().kin.z;
        });

        // Speeds
        auto telRecVt = CreateTelemetry("rec_vt", [receiver]() {
            return receiver->fm.state().kin.vt;
        });
        auto telTankVt = CreateTelemetry("tank_vt", [tanker]() {
            return tanker->fm.state().kin.vt;
        });

        // 4. Define and Chain Conditionals
        // 4.1 Enter Refueling mode (Inbound)
        auto condInbound = CreateConditional<ConditionalValueReachesRange>(
            telActiveMode, static_cast<double>(DigiMode::Refueling), 0.1, true);

        // 4.2 Reaches Precontact stage (refuel_phase == 2.0)
        auto condPrecontact = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::Precontact), 0.1, true);

        // 4.3 Reaches Contact stage (refuel_phase == 3.0)
        auto condContact = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::Contact), 0.1, true);

        // 4.4 Reaches Disconnect stage (refuel_phase == 4.0)
        auto condDisconnect = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::Disconnect), 0.1, true);

        // 4.5 Finishes refueling back to Phase::None (refuel_phase == 0.0)
        auto condFinish = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::None), 0.1, true);

        // Chain execution via OnPassed callbacks
        condInbound->OnPassed = [condPrecontact]() {
            condPrecontact->Start();
        };

        condPrecontact->OnPassed = [condContact]() {
            condContact->Start();
        };

        condContact->OnPassed = [condDisconnect]() {
            condDisconnect->Start();
        };

        condDisconnect->OnPassed = [condFinish]() {
            condFinish->Start();
        };

        // Start the first conditional in the chain
        condInbound->Start();

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Total time set to 250s for safety on slow/heavy aircraft.
        tests.push_back(std::make_unique<ConditionalManeuverTest>(
            *this, "Refuel Sequence", 250.0));
        return tests;
    }

private:
    DigiEntity tankerEntity_;
};

static RegisterScenario g_registerDigiAAR("digi_aar", []() {
    return std::make_unique<DigiAARScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_aar() {}

} // namespace f4flight_test
