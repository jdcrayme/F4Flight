// f4flight - scenarios/high_level/scenario_digi_aar.cpp
//
// Digi AI air-to-air refueling (AAR) test with both a tanker and receiver.
//
// The tanker flies along predefined waypoints. The receiver (AI aircraft)
// performs an approach, precontact, contact, and disconnect.

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
               "approach -> precontact -> contact (hold 5s) -> disconnect. "
               "The tanker is a real simulated aircraft, and the receiver completes AAR.";
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

        // Position receiver 1050 ft behind the tanker's initial position (at 15000 ft, heading north)
        // This is far enough to start in Approach phase (> 500 ft Contact threshold),
        // but close enough to capture quickly.
        receiver->fm.state().kin.x = 0.0;
        receiver->fm.state().kin.y = -1050.0;
        receiver->fm.state().kin.z = -alt;

        receiver->sc.setMode(SteeringController::Mode::HeadingAltitude);
        receiver->sc.setCornerSpeed(refuelCas); // Match target refueling speed exactly!
        receiver->sc.setMaxGs(ctx.cfg.geometry.maxGs);
        receiver->sc.setMaxBank(30.0);
        receiver->sc.setMaxGamma(10.0);
        receiver->sc.setAltitude(alt);
        receiver->sc.setHeading(PI / 2.0);

        receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        receiver->sc.brain().stateMutable().refuel.contactTimer = 0.0;
        receiver->sc.brain().stateMutable().refuel.contactDuration = 5.0; // short for testing

        // 2. Create tanker aircraft (using ctx.cfg for perfect flight stability!)
        auto tanker = CreateAircraft("Tanker", ctx.cfg);
        tanker->fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, PI / 2.0, true);

        // Tanker starts at (0, 0, -15000)
        tanker->fm.state().kin.x = 0.0;
        tanker->fm.state().kin.y = 0.0;
        tanker->fm.state().kin.z = -alt;

        tanker->sc.setWaypoints(g_tankerWaypoints);
        tanker->sc.setMode(SteeringController::Mode::Waypoint);
        tanker->sc.setAltitude(alt);
        tanker->sc.setHeading(PI / 2.0);

        // Set the tanker's target CAS to 238 knots so it maintains level flight stably for all categories!
        tanker->sc.setCornerSpeed(refuelCas);

        // 3. Define Telemetries
        auto telInjectTanker = CreateTelemetry("inject_tanker", [this, receiver, tanker]() {
            // Update tankerEntity_ state from simulated tanker
            tankerEntity_.x = tanker->fm.state().kin.x;
            tankerEntity_.y = tanker->fm.state().kin.y;
            tankerEntity_.z = tanker->fm.state().kin.z;
            tankerEntity_.yaw = tanker->fm.state().kin.psi;
            tankerEntity_.pitch = tanker->fm.state().kin.theta;
            tankerEntity_.roll = tanker->fm.state().kin.phi;
            tankerEntity_.speed = tanker->fm.state().kin.vt;
            tankerEntity_.vx = tanker->fm.state().kin.xdot;
            tankerEntity_.vy = tanker->fm.state().kin.ydot;
            tankerEntity_.vz = tanker->fm.state().kin.zdot;
            tankerEntity_.isDead = false;
            tankerEntity_.dcm = tanker->fm.state().kin.dcm;

            // Inject tanker into receiver's steering controller
            FrameInputs fi;
            fi.injectedTanker = &tankerEntity_;
            receiver->sc.brain().setFrameInputs(fi);

            return 1.0;
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
            return static_cast<double>(receiver->sc.brain().state().refuel.phase);
        });

        // Debug telemetries
        auto telContactTimer = CreateTelemetry("c_timer", [receiver]() {
            return receiver->sc.brain().state().refuel.contactTimer;
        });
        auto telContactDuration = CreateTelemetry("c_dur", [receiver]() {
            return receiver->sc.brain().state().refuel.contactDuration;
        });

        // Altitudes
        auto telRecZ = CreateTelemetry("rec_z", [receiver]() {
            return receiver->fm.state().kin.z;
        });
        auto telTankZ = CreateTelemetry("tank_z", [tanker]() {
            return tanker->fm.state().kin.z;
        });

        // Speeds (for debugging and visualization)
        auto telRecVt = CreateTelemetry("rec_vt", [receiver]() {
            return receiver->fm.state().kin.vt;
        });
        auto telTankVt = CreateTelemetry("tank_vt", [tanker]() {
            return tanker->fm.state().kin.vt;
        });

        // 4. Define and Chain Conditionals
        // 4.1 Enter Refueling mode (Approach)
        auto condApproach = CreateConditional<ConditionalValueReachesRange>(
            telActiveMode, static_cast<double>(DigiMode::Refueling), 0.1, true);

        // 4.2 Reach precontact within 400 ft
        auto condPrecontact = CreateConditional<ConditionalValueReachesRange>(
            telDistToBoom, 0.0, 400.0, true);

        // 4.3 Reach contact phase
        auto condContactPhase = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::Contact), 0.1, true);

        // 4.4 Hold contact for >= 5s
        auto condContactHold = CreateConditional<ConditionalDuration>(5.0, true);

        // 4.5 Enter disconnect phase
        auto condDisconnectPhase = CreateConditional<ConditionalValueReachesRange>(
            telRefuelPhase, static_cast<double>(DigiRefuelState::Phase::Disconnect), 0.1, true);

        // 4.6 Separate to > 500 ft
        auto condDisconnectSeparation = CreateConditional<ConditionalValueGreaterThan>(
            telDistToBoom, 500.0, true);

        // Parallel Activation on Approach Passed to avoid ordering race conditions
        condApproach->OnPassed = [condPrecontact, condContactPhase, condContactHold]() {
            condPrecontact->Start();
            condContactPhase->Start();
            condContactHold->Start();
        };

        condContactHold->OnPassed = [condDisconnectPhase, condDisconnectSeparation]() {
            condDisconnectPhase->Start();
            condDisconnectSeparation->Start();
        };

        // Start the first conditional in the chain
        condApproach->Start();

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<ConditionalManeuverTest>(
            *this, "Refuel Sequence", 150.0));
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
