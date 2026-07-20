// f4flight - scenarios/high_level/scenario_digi_aar.cpp
//
// Digi AI air-to-air refueling (AAR) test with both a tanker and receiver.
//
// The tanker flies along predefined waypoints. The receiver (AI aircraft)
// performs an approach, precontact, contact, and disconnect.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
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

enum class RefuelStage : int {
    Approach = 0,
    PrecontactStabilize = 1,
    Contact = 2,
    PostPrecontact = 3,
    DescendHold = 4,
    Depart = 5
};

struct AARBlackboard : public Blackboard {
    SimulatedAircraft* receiver {nullptr};
    SimulatedAircraft* tanker {nullptr};
    DigiEntity* tankerEntity {nullptr};
    RefuelStage* currentStage {nullptr};
    bool inject {true};
};

class AARApproachNode : public BehaviorNode {
public:
    AARApproachNode() : BehaviorNode("AARApproach") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::Approach;
        abb.receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::Approach;
        abb.inject = true;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);

        const double tanker_x = abb.tanker->fm.state().kin.x;
        const double tanker_y = abb.tanker->fm.state().kin.y;
        const double tanker_z = abb.tanker->fm.state().kin.z;
        const double tanker_yaw = abb.tanker->fm.state().kin.psi;

        constexpr double kBoomOffsetBackFt = 50.0;
        constexpr double kBoomOffsetDownFt = 20.0;
        const double boomX = tanker_x - kBoomOffsetBackFt * std::cos(tanker_yaw);
        const double boomY = tanker_y - kBoomOffsetBackFt * std::sin(tanker_yaw);
        const double boomZ = tanker_z + kBoomOffsetDownFt;

        const double dx = boomX - abb.receiver->fm.state().kin.x;
        const double dy = boomY - abb.receiver->fm.state().kin.y;
        const double dz = boomZ - abb.receiver->fm.state().kin.z;
        const double d_boom = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (d_boom < 400.0) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

class AARPrecontactStabilizeNode : public BehaviorNode {
public:
    AARPrecontactStabilizeNode() : BehaviorNode("AARPrecontactStabilize") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::PrecontactStabilize;
        abb.receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::Approach;
        abb.inject = true;
        timer_ = 0.0;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        timer_ += abb.dt;
        if (timer_ >= 10.0) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
private:
    double timer_ {0.0};
};

class AARContactNode : public BehaviorNode {
public:
    AARContactNode() : BehaviorNode("AARContact") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::Contact;
        abb.receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::Contact;
        abb.inject = true;
        timer_ = 0.0;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        timer_ += abb.dt;

        const double tanker_x = abb.tanker->fm.state().kin.x;
        const double tanker_y = abb.tanker->fm.state().kin.y;
        const double tanker_z = abb.tanker->fm.state().kin.z;
        const double tanker_yaw = abb.tanker->fm.state().kin.psi;

        const double shiftForward = 40.0;
        const double shiftUp = 10.0;
        abb.tankerEntity->x = tanker_x + shiftForward * std::cos(tanker_yaw);
        abb.tankerEntity->y = tanker_y + shiftForward * std::sin(tanker_yaw);
        abb.tankerEntity->z = tanker_z - shiftUp;

        if (timer_ >= 30.0) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
private:
    double timer_ {0.0};
};

class AARPostPrecontactNode : public BehaviorNode {
public:
    AARPostPrecontactNode() : BehaviorNode("AARPostPrecontact") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::PostPrecontact;
        abb.receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::Approach;
        abb.inject = true;
        timer_ = 0.0;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        timer_ += abb.dt;
        if (timer_ >= 10.0) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
private:
    double timer_ {0.0};
};

class AARDescendHoldNode : public BehaviorNode {
public:
    AARDescendHoldNode() : BehaviorNode("AARDescendHold") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::DescendHold;
        abb.inject = false;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);

        const double tanker_z = abb.tanker->fm.state().kin.z;
        abb.receiver->sc.setMode(SteeringController::Mode::HeadingAltitude);
        abb.receiver->sc.setAltitude(-tanker_z - 1000.0);
        abb.receiver->sc.setHeading(PI / 2.0);

        const double currentAlt = -abb.receiver->fm.state().kin.z;
        const double targetAlt = -tanker_z - 1000.0;
        if (std::abs(currentAlt - targetAlt) < 150.0) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

class AARDepartNode : public BehaviorNode {
public:
    AARDepartNode() : BehaviorNode("AARDepart") {}
protected:
    void onEnter(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        *(abb.currentStage) = RefuelStage::Depart;
        abb.inject = false;
    }
    NodeStatus onTick(Blackboard& bb) override {
        auto& abb = static_cast<AARBlackboard&>(bb);
        const double tanker_z = abb.tanker->fm.state().kin.z;
        abb.receiver->sc.setMode(SteeringController::Mode::HeadingAltitude);
        abb.receiver->sc.setAltitude(-tanker_z - 1000.0);
        abb.receiver->sc.setHeading(PI);
        return NodeStatus::Running;
    }
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

        currentStage_ = RefuelStage::Approach;

        // 1. Create receiver aircraft
        auto receiver = CreateAircraft("Receiver", ctx.cfg);
        receiver->fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, PI / 2.0, true);

        // Position receiver in 0.3 NM trail (1822 ft behind) and 500 ft below the tanker.
        // This is a highly robust starting point for all categories (fighter/heavy/attack).
        receiver->fm.state().kin.x = 0.0;
        receiver->fm.state().kin.y = -1822.0;
        receiver->fm.state().kin.z = -(alt - 500.0);

        receiver->sc.setMode(SteeringController::Mode::HeadingAltitude);
        receiver->sc.setCornerSpeed(refuelCas); // Match target refueling speed exactly!
        receiver->sc.setMaxGs(ctx.cfg.geometry.maxGs);
        receiver->sc.setMaxBank(30.0);
        receiver->sc.setMaxGamma(10.0);
        receiver->sc.setAltitude(alt);
        receiver->sc.setHeading(PI / 2.0);

        receiver->sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        receiver->sc.brain().stateMutable().refuel.contactTimer = 0.0;
        receiver->sc.brain().stateMutable().refuel.contactDuration = 30.0; // 30s contact hold

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

        // Limit tanker turns to 25 degrees bank
        tanker->sc.setMaxBank(25.0);
        tanker->sc.setCornerSpeed(refuelCas);

        // Initialize the Behavior Tree
        aarTree_ = std::make_shared<SequenceNode>("AARSequence");
        aarTree_->addChild(std::make_shared<AARApproachNode>());
        aarTree_->addChild(std::make_shared<AARPrecontactStabilizeNode>());
        aarTree_->addChild(std::make_shared<AARContactNode>());
        aarTree_->addChild(std::make_shared<AARPostPrecontactNode>());
        aarTree_->addChild(std::make_shared<AARDescendHoldNode>());
        aarTree_->addChild(std::make_shared<AARDepartNode>());

        // Initialize the Blackboard
        aarBlackboard_ = AARBlackboard();
        aarBlackboard_.receiver = receiver.get();
        aarBlackboard_.tanker = tanker.get();
        aarBlackboard_.tankerEntity = &tankerEntity_;
        aarBlackboard_.currentStage = &currentStage_;

        // 3. Define Telemetries
        auto telInjectTanker = CreateTelemetry("inject_tanker", [this, receiver, tanker]() {
            const double tanker_x = tanker->fm.state().kin.x;
            const double tanker_y = tanker->fm.state().kin.y;
            const double tanker_z = tanker->fm.state().kin.z;
            const double tanker_yaw = tanker->fm.state().kin.psi;

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

            // Tick the Behavior Tree!
            aarBlackboard_.dt = 1.0 / 60.0;
            aarTree_->tick(aarBlackboard_);

            if (aarBlackboard_.inject) {
                fi.injectedTanker = &tankerEntity_;
                receiver->sc.brain().setFrameInputs(fi);
            } else {
                receiver->sc.brain().setFrameInputs({});
            }

            return static_cast<double>(currentStage_);
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

        auto telRefuelStage = CreateTelemetry("refuel_stage", [this]() {
            return static_cast<double>(currentStage_);
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
        // 4.1 Enter Refueling mode (Approach)
        auto condApproach = CreateConditional<ConditionalValueReachesRange>(
            telActiveMode, static_cast<double>(DigiMode::Refueling), 0.1, true);

        // 4.2 Reaches pre-contact stabilization stage (refuel_stage == 1.0)
        auto condPrecontact = CreateConditional<ConditionalValueReachesRange>(
            telRefuelStage, 1.0, 0.1, true);

        // 4.3 Reaches contact stage (refuel_stage == 2.0)
        auto condContact = CreateConditional<ConditionalValueReachesRange>(
            telRefuelStage, 2.0, 0.1, true);

        // 4.4 Reaches post-precontact stage (refuel_stage == 3.0)
        auto condPostPrecontact = CreateConditional<ConditionalValueReachesRange>(
            telRefuelStage, 3.0, 0.1, true);

        // 4.5 Reaches descend hold stage (refuel_stage == 4.0)
        auto condDescend = CreateConditional<ConditionalValueReachesRange>(
            telRefuelStage, 4.0, 0.1, true);

        // 4.6 Reaches depart stage (refuel_stage == 5.0) and hold for 5.0 seconds
        auto condDepart = CreateConditional<ConditionalDuration>(5.0, true);

        // Chain execution via OnPassed callbacks
        condApproach->OnPassed = [condPrecontact]() {
            condPrecontact->Start();
        };

        condPrecontact->OnPassed = [condContact]() {
            condContact->Start();
        };

        condContact->OnPassed = [condPostPrecontact]() {
            condPostPrecontact->Start();
        };

        condPostPrecontact->OnPassed = [condDescend]() {
            condDescend->Start();
        };

        condDescend->OnPassed = [condDepart]() {
            condDepart->Start();
        };

        // Start the first conditional in the chain
        condApproach->Start();

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        // Total time set to 250s for safety on slow/heavy aircraft.
        tests.push_back(std::make_unique<ConditionalManeuverTest>(
            *this, "Refuel Sequence", 250.0));
        return tests;
    }

private:
    DigiEntity tankerEntity_;
    RefuelStage currentStage_{RefuelStage::Approach};

    // Behavior tree variables
    std::shared_ptr<SequenceNode> aarTree_;
    AARBlackboard aarBlackboard_;
};

static RegisterScenario g_registerDigiAAR("digi_aar", []() {
    return std::make_unique<DigiAARScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_aar() {}

} // namespace f4flight_test
