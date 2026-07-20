// f4flight unit tests - digi AI State Tree Refactoring Bootstrap
//
// Tests for:
//   - NodeStatus, BehaviorNode base, onEnter, onTick, onExit (node.h)
//   - SelectorNode fallback logic (node.h)
//   - SequenceNode execution logic (node.h)
//   - Blackboard data sharing (blackboard.h)
//   - FlightPlan queue and emergency task insertion (flight_plan.h)

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"

#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

class TestActionNode : public BehaviorNode {
public:
    TestActionNode(std::string name, NodeStatus tickResult)
        : BehaviorNode(std::move(name)), tickResult_(tickResult) {}

    int enterCount = 0;
    int tickCount = 0;
    int exitCount = 0;
    NodeStatus lastExitStatus = NodeStatus::Failure;
    NodeStatus tickResult_;

protected:
    void onEnter(Blackboard&) override {
        enterCount++;
    }

    NodeStatus onTick(Blackboard&) override {
        tickCount++;
        return tickResult_;
    }

    void onExit(Blackboard&, NodeStatus status) override {
        exitCount++;
        lastExitStatus = status;
    }
};

TEST(StateTreeTest, NodeLifecycle) {
    Blackboard bb;
    auto node = std::make_shared<TestActionNode>("test", NodeStatus::Running);

    // First frame: should enter and tick
    EXPECT_FALSE(node->hasEntered());
    NodeStatus status1 = node->tick(bb);
    EXPECT_EQ(status1, NodeStatus::Running);
    EXPECT_TRUE(node->hasEntered());
    EXPECT_EQ(node->enterCount, 1);
    EXPECT_EQ(node->tickCount, 1);
    EXPECT_EQ(node->exitCount, 0);

    // Second frame: still running, should tick again but not re-enter
    NodeStatus status2 = node->tick(bb);
    EXPECT_EQ(status2, NodeStatus::Running);
    EXPECT_TRUE(node->hasEntered());
    EXPECT_EQ(node->enterCount, 1);
    EXPECT_EQ(node->tickCount, 2);
    EXPECT_EQ(node->exitCount, 0);

    // Third frame: return Success, should exit and reset entered status
    node->tickResult_ = NodeStatus::Success;
    NodeStatus status3 = node->tick(bb);
    EXPECT_EQ(status3, NodeStatus::Success);
    EXPECT_FALSE(node->hasEntered());
    EXPECT_EQ(node->enterCount, 1);
    EXPECT_EQ(node->tickCount, 3);
    EXPECT_EQ(node->exitCount, 1);
    EXPECT_EQ(node->lastExitStatus, NodeStatus::Success);
}

TEST(StateTreeTest, SelectorNodeEvaluatesLeftToRight) {
    Blackboard bb;
    auto selector = std::make_shared<SelectorNode>("Selector");

    auto child1 = std::make_shared<TestActionNode>("Child1", NodeStatus::Failure);
    auto child2 = std::make_shared<TestActionNode>("Child2", NodeStatus::Success);
    auto child3 = std::make_shared<TestActionNode>("Child3", NodeStatus::Running);

    selector->addChild(child1);
    selector->addChild(child2);
    selector->addChild(child3);

    // First tick of selector: child1 ticks (fails), child2 ticks (succeeds), child3 never ticks
    NodeStatus status = selector->tick(bb);
    EXPECT_EQ(status, NodeStatus::Success);

    EXPECT_EQ(child1->tickCount, 1);
    EXPECT_EQ(child2->tickCount, 1);
    EXPECT_EQ(child3->tickCount, 0);
}

TEST(StateTreeTest, SelectorNodeRunning) {
    Blackboard bb;
    auto selector = std::make_shared<SelectorNode>("Selector");

    auto child1 = std::make_shared<TestActionNode>("Child1", NodeStatus::Failure);
    auto child2 = std::make_shared<TestActionNode>("Child2", NodeStatus::Running);

    selector->addChild(child1);
    selector->addChild(child2);

    NodeStatus status = selector->tick(bb);
    EXPECT_EQ(status, NodeStatus::Running);
    EXPECT_EQ(child1->tickCount, 1);
    EXPECT_EQ(child2->tickCount, 1);
}

TEST(StateTreeTest, SequenceNodeEvaluatesLeftToRight) {
    Blackboard bb;
    auto sequence = std::make_shared<SequenceNode>("Sequence");

    auto child1 = std::make_shared<TestActionNode>("Child1", NodeStatus::Success);
    auto child2 = std::make_shared<TestActionNode>("Child2", NodeStatus::Failure);
    auto child3 = std::make_shared<TestActionNode>("Child3", NodeStatus::Running);

    sequence->addChild(child1);
    sequence->addChild(child2);
    sequence->addChild(child3);

    // Sequence execution: child1 ticks (succeeds), child2 ticks (fails), child3 never ticks
    NodeStatus status = sequence->tick(bb);
    EXPECT_EQ(status, NodeStatus::Failure);

    EXPECT_EQ(child1->tickCount, 1);
    EXPECT_EQ(child2->tickCount, 1);
    EXPECT_EQ(child3->tickCount, 0);
}

TEST(StateTreeTest, SequenceNodeRunning) {
    Blackboard bb;
    auto sequence = std::make_shared<SequenceNode>("Sequence");

    auto child1 = std::make_shared<TestActionNode>("Child1", NodeStatus::Success);
    auto child2 = std::make_shared<TestActionNode>("Child2", NodeStatus::Running);

    sequence->addChild(child1);
    sequence->addChild(child2);

    NodeStatus status = sequence->tick(bb);
    EXPECT_EQ(status, NodeStatus::Running);
    EXPECT_EQ(child1->tickCount, 1);
    EXPECT_EQ(child2->tickCount, 1);
}

TEST(StateTreeTest, BlackboardAndFlightPlanIntegration) {
    Blackboard bb;
    bb.flightPlan = std::make_shared<FlightPlan>();

    MissionTask task1{TaskType::Takeoff, {0, 0, 0}, 300, 1000, 1, 60};
    MissionTask task2{TaskType::Navigate, {1000, 0, -5000}, 350, 5000, 2, 120};

    bb.flightPlan->pushTask(task1);
    bb.flightPlan->pushTask(task2);

    EXPECT_FALSE(bb.flightPlan->isComplete());
    EXPECT_EQ(bb.flightPlan->currentTask().type, TaskType::Takeoff);
    EXPECT_NEAR(bb.flightPlan->currentTask().speedKts, 300, 1e-9);

    bb.flightPlan->advanceTask();
    EXPECT_FALSE(bb.flightPlan->isComplete());
    EXPECT_EQ(bb.flightPlan->currentTask().type, TaskType::Navigate);
    EXPECT_NEAR(bb.flightPlan->currentTask().speedKts, 350, 1e-9);

    bb.flightPlan->advanceTask();
    EXPECT_TRUE(bb.flightPlan->isComplete());
}

TEST(StateTreeTest, FlightPlanEmergencyPreemption) {
    FlightPlan fp;
    MissionTask t1{TaskType::Navigate, {10, 10, 0}, 300, 1000, 0, 10};
    MissionTask t2{TaskType::Landing, {20, 20, 0}, 150, 1000, 0, 10};

    fp.pushTask(t1);
    fp.pushTask(t2);

    // Current task is Navigate
    EXPECT_EQ(fp.currentTask().type, TaskType::Navigate);

    // Insert an emergency task
    MissionTask emergency{TaskType::RTB, {5, 5, 0}, 400, 5000, 0, 0};
    fp.insertEmergencyTask(emergency);

    // Now current task should be the emergency task RTB!
    EXPECT_EQ(fp.currentTask().type, TaskType::RTB);
    EXPECT_NEAR(fp.currentTask().speedKts, 400, 1e-9);

    // Advancing goes to Navigate, then Landing
    fp.advanceTask();
    EXPECT_EQ(fp.currentTask().type, TaskType::Navigate);

    fp.advanceTask();
    EXPECT_EQ(fp.currentTask().type, TaskType::Landing);

    fp.advanceTask();
    EXPECT_TRUE(fp.isComplete());
}

#include "f4flight/digi/behavior_tree/flight_plan_nodes.h"

TEST(StateTreeTest, WaypointCaptureCheckAndNavigateTask) {
    Blackboard bb;
    bb.flightPlan = std::make_shared<FlightPlan>();

    AircraftState as;
    as.kin.costhe = 1.0;
    as.kin.cosphi = 1.0;
    as.kin.gmma = 0.0;
    as.kin.sigma = 0.0;
    as.kin.singam = 0.0;
    as.kin.x = 0.0;
    as.kin.y = 0.0;
    as.kin.z = -10000.0;
    as.kin.zdot = 0.0;
    as.vcas = 350.0;
    as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    bb.as = &as;

    FlightControlSystem fcs;
    FcsState fcsState;
    DigiState state;
    state.config.maxGs = 9.0;
    state.config.maxRoll = 45.0;
    state.config.maxGammaDeg = 15.0;
    bb.fcs = &fcs;
    bb.fcsState = &fcsState;
    bb.state = &state;
    bb.dt = 1.0 / 60.0;

    MissionTask task1{TaskType::Navigate, {0.0, 10000.0, -10000.0}, 300, 10000, kInvalidEntityId, 0.0};
    bb.flightPlan->pushTask(task1);

    auto captureCheck = std::make_shared<WaypointCaptureCheckNode>(5000.0);
    auto navigateTask = std::make_shared<NavigateTaskNode>();

    // Initially at (0, 0), task at (0, 10000). Distance = 10000 > 5000 (captureRadius).
    // WaypointCaptureCheckNode should tick successfully and NOT advance the task.
    NodeStatus checkStatus = captureCheck->tick(bb);
    EXPECT_EQ(checkStatus, NodeStatus::Success);
    EXPECT_FALSE(bb.flightPlan->isComplete());
    EXPECT_EQ(bb.flightPlan->currentTask().type, TaskType::Navigate);

    // NavigateTaskNode should run successfully and command heading and altitude.
    NodeStatus navStatus = navigateTask->tick(bb);
    EXPECT_EQ(navStatus, NodeStatus::Running);
    EXPECT_GT(state.commands.throttle, 0.0);

    // Move aircraft within capture radius (e.g. to (0, 8000)).
    as.kin.x = 0.0;
    as.kin.y = 8000.0;
    NodeStatus checkStatus2 = captureCheck->tick(bb);
    EXPECT_EQ(checkStatus2, NodeStatus::Success);
    // Task should be advanced (now complete)
    EXPECT_TRUE(bb.flightPlan->isComplete());
}

TEST(StateTreeTest, LoiterTaskAndActiveSelector) {
    Blackboard bb;
    bb.flightPlan = std::make_shared<FlightPlan>();

    AircraftState as;
    as.kin.costhe = 1.0;
    as.kin.cosphi = 1.0;
    as.kin.gmma = 0.0;
    as.kin.sigma = 0.0;
    as.kin.singam = 0.0;
    as.kin.x = 0.0;
    as.kin.y = 0.0;
    as.kin.z = -10000.0;
    as.kin.zdot = 0.0;
    as.vcas = 350.0;
    as.kin.vt = 350.0 * KNOTS_TO_FTPSEC;
    bb.as = &as;

    FlightControlSystem fcs;
    FcsState fcsState;
    DigiState state;
    state.config.maxGs = 9.0;
    state.config.maxRoll = 45.0;
    state.config.maxGammaDeg = 15.0;
    bb.fcs = &fcs;
    bb.fcsState = &fcsState;
    bb.state = &state;
    bb.dt = 1.0 / 60.0;

    MissionTask task1{TaskType::CAP, {0.0, 0.0, -10000.0}, 300, 10000, kInvalidEntityId, 10.0}; // 10s CAP loiter
    bb.flightPlan->pushTask(task1);

    auto activeSelector = std::make_shared<ActiveTaskSelectorNode>();
    auto loiterNode = std::make_shared<LoiterTaskNode>();
    auto navigateNode = std::make_shared<NavigateTaskNode>();

    activeSelector->addTaskNode(TaskType::CAP, loiterNode);
    activeSelector->addTaskNode(TaskType::Navigate, navigateNode);

    // First tick of selector should dispatch to LoiterTaskNode and return Running
    NodeStatus status1 = activeSelector->tick(bb);
    EXPECT_EQ(status1, NodeStatus::Running);

    // Loop until loiter duration completes and returns Success
    NodeStatus finalStatus = NodeStatus::Running;
    int tickCount = 0;
    while (finalStatus == NodeStatus::Running && tickCount < 1000) {
        finalStatus = activeSelector->tick(bb);
        tickCount++;
    }
    EXPECT_EQ(finalStatus, NodeStatus::Success);
    EXPECT_EQ(tickCount, 599); // exactly 10.0s (600 total ticks) at 60Hz
}
