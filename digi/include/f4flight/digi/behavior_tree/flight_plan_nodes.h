#pragma once

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/airspeed_conversions.h"
#include "f4flight/flight/core/constants.h"
#include "f4flight/digi/steering.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

namespace f4flight {
namespace digi {

// ===========================================================================
// WaypointCaptureCheckNode
// ===========================================================================
class WaypointCaptureCheckNode : public BehaviorNode {
public:
    explicit WaypointCaptureCheckNode(double captureRadius = 5000.0)
        : BehaviorNode("WaypointCaptureCheck"), captureRadius_(captureRadius) {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (!bb.as) return NodeStatus::Failure;

        const double dx = task.location.x - bb.as->kin.x;
        const double dy = task.location.y - bb.as->kin.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < captureRadius_) {
            bb.flightPlan->advanceTask();
        }

        return NodeStatus::Success;
    }

private:
    double captureRadius_;
};

// ===========================================================================
// NavigateTaskNode
// ===========================================================================
class NavigateTaskNode : public BehaviorNode {
public:
    NavigateTaskNode() : BehaviorNode("NavigateTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Navigate) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        const double dx = task.location.x - bb.as->kin.x;
        const double dy = task.location.y - bb.as->kin.y;
        const double desHeading = std::atan2(dy, dx);
        const double desAlt = task.altFt > 0 ? task.altFt : -task.location.z;

        ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
            *bb.state, *bb.as, *bb.fcs, *bb.fcsState, bb.state->config.maxGs);

        ManeuverPrimitives::machHoldCas(cas_kts(task.speedKts), true,
            *bb.state, *bb.as, 200.0, 800.0, bb.dt, 700.0);

        return NodeStatus::Running;
    }
};

// ===========================================================================
// LoiterTaskNode (CAP/Orbit behavior)
// ===========================================================================
class LoiterTaskNode : public BehaviorNode {
public:
    LoiterTaskNode() : BehaviorNode("LoiterTask") {}

protected:
    void onEnter(Blackboard&) override {
        timer_ = 0.0;
    }

    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::CAP) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // Steer toward and orbit the loiter location
        ManeuverPrimitives::Loiter(*bb.state, *bb.as, *bb.fcs, *bb.fcsState, bb.state->config.maxGs);
        ManeuverPrimitives::machHoldCas(cas_kts(task.speedKts), true,
            *bb.state, *bb.as, 200.0, 800.0, bb.dt, 700.0);

        timer_ += bb.dt;
        if (task.durationSec > 0.0 && timer_ >= task.durationSec) {
            return NodeStatus::Success; // completed loiter duration
        }

        return NodeStatus::Running;
    }

private:
    double timer_{0.0};
};

// ===========================================================================
// TakeoffTaskNode - executes takeoff maneuver from flight plan
// ===========================================================================
class TakeoffTaskNode : public BehaviorNode {
public:
    TakeoffTaskNode() : BehaviorNode("TakeoffTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Takeoff) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // Execute takeoff using the brain's runTakeoff method
        bb.brain->setCurMode(DigiMode::Takeoff);
        bb.state->nav.flightPhase = FlightPhase::GroundOps;
        bb.brain->runTakeoff(*bb.as, bb.dt, *bb.fcsState, bb.groundZ);

        // Check if takeoff is complete (airborne and past rotation)
        const auto gp = bb.state->ag.groundOps.phase;
        const bool takeoffComplete = (gp == GroundOpsPhase::AfterTakeoff);
        
        if (takeoffComplete) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

// ===========================================================================
// LandingTaskNode - executes landing approach from flight plan
// ===========================================================================
class LandingTaskNode : public BehaviorNode {
public:
    LandingTaskNode() : BehaviorNode("LandingTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Landing) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // Set target altitude from task for approach
        if (task.altFt > 0) {
            bb.state->nav.holdAlt = task.altFt;
        }

        // Execute landing using the brain's runLanding method
        bb.brain->setCurMode(DigiMode::Landing);
        bb.state->nav.flightPhase = FlightPhase::Approach;
        bb.brain->runLanding(*bb.as, bb.dt, *bb.fcsState, bb.groundZ);

        // Check if landing is complete (on ground)
        const auto gp = bb.state->ag.groundOps.phase;
        const bool landingComplete = (gp == GroundOpsPhase::Parking || 
                                      gp == GroundOpsPhase::HoldingShort);
        
        if (landingComplete) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

// ===========================================================================
// RTBTaskNode - executes Return To Base from flight plan
// ===========================================================================
class RTBTaskNode : public BehaviorNode {
public:
    RTBTaskNode() : BehaviorNode("RTBTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::RTB) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state || !bb.self) {
            return NodeStatus::Failure;
        }

        // Execute RTB using the brain's runRTB method
        bb.brain->setCurMode(DigiMode::RTB);
        bb.state->nav.flightPhase = FlightPhase::Cruise;
        bb.brain->runRTB(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);

        // RTB task completes when we transition to landing or reach base
        // For now, always return Running - the mode will handle completion
        return NodeStatus::Running;
    }
};

// ===========================================================================
// RefuelTaskNode - executes aerial refueling from flight plan
// ===========================================================================
class RefuelTaskNode : public BehaviorNode {
public:
    RefuelTaskNode() : BehaviorNode("RefuelTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Refuel) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // Execute refueling using the brain's runRefueling method
        bb.brain->setCurMode(DigiMode::Refueling);
        bb.state->nav.flightPhase = FlightPhase::Formation;
        bb.brain->runRefueling(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);

        // Check if refueling is complete
        if (bb.state->refuel.refuelComplete) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

// ===========================================================================
// StrikeTaskNode - executes air-to-ground strike from flight plan
// ===========================================================================
class StrikeTaskNode : public BehaviorNode {
public:
    StrikeTaskNode() : BehaviorNode("StrikeTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Strike) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // Set target location from task
        // The groundTarget should already be set via FrameInputs.injectedGroundTarget
        
        // Execute ground attack using the brain's runGroundAttack method
        bb.brain->setCurMode(DigiMode::GroundMnvr);
        bb.state->nav.flightPhase = FlightPhase::Combat;
        bb.brain->runGroundAttack(*bb.as, bb.dt, *bb.fcs, *bb.fcsState);

        // Check if strike is complete (weapon released or egress complete)
        if (bb.state->ag.agAttackPhase == AgAttackPhase::EgressComplete) {
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

// ===========================================================================
// AssembleTaskNode - executes formation join from flight plan
// ===========================================================================
class AssembleTaskNode : public BehaviorNode {
public:
    AssembleTaskNode() : BehaviorNode("AssembleTask") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Assemble) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // For assembly, navigate to the task location and hold position
        // This would typically involve formation flying to join up
        const double dx = task.location.x - bb.as->kin.x;
        const double dy = task.location.y - bb.as->kin.y;
        const double desHeading = std::atan2(dy, dx);
        const double desAlt = task.altFt > 0 ? task.altFt : -task.location.z;

        ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
            *bb.state, *bb.as, *bb.fcs, *bb.fcsState, bb.state->config.maxGs);

        ManeuverPrimitives::machHoldCas(cas_kts(task.speedKts), true,
            *bb.state, *bb.as, 200.0, 800.0, bb.dt, 700.0);

        // Check distance to assembly point
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 500.0) {  // Within 500 ft of assembly point
            return NodeStatus::Success;
        }
        return NodeStatus::Running;
    }
};

// ===========================================================================
// ActiveTaskSelectorNode (Dispatches to the active task's corresponding behavior)
// ===========================================================================
class ActiveTaskSelectorNode : public BehaviorNode {
public:
    ActiveTaskSelectorNode() : BehaviorNode("ActiveTaskSelector") {}

    void addTaskNode(TaskType type, BehaviorNodePtr node) {
        taskNodes_[type] = std::move(node);
    }

    void reset() override {
        BehaviorNode::reset();
        for (auto& kv : taskNodes_) {
            if (kv.second) kv.second->reset();
        }
    }

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        auto it = taskNodes_.find(task.type);
        if (it != taskNodes_.end() && it->second) {
            return it->second->tick(bb);
        }

        return NodeStatus::Failure;
    }

private:
    std::unordered_map<TaskType, BehaviorNodePtr> taskNodes_;
};

} // namespace digi
} // namespace f4flight
