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
