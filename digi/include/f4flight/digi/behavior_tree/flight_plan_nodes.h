#pragma once

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/airspeed_conversions.h"
#include "f4flight/flight/core/constants.h"
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

    void reset() override {
        BehaviorNode::reset();
        startPosInitialized_ = false;
    }

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

        size_t curIdx = bb.flightPlan->currentTaskIndex();
        bool hasNext = (curIdx + 1 < bb.flightPlan->tasks().size());
        double D = 0.0;

        if (hasNext && bb.state) {
            Vec3 A;
            if (curIdx > 0) {
                A = bb.flightPlan->tasks()[curIdx - 1].location;
                startPosInitialized_ = false;
            } else {
                if (!startPosInitialized_) {
                    startPos_ = Vec3{bb.as->kin.x, bb.as->kin.y, bb.as->kin.z};
                    startPosInitialized_ = true;
                }
                A = startPos_;
            }

            const auto& B = task.location;
            const auto& C = bb.flightPlan->tasks()[curIdx + 1].location;

            const double ABx = B.x - A.x;
            const double ABy = B.y - A.y;
            const double AB_len = std::sqrt(ABx * ABx + ABy * ABy);

            const double BCx = C.x - B.x;
            const double BCy = C.y - B.y;
            const double BC_len = std::sqrt(BCx * BCx + BCy * BCy);

            if (AB_len > 1e-3 && BC_len > 1e-3) {
                const double ux = ABx / AB_len;
                const double uy = ABy / AB_len;
                const double vx = BCx / BC_len;
                const double vy = BCy / BC_len;

                double cos_theta = ux * vx + uy * vy;
                double theta = std::acos(std::max(-1.0, std::min(1.0, cos_theta)));
                if (theta > 150.0 * 0.017453292519943295) {
                    theta = 150.0 * 0.017453292519943295;
                }

                const double V = bb.as->kin.vt;
                constexpr double phi_turn_rad = 35.0 * 0.017453292519943295;
                double R = (V * V) / (32.177 * std::tan(phi_turn_rad));
                D = R * std::tan(theta / 2.0);

                const double max_D = 0.5 * std::min(AB_len, BC_len);
                if (D > max_D) {
                    D = max_D;
                }
            }
        }

        double effectiveCaptureRadius = captureRadius_;
        if (hasNext && D > 0.0) {
            effectiveCaptureRadius = std::max(captureRadius_, D);
        }

        if (dist < effectiveCaptureRadius) {
            bb.flightPlan->advanceTask();
            startPosInitialized_ = false;
            if (bb.flightPlan->isComplete() && bb.state) {
                bb.state->nav.holdPsi = bb.as->kin.sigma;
                bb.state->nav.holdAlt = -bb.as->kin.z;
            }
        }

        return NodeStatus::Success;
    }

private:
    double captureRadius_;
    bool startPosInitialized_{false};
    Vec3 startPos_{0.0, 0.0, 0.0};
};

// ===========================================================================
// NavigateTaskNode
// ===========================================================================
class NavigateTaskNode : public BehaviorNode {
public:
    NavigateTaskNode() : BehaviorNode("NavigateTask") {}

    void reset() override {
        BehaviorNode::reset();
        startPosInitialized_ = false;
    }

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

        Vec3 A;
        size_t curIdx = bb.flightPlan->currentTaskIndex();
        if (curIdx > 0) {
            A = bb.flightPlan->tasks()[curIdx - 1].location;
            startPosInitialized_ = false;
        } else {
            if (!startPosInitialized_) {
                startPos_ = Vec3{bb.as->kin.x, bb.as->kin.y, bb.as->kin.z};
                startPosInitialized_ = true;
            }
            A = startPos_;
        }

        const double ABx = task.location.x - A.x;
        const double ABy = task.location.y - A.y;
        const double AB_len = std::sqrt(ABx * ABx + ABy * ABy);

        double desHeading = 0.0;
        if (AB_len < 1e-3) {
            const double dx = task.location.x - bb.as->kin.x;
            const double dy = task.location.y - bb.as->kin.y;
            desHeading = std::atan2(dy, dx);
        } else {
            const double courseHeading = std::atan2(ABy, ABx);
            const double APx = bb.as->kin.x - A.x;
            const double APy = bb.as->kin.y - A.y;
            const double d_xtk = (APx * ABy - APy * ABx) / AB_len;

            constexpr double K_xtk = 0.00025;
            double correction = K_xtk * d_xtk;
            if (correction > HALF_PI) correction = HALF_PI;
            if (correction < -HALF_PI) correction = -HALF_PI;

            desHeading = courseHeading + correction;
        }

        const double desAlt = task.altFt > 0 ? task.altFt : -task.location.z;

        ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
            *bb.state, *bb.as, *bb.fcs, *bb.fcsState, bb.state->config.maxGs);
        ManeuverPrimitives::PhugoidDamper(*bb.state, *bb.as);

        ManeuverPrimitives::machHoldCas(cas_kts(task.speedKts), true,
            *bb.state, *bb.as, 200.0, 800.0, bb.dt, 700.0);

        return NodeStatus::Running;
    }

private:
    bool startPosInitialized_{false};
    Vec3 startPos_{0.0, 0.0, 0.0};
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
        ManeuverPrimitives::PhugoidDamper(*bb.state, *bb.as);
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
