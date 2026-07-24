#pragma once

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/maneuvers/maneuver_primitives.h"
#include "f4flight/flight/core/airspeed_conversions.h"
#include "f4flight/flight/core/constants.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

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
        if (task.type == TaskType::Orbit) {
            return NodeStatus::Success;
        }

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

        double effectiveCaptureRadius = bb.brain ? bb.brain->captureRadius() : captureRadius_;
        if (hasNext && D > 0.0) {
            effectiveCaptureRadius = std::max(effectiveCaptureRadius, D);
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

            const double vx = bb.as->kin.xdot;
            const double vy = bb.as->kin.ydot;
            const double d_xtk_dot = (vx * ABy - vy * ABx) / AB_len;

            constexpr double K_p = 0.00025;
            constexpr double K_d = 0.00025;

            double psiErr = courseHeading - bb.as->kin.sigma;
            while (psiErr > M_PI) psiErr -= 2.0 * M_PI;
            while (psiErr < -M_PI) psiErr += 2.0 * M_PI;

            const double max_blend_err = 45.0 * 0.017453292519943295;
            double blend = 1.0 - (std::fabs(psiErr) / max_blend_err);
            if (blend < 0.0) blend = 0.0;
            if (blend > 1.0) blend = 1.0;

            double correction = blend * (K_p * d_xtk + K_d * d_xtk_dot);
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
// OrbitTaskNode (Racetrack Orbit behavior)
// ===========================================================================
enum class OrbitState {
    Ingress,
    OutboundTurn,
    OutboundLeg,
    InboundTurn,
    InboundLeg,
    ExitOrbit
};

class OrbitTaskNode : public BehaviorNode {
public:
    OrbitTaskNode() : BehaviorNode("OrbitTask") {}

    void reset() override {
        BehaviorNode::reset();
        state_ = OrbitState::Ingress;
        timer_ = 0.0;
        startPosInitialized_ = false;
    }

protected:
    void onEnter(Blackboard&) override {
        state_ = OrbitState::Ingress;
        timer_ = 0.0;
    }

    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.flightPlan || bb.flightPlan->isComplete()) {
            return NodeStatus::Failure;
        }

        const auto& task = bb.flightPlan->currentTask();
        if (task.type != TaskType::Orbit) {
            return NodeStatus::Failure;
        }

        if (!bb.as || !bb.fcs || !bb.fcsState || !bb.state) {
            return NodeStatus::Failure;
        }

        // 1. Resolve waypoints A (previous/start) and B (orbit waypoint)
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
        const Vec3& B = task.location;

        // Inbound course calculations
        const double ABx = B.x - A.x;
        const double ABy = B.y - A.y;
        const double AB_len = std::sqrt(ABx * ABx + ABy * ABy);
        double inboundCourse = 0.0;
        double ux = 1.0;
        double uy = 0.0;
        if (AB_len > 1.0) {
            inboundCourse = std::atan2(ABy, ABx);
            ux = ABx / AB_len;
            uy = ABy / AB_len;
        } else {
            inboundCourse = bb.as->kin.sigma;
            ux = std::cos(inboundCourse);
            uy = std::sin(inboundCourse);
        }

        // Perpendicular vector for parallel offset
        double nx = 0.0;
        double ny = 0.0;
        if (task.orbitDir == OrbitDirection::Left) {
            nx = -uy;
            ny = ux;
        } else {
            nx = uy;
            ny = -ux;
        }

        // Calculate offset based on dynamic turn radius R
        const double V = bb.as->kin.vt;
        constexpr double phi_turn_rad = 35.0 * 0.017453292519943295;
        double R = (V * V) / (32.177 * std::tan(phi_turn_rad));
        if (R < 1000.0) R = 5000.0;
        const double d = 2.0 * R;

        Vec3 A_out = A;
        A_out.x += d * nx;
        A_out.y += d * ny;

        Vec3 B_out = B;
        B_out.x += d * nx;
        B_out.y += d * ny;

        double desHeading = inboundCourse;
        double distToB = std::sqrt((B.x - bb.as->kin.x)*(B.x - bb.as->kin.x) + (B.y - bb.as->kin.y)*(B.y - bb.as->kin.y));
        double effectiveCaptureRadius = bb.brain ? bb.brain->captureRadius() : 5000.0;

        // Overall timer ticks during orbit phases (not ingress or exit)
        if (state_ != OrbitState::Ingress && state_ != OrbitState::ExitOrbit) {
            timer_ += bb.dt;
            if (timer_ >= task.durationSec) {
                state_ = OrbitState::ExitOrbit;
            }
        }

        // State Machine
        switch (state_) {
            case OrbitState::Ingress: {
                desHeading = computeCrossTrackHeading(A, B, *bb.as);
                if (distToB < effectiveCaptureRadius) {
                    state_ = OrbitState::OutboundTurn;
                    timer_ = 0.0;
                }
                break;
            }
            case OrbitState::OutboundTurn: {
                double outboundCourse = inboundCourse + M_PI;
                while (outboundCourse > M_PI) outboundCourse -= 2.0 * M_PI;
                while (outboundCourse < -M_PI) outboundCourse += 2.0 * M_PI;

                double diff = outboundCourse - bb.as->kin.sigma;
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;

                if (std::fabs(diff) <= 1.5) {
                    desHeading = outboundCourse;
                } else {
                    if (task.orbitDir == OrbitDirection::Left) {
                        desHeading = bb.as->kin.sigma + 1.5;
                    } else {
                        desHeading = bb.as->kin.sigma - 1.5;
                    }
                }

                if (std::fabs(diff) < 0.1) {
                    state_ = OrbitState::OutboundLeg;
                }
                break;
            }
            case OrbitState::OutboundLeg: {
                desHeading = computeCrossTrackHeading(B_out, A_out, *bb.as);

                const double SEx = A_out.x - B_out.x;
                const double SEy = A_out.y - B_out.y;
                const double SE_len = std::sqrt(SEx * SEx + SEy * SEy);
                const double SPx = bb.as->kin.x - B_out.x;
                const double SPy = bb.as->kin.y - B_out.y;
                const double progress = (SPx * SEx + SPy * SEy) / SE_len;
                if (progress >= SE_len) {
                    state_ = OrbitState::InboundTurn;
                }
                break;
            }
            case OrbitState::InboundTurn: {
                double diff = inboundCourse - bb.as->kin.sigma;
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;

                if (std::fabs(diff) <= 1.5) {
                    desHeading = inboundCourse;
                } else {
                    if (task.orbitDir == OrbitDirection::Left) {
                        desHeading = bb.as->kin.sigma + 1.5;
                    } else {
                        desHeading = bb.as->kin.sigma - 1.5;
                    }
                }

                if (std::fabs(diff) < 0.1) {
                    state_ = OrbitState::InboundLeg;
                }
                break;
            }
            case OrbitState::InboundLeg: {
                desHeading = computeCrossTrackHeading(A, B, *bb.as);
                if (distToB < effectiveCaptureRadius) {
                    state_ = OrbitState::OutboundTurn;
                }
                break;
            }
            case OrbitState::ExitOrbit: {
                desHeading = computeCrossTrackHeading(A, B, *bb.as);
                if (distToB < effectiveCaptureRadius) {
                    bb.flightPlan->advanceTask();
                    startPosInitialized_ = false;
                    if (bb.flightPlan->isComplete() && bb.state) {
                        bb.state->nav.holdPsi = bb.as->kin.sigma;
                        bb.state->nav.holdAlt = -bb.as->kin.z;
                    }
                    return NodeStatus::Success;
                }
                break;
            }
        }

        // Apply control commands
        const double desAlt = task.altFt > 0 ? task.altFt : -task.location.z;

        ManeuverPrimitives::HeadingAndAltitudeHold(desHeading, desAlt,
            *bb.state, *bb.as, *bb.fcs, *bb.fcsState, bb.state->config.maxGs);
        ManeuverPrimitives::PhugoidDamper(*bb.state, *bb.as);

        ManeuverPrimitives::machHoldCas(cas_kts(task.speedKts), true,
            *bb.state, *bb.as, 200.0, 800.0, bb.dt, 700.0);

        return NodeStatus::Running;
    }

private:
    double computeCrossTrackHeading(const Vec3& S, const Vec3& E, const AircraftState& as) {
        const double SEx = E.x - S.x;
        const double SEy = E.y - S.y;
        const double SE_len = std::sqrt(SEx * SEx + SEy * SEy);
        if (SE_len < 1.0) {
            const double dx = E.x - as.kin.x;
            const double dy = E.y - as.kin.y;
            return std::atan2(dy, dx);
        }

        const double courseHeading = std::atan2(SEy, SEx);
        const double SPx = as.kin.x - S.x;
        const double SPy = as.kin.y - S.y;
        const double d_xtk = (SPx * SEy - SPy * SEx) / SE_len;

        const double vx = as.kin.xdot;
        const double vy = as.kin.ydot;
        const double d_xtk_dot = (vx * SEy - vy * SEx) / SE_len;

        constexpr double K_p = 0.00025;
        constexpr double K_d = 0.00025;

        double psiErr = courseHeading - as.kin.sigma;
        while (psiErr > M_PI) psiErr -= 2.0 * M_PI;
        while (psiErr < -M_PI) psiErr += 2.0 * M_PI;

        const double max_blend_err = 45.0 * 0.017453292519943295;
        double blend = 1.0 - (std::fabs(psiErr) / max_blend_err);
        if (blend < 0.0) blend = 0.0;
        if (blend > 1.0) blend = 1.0;

        double correction = blend * (K_p * d_xtk + K_d * d_xtk_dot);
        if (correction > M_PI_2) correction = M_PI_2;
        if (correction < -M_PI_2) correction = -M_PI_2;

        return courseHeading + correction;
    }

    OrbitState state_{OrbitState::Ingress};
    double timer_{0.0};
    bool startPosInitialized_{false};
    Vec3 startPos_{0.0, 0.0, 0.0};
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
