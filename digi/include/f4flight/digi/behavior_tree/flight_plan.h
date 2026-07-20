#pragma once

#include "f4flight/flight/core/types.h"
#include <deque>
#include <utility>

namespace f4flight {
namespace digi {

enum class TaskType {
    Takeoff,
    Navigate,
    Assemble,   // Join formation
    CAP,        // Combat Air Patrol (orbit / defend area)
    Strike,      // Air-to-ground delivery
    Refuel,     // Aerial refueling contact
    Landing,
    RTB
};

struct MissionTask {
    TaskType type;
    Vec3 location;           // Target position, waypoint, or runway threshold
    double speedKts;         // Intended speed for this segment
    double altFt;            // Intended altitude
    EntityId targetId {kInvalidEntityId};       // Ground target or refueling tanker ID
    double durationSec;      // Time-limit (e.g., loiter/CAP time)
};

class FlightPlan {
public:
    void pushTask(MissionTask task) {
        tasks_.push_back(std::move(task));
    }

    void insertEmergencyTask(MissionTask task) {
        // Prepend before the current task index so it becomes the active task immediately
        if (currentTaskIndex_ <= tasks_.size()) {
            tasks_.insert(tasks_.begin() + currentTaskIndex_, std::move(task));
        } else {
            tasks_.push_back(std::move(task));
            currentTaskIndex_ = tasks_.size() - 1;
        }
    }

    void clear() {
        tasks_.clear();
        currentTaskIndex_ = 0;
    }

    bool isComplete() const {
        return currentTaskIndex_ >= tasks_.size();
    }

    const MissionTask& currentTask() const {
        if (isComplete()) {
            static const MissionTask defaultTask{TaskType::RTB, {}, 0.0, 0.0, kInvalidEntityId, 0.0};
            return defaultTask;
        }
        return tasks_[currentTaskIndex_];
    }

    void advanceTask() {
        if (!isComplete()) {
            currentTaskIndex_++;
        }
    }

    const std::deque<MissionTask>& tasks() const { return tasks_; }
    size_t currentTaskIndex() const { return currentTaskIndex_; }

private:
    std::deque<MissionTask> tasks_;
    size_t currentTaskIndex_ {0};
};

} // namespace digi
} // namespace f4flight
