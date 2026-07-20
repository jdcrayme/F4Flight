# F4Flight Digi AI State Tree Implementation Strategy
## Architectural Bootstrap and Incremental Migration Plan

To move this refactoring from a conceptual proposal to an actionable implementation, we must bootstrap a lightweight **Behavior Tree (BT)** and **Blackboard** framework in C++17, and then incrementally migrate behaviors without disrupting existing tests.

This document outlines the core C++ structures required, shows how to model the Blackboard, and describes an **incremental migration path** to ensure 100% test compatibility at every step.

---

## 1. Core State Tree Types and Interfaces

We need a lightweight, highly efficient header-only or source-defined BT library that avoids dynamic memory allocations during the per-frame tick.

### Node Status and Base Class

```cpp
#pragma once

#include <memory>
#include <vector>
#include <string>

namespace f4flight {
namespace digi {

class Blackboard;

enum class NodeStatus {
    Success,
    Failure,
    Running
};

class BehaviorNode {
public:
    explicit BehaviorNode(std::string name) : name_(std::move(name)) {}
    virtual ~BehaviorNode() = default;

    // Tick wrapper: handles enter/exit lifecycles automatically
    NodeStatus tick(Blackboard& blackboard) {
        if (!hasEntered_) {
            onEnter(blackboard);
            hasEntered_ = true;
        }

        NodeStatus status = onTick(blackboard);

        if (status != NodeStatus::Running) {
            onExit(blackboard, status);
            hasEntered_ = false;
        }
        return status;
    }

    const std::string& name() const { return name_; }

protected:
    virtual void onEnter(Blackboard& /*bb*/) {}
    virtual NodeStatus onTick(Blackboard& bb) = 0;
    virtual void onExit(Blackboard& /*bb*/, NodeStatus /*status*/) {}

private:
    std::string name_;
    bool hasEntered_ {false};
};

using BehaviorNodePtr = std::shared_ptr<BehaviorNode>;

} // namespace digi
} // namespace f4flight
```

---

## 2. Composite Nodes (Control Flow)

The fundamental composites needed for our architecture are **Selectors** (fallback) and **Sequences**.

### Selector Node (OR Logic)
Ticks children in order until one returns `Running` or `Success`.

```cpp
class SelectorNode : public BehaviorNode {
public:
    using BehaviorNode::BehaviorNode;

    void addChild(BehaviorNodePtr child) {
        children_.push_back(std::move(child));
    }

protected:
    void onEnter(Blackboard&) override {
        currentChildIndex_ = 0;
    }

    NodeStatus onTick(Blackboard& bb) override {
        while (currentChildIndex_ < children_.size()) {
            NodeStatus status = children_[currentChildIndex_]->tick(bb);
            if (status != NodeStatus::Failure) {
                return status; // Running or Success
            }
            currentChildIndex_++;
        }
        return NodeStatus::Failure;
    }

private:
    std::vector<BehaviorNodePtr> children_;
    size_t currentChildIndex_ {0};
};
```

### Sequence Node (AND Logic)
Ticks children in order until one returns `Running` or `Failure`.

```cpp
class SequenceNode : public BehaviorNode {
public:
    using BehaviorNode::BehaviorNode;

    void addChild(BehaviorNodePtr child) {
        children_.push_back(std::move(child));
    }

protected:
    void onEnter(Blackboard&) override {
        currentChildIndex_ = 0;
    }

    NodeStatus onTick(Blackboard& bb) override {
        while (currentChildIndex_ < children_.size()) {
            NodeStatus status = children_[currentChildIndex_]->tick(bb);
            if (status != NodeStatus::Success) {
                return status; // Running or Failure
            }
            currentChildIndex_++;
        }
        return NodeStatus::Success;
    }

private:
    std::vector<BehaviorNodePtr> children_;
    size_t currentChildIndex_ {0};
};
```

---

## 3. The Blackboard: Decoupling State from Logic

A typesafe, shared context container ensures nodes can exchange data without hard dependencies on other nodes. We can use a strongly-typed, aggregate structure that bridges directly to the existing `DigiState` to maintain backward-compatibility.

```cpp
#include "f4flight/digi/digi_state.h"
#include "f4flight/flight/aircraft_state.h"

namespace f4flight {
namespace digi {

class Blackboard {
public:
    // References to the active, live simulation parameters
    const AircraftState* as {nullptr};
    double dt {0.0};
    double groundZ {0.0};

    // Core inputs and persistence state
    DigiState* state {nullptr};
    const DigiEntity* self {nullptr};
    const DigiEntity* target {nullptr};

    // Tactical role information
    bool isLead {true};
    EntityId leadId {kInvalidEntityId};

    // Active flight plan
    std::shared_ptr<FlightPlan> flightPlan;
};

} // namespace digi
} // namespace f4flight
```

---

## 4. Incremental Migration Plan (Risk-Free Implementation)

A total "stop-the-world" rewrite of `DigiBrain` would break dozens of sophisticated flight tests and unit test suites. Instead, we use an **incremental, side-by-side execution strategy** to migrate behaviors safely.

### Step 1: Wrap Legacy Primitives in Leaf Nodes
Rather than rewriting functions like `RollAndPull` or `runWaypoint`, we construct `ActionNode` wrapper classes that call them directly.

```cpp
class WaypointFollowNode : public BehaviorNode {
public:
    WaypointFollowNode() : BehaviorNode("WaypointFollow") {}

protected:
    NodeStatus onTick(Blackboard& bb) override {
        if (!bb.as || !bb.state) return NodeStatus::Failure;

        // Directly invoke the legacy runner with blackboard references
        runLegacyWaypoint(*bb.as, bb.dt, *bb.state);
        return NodeStatus::Running;
    }
};
```

### Step 2: Dual-Brain Validation (Shadow Executive)
During the early phases, we execute **both** the legacy priority solver and the new Behavior Tree in parallel:

```cpp
PilotInput DigiBrain::compute(const AircraftState& as, double dt, ...) {
    // 1. Run legacy resolver
    resolveMode(as, groundZ, dt);
    PilotInput legacyOutput = computeLegacyInput(as, dt);

    // 2. Run new state tree shadow execution
    blackboard_.as = &as;
    blackboard_.dt = dt;
    blackboard_.groundZ = groundZ;

    NodeStatus btStatus = rootNode_->tick(blackboard_);

    // 3. Compare, trace, and validate
    if (activeMode() != blackboard_.state->expectedModeFromBT) {
        // Log mismatch or issue warning on mismatch in development
    }

    // 4. Return the legacy output while validating the BT
    return legacyOutput;
}
```
This lets us verify that our safety conditions and selector trees match the exact behavior of the existing priority stack before flipping the production switch.

### Step 3: Flip the Switch
Once all custom scenario and fixture tests (such as BVR, formation follow, and AAR refueling) pass identically under shadow-execution, we remove the legacy `resolveMode()` and direct `compute()` to execute the BT natively.
