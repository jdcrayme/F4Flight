# F4Flight Digi AI State Tree Refactoring
## Conceptual Architecture and Design Specification

This document proposes a comprehensive, hypothetical redesign of the F4Flight Digital AI (`DigiBrain`) from its current priority-stack structure (`DigiMode` and `addMode` logic) to a **hybrid Behavior Tree (BT) and Hierarchical State Machine (HSM)**.

This redesign is motivated by the upcoming requirement to support formation-level operations directed by a dynamic Air Tasking Order (ATO) and Ground Controlled Intercept (GCI).

---

## 1. Why Refactor? The Current Priority-Stack Limitations

The current `DigiBrain` implementation uses a flat `enum class DigiMode` representing priority, where lower numerical values have higher priority (e.g., `GroundAvoid = 0`, `MissileDefeat = 1`, `Waypoint = 12`). In `resolveMode`, we evaluate conditions in a fixed sequence and use `addMode()` to queue candidate modes.

While this was highly optimized for early 2000s computing, it exhibits several architectural drawbacks:
1. **Implicit Interlocks:** Rules like "Landing can't be bumped by WVR once started" are scattered across `addMode()` as manual conditional blocks. Adding new behaviors creates combinatorial complexity and risk of regression.
2. **Coarse-Grained Transitions:** The brain must switch entirely between distinct modes (e.g., swapping `Waypoint` for `WVREngage`), making it difficult to maintain concurrent state (like tracking a flight plan while executing a defensive jink).
3. **Rigid Sequence Execution:** Implementing multi-phase actions (like Ground Attack's approach $\rightarrow$ dive $\rightarrow$ pullout $\rightarrow$ egress) requires keeping internal integer state variables (e.g., `agApproach` or `groundOps.phase`) inside the state struct, scattering the state machine logic across the codebase.
4. **Poor Scalability for Formations:** Flight leads and wingmen use the same mode evaluation loop. Wingman behaviors are shoehorned into `FollowOrders` or `Wingy` with ad-hoc checks to override normal waypoint navigation.

---

## 2. Proposed Architecture: The Hybrid State Tree

To solve these problems, we propose a **Behavior Tree (BT)** as the execution engine, augmented with a **Blackboard** and a nested **Hierarchical State Machine (HSM)** for stateful, sequential tasks (like takeoff, landing, and flight plan progression).

### Why Behavior Trees + HSM?
* **BTs** are exceptional at **reactive decision making** (handling threats, terrain avoidance, sensor management) through preemptive decorators and selectors.
* **HSMs** are exceptional at **sequential execution** (flight plans, takeoff rolls, landing patterns, refueling sequences) where the system needs to maintain historical context and progress through strict phases.
* Combining them lets a BT node act as the controller for a sub-HSM, giving us the best of both worlds: high-level reactivity and robust sequence tracking.

```
                                  [ Root Selector ]
                                         │
        ┌────────────────────────────────┼──────────────────────────────┐
        ▼                                ▼                              ▼
[ Critical Safety ]              [ Combat Reactivity ]          [ Mission Execution ]
  (Ground/Collision)              (Missile/Guns Defeat)                 │
                                                                        ▼
                                                             [ FollowFlightPlan HSM ]
                                                                        │
                                                ┌───────────────────────┼───────────────────────┐
                                                ▼                       ▼                       ▼
                                           (Navigate)               (Engage Tgt)            (Re-Route)
```

---

## 3. High-Level System Layers

We organize the AI functionality into four distinct, highly decoupled layers:

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 4: STRATEGIC / MISSION (The Flight Plan & ATO)                        │
│   - Manages global objectives (CAP, Strike, Refueling, RTB).                 │
│   - Receives dynamic updates from ATO/GCI.                                   │
│   - Tracks position in flight plan (Sequential orders).                       │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       ▼ (Drives Target, Route, and Role)
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3: BEHAVIORAL / TACTICAL (Behavior Tree & Decision Trees)              │
│   - Evaluates defensive reactions (Missile Defeat, Guns Jink).               │
│   - Executes offensive maneuvers (BVR Intercept, WVR BFM, Merge).            │
│   - Coordinates Lead/Wingman tactics (Pince, Flex, Rejoin).                  │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       ▼ (Selects Maneuver Primitive & Parameters)
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: MANEUVER / GUIDANCE (Maneuver Primitives)                           │
│   - Directs the aircraft's 3D spatial objectives.                            │
│   - HeadingAndAltitudeHold, Loiter, RollOutOfPlane, BugOut, WaypointCapture.  │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       ▼ (Provides Target Attitude & Speed)
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: DIRECT CONTROL LOOPS (Autopilot & FCS Core)                         │
│   - Translates physical errors into stick and throttle commands.             │
│   - PitchHold, RollHold, GammaHold, MachHold, PhugoidDamper.                 │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Layer Interaction Pattern:
* **Downwards (Commands):** Higher layers command lower layers by setting parameters on the **Blackboard** or invoking lower-level APIs. For example, Layer 3 (offensive) decides to perform a high-G turn, selects the `RollAndPull` maneuver in Layer 2, which in turn commands specific pitch angles (`GammaHold`) and airspeeds (`MachHold`) in Layer 1.
* **Upwards (Feedback):** Lower layers report status upwards (e.g., `ManeuverComplete`, `LimitReached`, `TargetLocked`).

---

## 4. The Unified Flight Plan Structure

To support the dynamic ATO and GCI coordination, we introduce a structured, data-driven `FlightPlan` class. A flight plan is not just a list of raw 3D coordinates, but a sequential queue of **Mission Tasks**.

```cpp
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
    EntityId targetId;       // Ground target or refueling tanker ID
    double durationSec;      // Time-limit (e.g., loiter/CAP time)
};

class FlightPlan {
public:
    void pushTask(MissionTask task);
    void insertEmergencyTask(MissionTask task); // Prepend (e.g., GCI divert)
    void clear();

    bool isComplete() const;
    const MissionTask& currentTask() const;
    void advanceTask();

private:
    std::deque<MissionTask> tasks_;
    size_t currentTaskIndex_ {0};
};
```

### Dynamic Retasking & GCI Di-verts:
When GCI commands an emergency intercept or the ATO pushes a dynamic retasking, the orchestrator does not need to rewrite the entire AI logic. It simply **replaces the `FlightPlan` queue** on the blackboard.
The `FollowFlightPlan` HSM state in the BT detects this change, resets its current task execution pointer, and immediately executes the new task (e.g., transitioning from a `Navigate` task to a `CAP` or `Intercept` task).

---

## 5. Behavior Tree Node Layout

Below is the conceptual layout of the `DigiAI` Behavior Tree. The tree is traversed from left to right, top to bottom, on every tick (e.g., 20 Hz).

```
                                              [ Root Selector ]
                                                      │
         ┌────────────────────────┬───────────────────┴───────────────┬────────────────────────┐
         ▼                        ▼                                   ▼                        ▼
  [ Safety Check ]        [ Defensive Check ]               [ Flight Status ]         [ Mission Execution ]
  (Force Pull-Up)         (Evasive Maneuvers)               (Takeoff/Landing)         (Follow Flight Plan)
         │                        │                                   │                        │
         ▼                        ▼                                   ▼                        ▼
┌──────────────────┐    ┌──────────────────┐                ┌──────────────────┐     ┌──────────────────┐
│ Selector         │    │ Selector         │                │ Selector         │     │ Sequence         │
│ ├─ GroundAvoid   │    │ ├─ MissileDefeat │                │ ├─ TakeoffPhase  │     │ ├─ IsInFormation │
│ └─ CollisionAvoid│    │ └─ GunsJink      │                │ └─ LandingPhase  │     │ │   (Wingman Follow)
└──────────────────┘    └──────────────────┘                └──────────────────┘     │ └─ ExecuteTask   │
                                                                                     │    (Active Task) │
                                                                                     └──────────────────┘
```

### Safety and Defensive Overrides as Reactive Decorators:
In a Behavior Tree, `GroundAvoid`, `CollisionAvoid`, and `MissileDefeat` live on the **leftmost high-priority branches**.
Because a `Selector` executes the first child that returns `RUNNING` or `SUCCESS`, if an incoming missile is detected:
1. The `Defensive Check` branch immediately succeeds and runs `MissileDefeat`.
2. This **preempts** the lower-priority `Mission Execution` branch.
3. Once the missile is defeated, `Defensive Check` returns `FAILURE` on the next tick.
4. The tree naturally falls back to `Mission Execution`, resuming `FollowFlightPlan` exactly where it left off, without any custom mode-switching code.

---

## 6. The Lead-Wingman Relationship and Promotion

Rather than hardcoding discrete behaviors for wingmen, the tree uses a **Blackboard-driven Role System**.

```cpp
struct FlightRole {
    bool isLead {true};
    EntityId leadId {kInvalidEntityId};
    FormationSlot slot {FormationSlot::Trail};
};
```

### Formation Logic in the Tree:
The `Mission Execution` branch utilizes a conditional decorator:

```
                    [ Mission Execution Selector ]
                                  │
         ┌────────────────────────┴────────────────────────┐
         ▼                                                 ▼
   [ Is Wingman? ]                                    [ Is Lead? ]
         │                                                 │
         ▼                                                 ▼
[ Wingman Follow Sequence ]                       [ Execute Active Task ]
  ├─ Follow Lead's Flight Path                      ├─ Navigate / Strike / CAP
  └─ Execute Received Flight Commands               └─ Direct Wingman Formations
```

### Promotion and Failover:
If the flight lead is shot down or suffers a critical failure:
1. GCI or the internal team-communication bus detects the loss.
2. The wingman's blackboard is updated: `role.isLead = true`, `role.leadId = selfId`.
3. On the next tree tick, the `Is Wingman?` decorator fails.
4. The tree routes execution to the `Is Lead?` branch.
5. The promoted wingman **takes ownership of the Flight Plan** and begins executing lead-specific behaviors, ensuring mission continuity.

---

## 7. Concrete Comparison: Priority Stack vs. State Tree

| Metric / Scenario | Current Priority-Stack AI | Proposed State Tree AI |
| :--- | :--- | :--- |
| **Safety Interlocks** | Scattered across `addMode()` (e.g., `nextMode == Landing && newMode == WVREngage` guards). | Handled by structural placement in the tree (Safety is leftmost). No interlock code required. |
| **Adding a Behavior** | Must add a value to `DigiMode`, update priority numbers, modify `addMode()` rules, and edit the massive switch statement. | Create a self-contained Node class and attach it to the appropriate branch. |
| **Stateful Sequences** | Tracked via auxiliary variables (`agApproach` counters) directly within state structures, risking dirty states. | Managed cleanly by active Node lifetimes, state enters/exits, and sub-HSMs. |
| **Dynamic Retasking** | Requires external systems to force a mode or manually wipe and re-inject waypoints. | Merely requires swapping the `FlightPlan` queue on the blackboard; the tree adapts on the next tick. |
| **Wingman Integration** | Wingmen are forced into specialized modes (`Wingy`, `FollowOrders`) overriding default logic. | Wingmen and leads share the same tree, branching cleanly based on the `FlightRole` blackboard variable. |

---

## 8. Summary of Benefits

Refactoring to a State Tree structure provides:
1. **Unparalleled Modularization:** Separation of tactical decision-making, flight control, and mission planning makes the codebase highly maintainable.
2. **Deterministic Preemption:** Natural hierarchical priority eliminates the spaghetti code of custom mode transition logic.
3. **Preparedness for Dynamic ATO/GCI:** Direct support for dynamic, queue-based flight plans fits perfectly with modern tactical simulation requirements.
