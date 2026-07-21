#pragma once

#include "f4flight/digi/behavior_tree/node.h"
#include "f4flight/digi/behavior_tree/blackboard.h"
#include "f4flight/digi/behavior_tree/flight_plan_nodes.h"

namespace f4flight {
namespace digi {

class DigiBrain;

// ---------------------------------------------------------------------------
// 0. ForcedModeNode
// ---------------------------------------------------------------------------
class ForcedModeNode : public BehaviorNode {
public:
    ForcedModeNode() : BehaviorNode("ForcedMode") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 1. GroundAvoidNode
// ---------------------------------------------------------------------------
class GroundAvoidNode : public BehaviorNode {
public:
    GroundAvoidNode() : BehaviorNode("GroundAvoid") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 2. CollisionAvoidNode
// ---------------------------------------------------------------------------
class CollisionAvoidNode : public BehaviorNode {
public:
    CollisionAvoidNode() : BehaviorNode("CollisionAvoid") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 3. MissileDefeatNode
// ---------------------------------------------------------------------------
class MissileDefeatNode : public BehaviorNode {
public:
    MissileDefeatNode() : BehaviorNode("MissileDefeat") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 4. GunsJinkNode
// ---------------------------------------------------------------------------
class GunsJinkNode : public BehaviorNode {
public:
    GunsJinkNode() : BehaviorNode("GunsJink") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 5. TakeoffNode
// ---------------------------------------------------------------------------
class TakeoffNode : public BehaviorNode {
public:
    TakeoffNode() : BehaviorNode("Takeoff") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 6. LandingNode
// ---------------------------------------------------------------------------
class LandingNode : public BehaviorNode {
public:
    LandingNode() : BehaviorNode("Landing") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 7. FollowOrdersNode
// ---------------------------------------------------------------------------
class FollowOrdersNode : public BehaviorNode {
public:
    FollowOrdersNode() : BehaviorNode("FollowOrders") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 8. RTBNode
// ---------------------------------------------------------------------------
class RTBNode : public BehaviorNode {
public:
    RTBNode() : BehaviorNode("RTB") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 9. CombatNode (Handles BVR/WVR/Guns/Missile/Merge)
// ---------------------------------------------------------------------------
class CombatNode : public BehaviorNode {
public:
    CombatNode() : BehaviorNode("Combat") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 10. WingyNode
// ---------------------------------------------------------------------------
class WingyNode : public BehaviorNode {
public:
    WingyNode() : BehaviorNode("Wingy") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 11. RefuelNode
// ---------------------------------------------------------------------------
class RefuelNode : public BehaviorNode {
public:
    RefuelNode() : BehaviorNode("Refuel") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 12. GroundMnvrNode
// ---------------------------------------------------------------------------
class GroundMnvrNode : public BehaviorNode {
public:
    GroundMnvrNode() : BehaviorNode("GroundMnvr") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

// ---------------------------------------------------------------------------
// 13. WaypointFollowNode
// ---------------------------------------------------------------------------
class WaypointFollowNode : public BehaviorNode {
public:
    WaypointFollowNode();
    void reset() override;
protected:
    NodeStatus onTick(Blackboard& bb) override;
private:
    std::shared_ptr<WaypointCaptureCheckNode> captureCheckNode_;
    std::shared_ptr<ActiveTaskSelectorNode> activeTaskSelectorNode_;
};

// ---------------------------------------------------------------------------
// 14. DefaultFallbackNode
// ---------------------------------------------------------------------------
class DefaultFallbackNode : public BehaviorNode {
public:
    DefaultFallbackNode() : BehaviorNode("DefaultFallback") {}
protected:
    NodeStatus onTick(Blackboard& bb) override;
};

} // namespace digi
} // namespace f4flight
