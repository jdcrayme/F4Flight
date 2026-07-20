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
    bool hasEntered() const { return hasEntered_; }

protected:
    virtual void onEnter(Blackboard& /*bb*/) {}
    virtual NodeStatus onTick(Blackboard& bb) = 0;
    virtual void onExit(Blackboard& /*bb*/, NodeStatus /*status*/) {}

private:
    std::string name_;
    bool hasEntered_ {false};
};

using BehaviorNodePtr = std::shared_ptr<BehaviorNode>;

class SelectorNode : public BehaviorNode {
public:
    using BehaviorNode::BehaviorNode;

    void addChild(BehaviorNodePtr child) {
        children_.push_back(std::move(child));
    }

    const std::vector<BehaviorNodePtr>& children() const { return children_; }

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

class SequenceNode : public BehaviorNode {
public:
    using BehaviorNode::BehaviorNode;

    void addChild(BehaviorNodePtr child) {
        children_.push_back(std::move(child));
    }

    const std::vector<BehaviorNodePtr>& children() const { return children_; }

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

} // namespace digi
} // namespace f4flight
