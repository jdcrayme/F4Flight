// f4flight - digi/comms/message_bus.h
//
// MessageBus — routes messages between entities.
//
// The bus is the central hub for all communication. Senders publish messages
// addressed to a specific entity, a flight, a package, or broadcast. The bus
// resolves group addresses to individual entity mailboxes and delivers.
//
// Design:
//   - The bus owns no state except the group membership map.
//   - Mailboxes are registered by EntityId. The bus holds raw pointers —
//     the owner (DigiBrain) is responsible for unregistering on destruction.
//   - Group membership (flight, package) is registered by the host. The host
//     calls addToGroup() when forming flights and removeFromGroup() when
//     entities leave.
//   - Delivery is synchronous: publish() immediately writes to recipient
//     mailboxes. This is simple and sufficient for a single-threaded sim.
//     For multi-threaded, wrap publish() in a mutex.
//
// Comparison to FreeFalcon:
//   FreeFalcon routes messages through the VuMessage system, which is
//   designed for multiplayer network replication. That's overkill for an
//   in-process AI library and couples messaging to the network layer.
//   Our MessageBus is a simple in-process pub/sub — ~150 lines.

#pragma once

#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/comms/mailbox.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace f4flight {
namespace digi {

class MessageBus {
public:
    // Register a mailbox for an entity. The bus holds a raw pointer; the
    // caller must unregister before destroying the mailbox.
    void registerMailbox(EntityId id, Mailbox* mailbox) {
        mailboxes_[id] = mailbox;
    }

    void unregisterMailbox(EntityId id) {
        mailboxes_.erase(id);
    }

    // Group membership. A group is identified by a flight/package ID.
    // addToGroup(flightId, entityId) makes entityId a member of flightId.
    void addToGroup(EntityId groupId, EntityId entityId) {
        groups_[groupId].insert(entityId);
    }

    void removeFromGroup(EntityId groupId, EntityId entityId) {
        auto it = groups_.find(groupId);
        if (it != groups_.end()) {
            it->second.erase(entityId);
            if (it->second.empty()) groups_.erase(it);
        }
    }

    // Publish a message. The recipient field determines delivery:
    //   - kBroadcastId: deliver to all registered mailboxes
    //   - Specific EntityId: deliver to that entity only
    //   - Group address: resolve via groups_ and deliver to each member
    void publish(const Message& msg, double simTime) {
        Message timed = msg;
        timed.timestamp = simTime;

        if (msg.recipient == kBroadcastId) {
            // Broadcast to all
            for (auto& [id, mailbox] : mailboxes_) {
                if (id != msg.sender) {  // don't echo to sender
                    mailbox->push(timed);
                }
            }
        } else {
            // Specific recipient
            auto it = mailboxes_.find(msg.recipient);
            if (it != mailboxes_.end()) {
                it->second->push(timed);
            }
        }
    }

    // Publish to a group (flight or package). All members receive the message.
    void publishToGroup(EntityId groupId, const Message& msg, double simTime) {
        Message timed = msg;
        timed.timestamp = simTime;

        auto it = groups_.find(groupId);
        if (it != groups_.end()) {
            for (EntityId memberId : it->second) {
                if (memberId != msg.sender) {  // don't echo to sender
                    auto mbIt = mailboxes_.find(memberId);
                    if (mbIt != mailboxes_.end()) {
                        mbIt->second->push(timed);
                    }
                }
            }
        }
    }

    // Convenience: publish to an Address (resolves group vs specific).
    void publishToAddress(const Address& addr, const Message& msg, double simTime) {
        switch (addr.type) {
            case AddressType::Broadcast:
                publish(msg, simTime);
                break;
            case AddressType::SpecificEntity:
                publish(msg, simTime);
                break;
            case AddressType::Flight:
            case AddressType::Package:
                publishToGroup(addr.id, msg, simTime);
                break;
            case AddressType::ATC:
                // ATC is a specific entity — publish directly
                publish(msg, simTime);
                break;
        }
    }

    std::size_t registeredCount() const { return mailboxes_.size(); }

private:
    std::unordered_map<EntityId, Mailbox*> mailboxes_;
    std::unordered_map<EntityId, std::unordered_set<EntityId>> groups_;
};

} // namespace digi
} // namespace f4flight
