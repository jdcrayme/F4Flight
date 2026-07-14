// f4flight unit tests - digi AI communication system (Phase 1)
//
// Tests for:
//   - Message struct (message.h)
//   - Mailbox (mailbox.h)
//   - MessageBus (message_bus.h)
//   - ATC messages (atc_messages.h)
//   - ATC controller (atc_controller.h)
//   - TaxiGraph (taxi_graph.h)

#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/comms/message_bus.h"
#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/atc/atc_controller.h"
#include "f4flight/digi/atc/taxi_graph.h"

#include <gtest/gtest.h>

using namespace f4flight;
using namespace f4flight::digi;
using namespace f4flight::digi::atc;

// ===========================================================================
// Message tests
// ===========================================================================
TEST(MessageTest, DefaultConstruction) {
    Message m;
    EXPECT_EQ(m.type, MessageType::SystemPing);
    EXPECT_EQ(m.sender, kInvalidEntityId);
    EXPECT_EQ(m.recipient, kBroadcastId);
    EXPECT_NEAR(m.timestamp, 0.0, 1e-9);
}

TEST(MessageTest, ConstructWithTypeSenderRecipient) {
    Message m(MessageType::ATCClearedTakeoff, 100, 200);
    EXPECT_EQ(m.type, MessageType::ATCClearedTakeoff);
    EXPECT_EQ(m.sender, 100);
    EXPECT_EQ(m.recipient, 200);
}

TEST(MessageTest, PayloadDefaults) {
    Message m;
    EXPECT_NEAR(m.payload.x, 0.0, 1e-9);
    EXPECT_NEAR(m.payload.heading, 0.0, 1e-9);
    EXPECT_EQ(m.payload.entityId, kInvalidEntityId);
}

// ===========================================================================
// Mailbox tests
// ===========================================================================
TEST(MailboxTest, EmptyByDefault) {
    Mailbox mb;
    EXPECT_TRUE(mb.empty());
    EXPECT_EQ(mb.size(), 0u);
    EXPECT_FALSE(mb.peek().has_value());
    EXPECT_FALSE(mb.pop().has_value());
}

TEST(MailboxTest, PushAndPop) {
    Mailbox mb;
    Message m1(MessageType::ATCClearedTakeoff, 100, 200);
    Message m2(MessageType::ATCClearedLanding, 100, 200);

    mb.push(m1);
    mb.push(m2);

    EXPECT_EQ(mb.size(), 2u);
    EXPECT_FALSE(mb.empty());

    auto popped1 = mb.pop();
    ASSERT_TRUE(popped1.has_value());
    EXPECT_EQ(popped1->type, MessageType::ATCClearedTakeoff);

    auto popped2 = mb.pop();
    ASSERT_TRUE(popped2.has_value());
    EXPECT_EQ(popped2->type, MessageType::ATCClearedLanding);

    EXPECT_TRUE(mb.empty());
}

TEST(MailboxTest, PeekDoesNotRemove) {
    Mailbox mb;
    Message m(MessageType::ATCClearedTakeoff, 100, 200);
    mb.push(m);

    auto peeked = mb.peek();
    ASSERT_TRUE(peeked.has_value());
    EXPECT_EQ(peeked->type, MessageType::ATCClearedTakeoff);

    // Should still be there
    EXPECT_EQ(mb.size(), 1u);

    auto popped = mb.pop();
    ASSERT_TRUE(popped.has_value());
    EXPECT_EQ(popped->type, MessageType::ATCClearedTakeoff);
}

TEST(MailboxTest, FIFOOrder) {
    Mailbox mb;
    for (int i = 0; i < 5; ++i) {
        Message m;
        m.payload.value = i;
        mb.push(m);
    }

    for (int i = 0; i < 5; ++i) {
        auto popped = mb.pop();
        ASSERT_TRUE(popped.has_value());
        EXPECT_EQ(popped->payload.value, i);
    }
}

TEST(MailboxTest, Clear) {
    Mailbox mb;
    mb.push(Message());
    mb.push(Message());
    EXPECT_EQ(mb.size(), 2u);

    mb.clear();
    EXPECT_EQ(mb.size(), 0u);
    EXPECT_TRUE(mb.empty());
}

TEST(MailboxTest, FullDropsOldest) {
    Mailbox mb;
    // Fill to capacity — each push should return true (not full yet).
    for (std::size_t i = 0; i < Mailbox::kCapacity; ++i) {
        Message m;
        m.payload.value = static_cast<int>(i);
        EXPECT_TRUE(mb.push(m)) << "push " << i << " should return true (not full yet)";
    }
    EXPECT_EQ(mb.size(), Mailbox::kCapacity);
    EXPECT_TRUE(mb.full());

    // Push one more — oldest should be dropped, push should return false
    // (mailbox was full before the push).
    Message m;
    m.payload.value = 999;
    EXPECT_FALSE(mb.push(m)) << "push to full mailbox should return false";
    EXPECT_EQ(mb.size(), Mailbox::kCapacity);

    // First popped should be value=1 (0 was dropped)
    auto first = mb.pop();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->payload.value, 1);

    // Last should be 999
    while (mb.size() > 1) mb.pop();
    auto last = mb.pop();
    ASSERT_TRUE(last.has_value());
    EXPECT_EQ(last->payload.value, 999);
}

// ===========================================================================
// MessageBus tests
// ===========================================================================
TEST(MessageBusTest, RegisterUnregister) {
    MessageBus bus;
    Mailbox mb1, mb2;

    bus.registerMailbox(100, &mb1);
    bus.registerMailbox(200, &mb2);
    EXPECT_EQ(bus.registeredCount(), 2u);

    bus.unregisterMailbox(100);
    EXPECT_EQ(bus.registeredCount(), 1u);
}

TEST(MessageBusTest, PublishToSpecificRecipient) {
    MessageBus bus;
    Mailbox mb1, mb2;
    bus.registerMailbox(100, &mb1);
    bus.registerMailbox(200, &mb2);

    Message m(MessageType::ATCClearedTakeoff, 50, 100);
    bus.publish(m, 1.0);

    EXPECT_EQ(mb1.size(), 1u);
    EXPECT_EQ(mb2.size(), 0u);

    auto popped = mb1.pop();
    ASSERT_TRUE(popped.has_value());
    EXPECT_EQ(popped->type, MessageType::ATCClearedTakeoff);
    EXPECT_NEAR(popped->timestamp, 1.0, 1e-9);
}

TEST(MessageBusTest, PublishBroadcast) {
    MessageBus bus;
    Mailbox mb1, mb2, mb3;
    bus.registerMailbox(100, &mb1);
    bus.registerMailbox(200, &mb2);
    bus.registerMailbox(300, &mb3);

    Message m(MessageType::SystemPing, 50, kBroadcastId);
    bus.publish(m, 2.0);

    // All except sender should receive
    EXPECT_EQ(mb1.size(), 1u);
    EXPECT_EQ(mb2.size(), 1u);
    EXPECT_EQ(mb3.size(), 1u);
}

TEST(MessageBusTest, PublishToGroup) {
    MessageBus bus;
    Mailbox mb1, mb2, mb3;
    bus.registerMailbox(100, &mb1);
    bus.registerMailbox(200, &mb2);
    bus.registerMailbox(300, &mb3);

    // Form a flight: group 1000 contains entities 100 and 200
    bus.addToGroup(1000, 100);
    bus.addToGroup(1000, 200);

    Message m(MessageType::FlightCmdRejoin, 300, kBroadcastId);
    bus.publishToGroup(1000, m, 1.0);

    // Only group members receive (not sender)
    EXPECT_EQ(mb1.size(), 1u);
    EXPECT_EQ(mb2.size(), 1u);
    EXPECT_EQ(mb3.size(), 0u);  // not in group
}

TEST(MessageBusTest, RemoveFromGroup) {
    MessageBus bus;
    Mailbox mb1, mb2;
    bus.registerMailbox(100, &mb1);
    bus.registerMailbox(200, &mb2);

    bus.addToGroup(1000, 100);
    bus.addToGroup(1000, 200);

    Message m(MessageType::SystemPing, 50, kBroadcastId);
    bus.publishToGroup(1000, m, 1.0);
    EXPECT_EQ(mb1.size(), 1u);
    EXPECT_EQ(mb2.size(), 1u);

    mb1.clear();
    mb2.clear();

    bus.removeFromGroup(1000, 200);
    bus.publishToGroup(1000, m, 2.0);
    EXPECT_EQ(mb1.size(), 1u);
    EXPECT_EQ(mb2.size(), 0u);  // removed from group
}

// ===========================================================================
// ATC message factory tests
// ===========================================================================
TEST(ATCMessageTest, ClearanceRequest) {
    Message m = makeClearanceRequest(100, 200);
    EXPECT_EQ(m.type, MessageType::ATCClearanceRequest);
    EXPECT_EQ(m.sender, 100);
    EXPECT_EQ(m.recipient, 200);
}

TEST(ATCMessageTest, ClearedTakeoff) {
    Message m = makeClearedTakeoff(200, 100, 270);
    EXPECT_EQ(m.type, MessageType::ATCClearedTakeoff);
    EXPECT_EQ(m.sender, 200);
    EXPECT_EQ(m.recipient, 100);
    EXPECT_EQ(runwayFromMessage(m), 270);
}

TEST(ATCMessageTest, ClearedLanding) {
    Message m = makeClearedLanding(200, 100, 90);
    EXPECT_EQ(m.type, MessageType::ATCClearedLanding);
    EXPECT_EQ(runwayFromMessage(m), 90);
}

TEST(ATCMessageTest, HoldShort) {
    Message m = makeHoldShort(200, 100, 270);
    EXPECT_EQ(m.type, MessageType::ATCHoldShort);
    EXPECT_EQ(runwayFromMessage(m), 270);
}

TEST(ATCMessageTest, TaxiInstruction) {
    Message m = makeTaxiInstruction(200, 100, 1500.0, 3000.0);
    EXPECT_EQ(m.type, MessageType::ATCTaxiInstruction);

    double x, y;
    taxiPointFromMessage(m, x, y);
    EXPECT_NEAR(x, 1500.0, 1e-9);
    EXPECT_NEAR(y, 3000.0, 1e-9);
}

TEST(ATCMessageTest, GoAround) {
    Message m = makeGoAround(200, 100);
    EXPECT_EQ(m.type, MessageType::ATCGoAround);
    EXPECT_EQ(m.recipient, 100);
}

// ===========================================================================
// ATC controller tests
// ===========================================================================
class ATCControllerTest : public ::testing::Test {
protected:
    MessageBus bus;
    Mailbox aircraftMb;
    ATCController atc{200};  // ATC entity ID = 200
    static constexpr EntityId aircraftId = 100;

    void SetUp() override {
        bus.registerMailbox(aircraftId, &aircraftMb);
        bus.registerMailbox(atc.id(), &atc.mailbox());
        atc.addRunway(270);  // runway 27
    }
};

TEST_F(ATCControllerTest, IdleRunwayState) {
    EXPECT_EQ(atc.runwayState(270), RunwayState::Idle);
}

TEST_F(ATCControllerTest, ClearTakeoffRequest) {
    // Aircraft requests clearance
    bus.publish(makeClearanceRequest(aircraftId, atc.id()), 1.0);

    // ATC processes
    atc.update(1.0, bus);

    // Aircraft should have received takeoff clearance
    EXPECT_FALSE(aircraftMb.empty());
    auto msg = aircraftMb.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::ATCClearedTakeoff);
    EXPECT_EQ(runwayFromMessage(*msg), 270);
}

TEST_F(ATCControllerTest, QueueWhenRunwayBusy) {
    // First aircraft gets immediate clearance
    bus.publish(makeClearanceRequest(aircraftId, atc.id()), 1.0);
    atc.update(1.0, bus);
    EXPECT_FALSE(aircraftMb.empty());
    aircraftMb.clear();

    // Runway is now busy (Departing). Second aircraft should be queued.
    Mailbox mb2;
    bus.registerMailbox(101, &mb2);

    bus.publish(makeClearanceRequest(101, atc.id()), 2.0);
    atc.update(2.0, bus);

    // Second aircraft should get hold-short, not takeoff clearance
    EXPECT_FALSE(mb2.empty());
    auto msg = mb2.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::ATCHoldShort);

    EXPECT_EQ(atc.departureQueueSize(), 1u);
}

TEST_F(ATCControllerTest, RunwayClearsAfterVacated) {
    // Aircraft takes off
    bus.publish(makeClearanceRequest(aircraftId, atc.id()), 1.0);
    atc.update(1.0, bus);
    aircraftMb.clear();

    // Runway is now Departing
    EXPECT_EQ(atc.runwayState(270), RunwayState::Departing);

    // Aircraft reports it has vacated the runway
    bus.publish(Message(MessageType::ATCRunwayClear, aircraftId, atc.id()), 5.0);
    atc.update(5.0, bus);

    // Runway should now be Idle
    EXPECT_EQ(atc.runwayState(270), RunwayState::Idle);
}

TEST_F(ATCControllerTest, LandingRequest) {
    atc.handleLandingRequest(aircraftId, 1.0, bus);

    EXPECT_FALSE(aircraftMb.empty());
    auto msg = aircraftMb.pop();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->type, MessageType::ATCClearedLanding);
}

// ===========================================================================
// TaxiGraph tests
// ===========================================================================
TEST(TaxiGraphTest, AddNodesAndEdges) {
    TaxiGraph g;

    TaxiNode parking;
    parking.id = 0;
    parking.position = {0.0, 0.0, 0.0};
    parking.type = TaxiNodeType::ParkingSpot;
    EXPECT_TRUE(g.addNode(parking));  // fresh insertion

    TaxiNode holdShort;
    holdShort.id = 1;
    holdShort.position = {1000.0, 0.0, 0.0};
    holdShort.type = TaxiNodeType::HoldShort;
    holdShort.runway = 270;
    EXPECT_TRUE(g.addNode(holdShort));

    TaxiNode threshold;
    threshold.id = 2;
    threshold.position = {1100.0, 0.0, 0.0};
    threshold.type = TaxiNodeType::RunwayThreshold;
    threshold.runway = 270;
    EXPECT_TRUE(g.addNode(threshold));

    g.addEdge(0, 1);  // parking → hold short
    g.addEdge(1, 2);  // hold short → threshold

    EXPECT_EQ(g.nodeCount(), 3u);
    EXPECT_EQ(g.findRunwayThreshold(270), 2);
    EXPECT_EQ(g.findHoldShort(270), 1);
}

TEST(TaxiGraphTest, AddNodeReturnsFalseOnDuplicate) {
    // Bug L: addNode should return false when overwriting an existing id,
    // so callers can detect duplicates instead of silently losing data.
    TaxiGraph g;

    TaxiNode n0;
    n0.id = 5;
    n0.position = {100.0, 200.0, 0.0};
    n0.type = TaxiNodeType::ParkingSpot;
    EXPECT_TRUE(g.addNode(n0));

    // Same id, different position — should overwrite but return false.
    TaxiNode n0_dup;
    n0_dup.id = 5;
    n0_dup.position = {999.0, 999.0, 0.0};
    n0_dup.type = TaxiNodeType::ParkingSpot;
    EXPECT_FALSE(g.addNode(n0_dup));

    // Verify the position was overwritten (current behavior preserved).
    EXPECT_NEAR(g.node(5).position.x, 999.0, 1e-9);
}

TEST(TaxiGraphTest, AddNodeRejectsNegativeId) {
    TaxiGraph g;
    TaxiNode bad;
    bad.id = -1;
    bad.position = {0.0, 0.0, 0.0};
    bad.type = TaxiNodeType::ParkingSpot;
    EXPECT_FALSE(g.addNode(bad));
}

TEST(TaxiGraphTest, FindPath) {
    TaxiGraph g;

    for (int i = 0; i < 4; ++i) {
        TaxiNode n;
        n.id = i;
        n.position = {static_cast<double>(i * 1000), 0.0, 0.0};
        n.type = TaxiNodeType::Intersection;
        g.addNode(n);
    }

    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    auto path = g.findPath(0, 3);
    ASSERT_EQ(path.size(), 4u);
    EXPECT_EQ(path[0], 0);
    EXPECT_EQ(path[1], 1);
    EXPECT_EQ(path[2], 2);
    EXPECT_EQ(path[3], 3);
}

TEST(TaxiGraphTest, NoPath) {
    TaxiGraph g;

    TaxiNode a, b;
    a.id = 0; a.position = {0, 0, 0};
    b.id = 1; b.position = {1000, 0, 0};
    g.addNode(a);
    g.addNode(b);
    // No edge between them

    auto path = g.findPath(0, 1);
    EXPECT_TRUE(path.empty());
}

TEST(TaxiGraphTest, FindNearestByType) {
    TaxiGraph g;

    TaxiNode parking1;
    parking1.id = 0;
    parking1.position = {0.0, 0.0, 0.0};
    parking1.type = TaxiNodeType::ParkingSpot;
    g.addNode(parking1);

    TaxiNode parking2;
    parking2.id = 1;
    parking2.position = {2000.0, 0.0, 0.0};
    parking2.type = TaxiNodeType::ParkingSpot;
    g.addNode(parking2);

    // Search near parking2
    Vec3 pos = {1900.0, 0.0, 0.0};
    TaxiNodeId nearest = g.findNearest(pos, TaxiNodeType::ParkingSpot);
    EXPECT_EQ(nearest, 1);
}
