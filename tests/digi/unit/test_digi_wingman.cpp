// f4flight - test_digi_wingman.cpp
//
// Unit tests for the WingmanState + receiveOrders command dispatch.
// Round-3 structural addition (DIGI_AUDIT_ROUND3.md §3.2).

#include <gtest/gtest.h>
#include "f4flight/digi/wingman/wingman_state.h"
#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/ground/ag_attack_phase.h"

using namespace f4flight::digi;

TEST(WingmanStateTest, ResetClearsAllState) {
    WingmanState ws;
    ws.actionFlags[static_cast<int>(WingmanAction::RTB)] = 1;
    ws.designatedTargetId = 42;
    ws.currentManeuver = WingmanManeuver::BreakLeft;
    ws.weaponsAction = WeaponsAction::Free;

    ws.reset();

    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::RTB)], 0);
    EXPECT_EQ(ws.designatedTargetId, kInvalidEntityId);
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(ws.weaponsAction, WeaponsAction::Hold);
}

TEST(ReceiveOrdersTest, RTBCommandSetsActionFlag) {
    WingmanState ws;
    Message msg{MessageType::FlightCmdRTB, 1, 2};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::RTB)], 1);
}

TEST(ReceiveOrdersTest, EngageTargetSetsDesignatedTarget) {
    WingmanState ws;
    Message msg{MessageType::FlightCmdEngage, 1, 2};
    msg.payload.entityId = 999;
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::EngageTarget)], 1);
    EXPECT_EQ(ws.designatedTargetId, 999u);
    EXPECT_EQ(ws.designatedType, DesignatedType::Target);
    EXPECT_EQ(ws.weaponsAction, WeaponsAction::Free);
}

TEST(ReceiveOrdersTest, BreakCommandSetsManeuver) {
    WingmanState ws;
    Message msg{MessageType::FlightCmdBreak, 1, 2};
    msg.payload.heading = -1.0;  // left
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::BreakLeft);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 1);
}

TEST(ReceiveOrdersTest, FormationCommandsSetCurrentFormation) {
    WingmanState ws;
    Message msg{MessageType::FlightCmdTrail, 1, 2};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentFormation, f4flight::digi::formation::FormationType::Trail);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)], 1);
}

TEST(ReceiveOrdersTest, WeaponsHoldFree) {
    WingmanState ws;
    ws.weaponsAction = WeaponsAction::Free;
    Message msg{MessageType::FlightCmdWeaponsHold, 1, 2};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.weaponsAction, WeaponsAction::Hold);
}

TEST(ReceiveOrdersTest, NonWingmanMessageReturnsFalse) {
    WingmanState ws;
    Message msg{MessageType::ATCClearedTakeoff, 1, 2};
    EXPECT_FALSE(receiveOrders(ws, msg));
}

TEST(ReceiveOrdersTest, RejoinClearsManeuver) {
    WingmanState ws;
    ws.currentManeuver = WingmanManeuver::BreakLeft;
    ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
    Message msg{MessageType::FlightCmdRejoin, 1, 2};
    EXPECT_TRUE(receiveOrders(ws, msg));
    EXPECT_EQ(ws.currentManeuver, WingmanManeuver::None);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)], 0);
    EXPECT_EQ(ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)], 1);
}

TEST(AgAttackPhaseTest, EnumValuesAndNames) {
    EXPECT_EQ(agAttackPhaseName(AgAttackPhase::NotThereYet), "NotThereYet");
    EXPECT_EQ(agAttackPhaseName(AgAttackPhase::Final), "Final");
    EXPECT_EQ(agAttackPhaseName(AgAttackPhase::Stabalizing), "Stabalizing");
    // Verify the enum has 8 values (matching FreeFalcon's A/G phases)
    EXPECT_EQ(static_cast<int>(AgAttackPhase::Stabalizing), 7);
}
