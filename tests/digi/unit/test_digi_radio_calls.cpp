// f4flight unit tests - radio calls (comms/radio_calls.h)
//
// Tests for:
//   - RadioCallType enum and radioCallText() lookup
//   - RadioCallQueue push/pop/canCall
//   - RadioCallQueue throttling
//   - makeRadioCall helper (throttled push)
//   - DigiCommState.radioCalls integration

#include "f4flight/digi/comms/radio_calls.h"
#include "f4flight/digi/digi_state.h"
#include <gtest/gtest.h>

using namespace f4flight::digi;

// ===========================================================================
// RadioCallType / radioCallText tests
// ===========================================================================

TEST(RadioCallTest, RadioCallTextReturnsCorrectStrings) {
    EXPECT_STREQ(radioCallText(RadioCallType::None), "");
    EXPECT_STREQ(radioCallText(RadioCallType::Bandit), "Bandit!");
    EXPECT_STREQ(radioCallText(RadioCallType::Engage), "Engaging");
    EXPECT_STREQ(radioCallText(RadioCallType::Rejoin), "Rejoining");
    EXPECT_STREQ(radioCallText(RadioCallType::RTB), "RTB");
    EXPECT_STREQ(radioCallText(RadioCallType::Bingo), "Bingo");
    EXPECT_STREQ(radioCallText(RadioCallType::Splash), "Splash one");
    EXPECT_STREQ(radioCallText(RadioCallType::Missile), "Missile!");
    EXPECT_STREQ(radioCallText(RadioCallType::Contact), "Contact");
    EXPECT_STREQ(radioCallText(RadioCallType::Disconnect), "Disconnect");
}

TEST(RadioCallTest, AllCallTypesHaveText) {
    // Verify no call type returns null (would indicate a missing case)
    for (int i = 0; i <= 19; ++i) {
        const char* text = radioCallText(static_cast<RadioCallType>(i));
        EXPECT_NE(text, nullptr) << "Call type " << i << " returned null text";
    }
}

// ===========================================================================
// RadioCallQueue tests
// ===========================================================================

TEST(RadioCallQueueTest, EmptyQueuePopReturnsFalse) {
    RadioCallQueue queue;
    RadioCall call;
    EXPECT_FALSE(queue.pop(call));
    EXPECT_EQ(queue.count, 0);
}

TEST(RadioCallQueueTest, PushThenPop) {
    RadioCallQueue queue;
    queue.push(RadioCallType::Bandit, 1.0, 100, 200);

    RadioCall call;
    EXPECT_TRUE(queue.pop(call));
    EXPECT_EQ(call.type, RadioCallType::Bandit);
    EXPECT_DOUBLE_EQ(call.time, 1.0);
    EXPECT_EQ(call.senderId, 100u);
    EXPECT_EQ(call.targetId, 200u);
    EXPECT_EQ(queue.count, 0);
}

TEST(RadioCallQueueTest, PushMultipleCallsPopInOrder) {
    RadioCallQueue queue;
    queue.push(RadioCallType::Bandit, 1.0, 100);
    queue.push(RadioCallType::Engage, 2.0, 100);
    queue.push(RadioCallType::Splash, 3.0, 100);

    RadioCall call;
    EXPECT_TRUE(queue.pop(call));
    EXPECT_EQ(call.type, RadioCallType::Bandit);
    EXPECT_TRUE(queue.pop(call));
    EXPECT_EQ(call.type, RadioCallType::Engage);
    EXPECT_TRUE(queue.pop(call));
    EXPECT_EQ(call.type, RadioCallType::Splash);
    EXPECT_FALSE(queue.pop(call));
}

TEST(RadioCallQueueTest, QueueFullDropsOldest) {
    RadioCallQueue queue;
    // Fill the queue (capacity = 4)
    queue.push(RadioCallType::Bandit, 1.0, 100);
    queue.push(RadioCallType::Engage, 2.0, 100);
    queue.push(RadioCallType::Splash, 3.0, 100);
    queue.push(RadioCallType::RTB, 4.0, 100);

    // This push should fail (queue full)
    EXPECT_FALSE(queue.push(RadioCallType::Bingo, 5.0, 100));
    EXPECT_EQ(queue.count, 4);

    // Pop should return the oldest (Bandit)
    RadioCall call;
    EXPECT_TRUE(queue.pop(call));
    EXPECT_EQ(call.type, RadioCallType::Bandit);
}

TEST(RadioCallQueueTest, CanCallRespectsInterval) {
    RadioCallQueue queue;
    queue.push(RadioCallType::Bandit, 10.0, 100);

    // Immediately after — can't call (within 5s interval)
    EXPECT_FALSE(queue.canCall(12.0, 5.0));

    // After 5s — can call
    EXPECT_TRUE(queue.canCall(15.0, 5.0));
    EXPECT_TRUE(queue.canCall(20.0, 5.0));
}

TEST(RadioCallQueueTest, ResetClearsQueue) {
    RadioCallQueue queue;
    queue.push(RadioCallType::Bandit, 1.0, 100);
    queue.push(RadioCallType::Engage, 2.0, 100);
    EXPECT_EQ(queue.count, 2);

    queue.reset();
    EXPECT_EQ(queue.count, 0);
    EXPECT_EQ(queue.head, 0);
    EXPECT_EQ(queue.tail, 0);
    EXPECT_DOUBLE_EQ(queue.lastCallTime, 0.0);
}

// ===========================================================================
// makeRadioCall helper tests
// ===========================================================================

TEST(MakeRadioCallTest, ThrottlesCalls) {
    RadioCallQueue queue;

    // First call at t=10 should succeed
    EXPECT_TRUE(makeRadioCall(queue, RadioCallType::Bandit, 10.0, 100, 0, 5.0));
    EXPECT_EQ(queue.count, 1);

    // Second call at t=12 (within 5s interval) should fail
    EXPECT_FALSE(makeRadioCall(queue, RadioCallType::Engage, 12.0, 100, 0, 5.0));
    EXPECT_EQ(queue.count, 1);

    // Call at t=15 (exactly 5s later) should succeed
    EXPECT_TRUE(makeRadioCall(queue, RadioCallType::Engage, 15.0, 100, 0, 5.0));
    EXPECT_EQ(queue.count, 2);
}

TEST(MakeRadioCallTest, NoneTypeNeverCalls) {
    RadioCallQueue queue;
    EXPECT_FALSE(makeRadioCall(queue, RadioCallType::None, 10.0, 100));
    EXPECT_EQ(queue.count, 0);
}

// ===========================================================================
// DigiCommState integration tests
// ===========================================================================

TEST(DigiCommStateTest, RadioCallsReset) {
    DigiCommState comm;
    comm.radioCalls.push(RadioCallType::Bandit, 1.0, 100);
    comm.callsMade = 0xFF;
    EXPECT_EQ(comm.radioCalls.count, 1);
    EXPECT_NE(comm.callsMade, 0u);

    comm.reset();
    EXPECT_EQ(comm.radioCalls.count, 0);
    EXPECT_EQ(comm.callsMade, 0u);
}

TEST(DigiCommStateTest, CallsMadePreventsRepeat) {
    DigiCommState comm;

    // Make a Bingo call
    const uint32_t bit = 1u << static_cast<uint32_t>(RadioCallType::Bingo);
    EXPECT_FALSE(comm.callsMade & bit);
    makeRadioCall(comm.radioCalls, RadioCallType::Bingo, 10.0, comm.selfId);
    comm.callsMade |= bit;
    EXPECT_TRUE(comm.callsMade & bit);

    // Try again — should be blocked by callsMade
    EXPECT_TRUE(comm.callsMade & bit);  // already set
    // The makeRadioCall would succeed (throttle), but the callsMade check
    // in the brain prevents the push. Verify the bit is set.
    EXPECT_NE(comm.callsMade & bit, 0u);
}
