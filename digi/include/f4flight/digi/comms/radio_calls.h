// f4flight - digi/comms/radio_calls.h
//
// Wingman radio/voice calls.
//
// Port of FreeFalcon's wingradio.cpp (867 lines). FreeFalcon's version
// creates FalconRadioChatterMessage objects and sends them over the network
// for voice playback. F4Flight's version generates RadioCall events that
// the host can display, log, or forward to a voice system.
//
// The brain triggers radio calls at key events:
//   - Mode transitions ("Engaging", "Rejoin", "RTB")
//   - Threat detections ("Bandit", "Missile", "SAM")
//   - Weapon events ("Splash", "Winchester")
//   - Fuel state ("Bingo", "Joker")
//   - Formation ("In position", "Clear six")
//
// The calls are throttled (one per 5 seconds per aircraft) to prevent spam.

#pragma once

#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/comms/message.h"  // EntityId
#include <string>
#include <cstdint>

namespace f4flight {
namespace digi {

// ===========================================================================
// RadioCallType — the type of radio call.
//
// Matches FreeFalcon's rc* constants (conv.h) for the most common calls.
// ===========================================================================
enum class RadioCallType : int {
    None            = 0,   // no call
    Bandit          = 1,   // "Bandit!" — detected an enemy aircraft
    Tally           = 2,   // "Tally bandit!" — visual on the bandit
    NoJoy           = 3,   // "No joy" — can't see the bandit
    Engage          = 4,   // "Engaging" — committing to attack
    Rejoin          = 5,   // "Rejoining" — returning to formation
    RTB             = 6,   // "RTB" — returning to base
    Bingo           = 7,   // "Bingo" — at bingo fuel
    Joker           = 8,   // "Joker" — at joker fuel
    Winchester      = 9,   // "Winchester" — out of A/A weapons
    Splash          = 10,  // "Splash" — killed a bandit
    Missile         = 11,  // "Missile!" — incoming missile detected
    SAM             = 12,  // "SAM!" — SAM detection
    InPosition      = 13,  // "In position" — reached formation slot
    ClearSix        = 14,  // "Clear six" — checked six, no bandits
    BreakLeft       = 15,  // "Break left!" — defensive break
    BreakRight      = 16,  // "Break right!" — defensive break
    Mayday          = 17,  // "Mayday" — critically damaged
    Contact         = 18,  // "Contact" — AAR contact made
    Disconnect      = 19,  // "Disconnect" — AAR disconnect
};

// Return a human-readable string for a radio call (for display/logging).
inline const char* radioCallText(RadioCallType type) {
    switch (type) {
        case RadioCallType::None:       return "";
        case RadioCallType::Bandit:     return "Bandit!";
        case RadioCallType::Tally:      return "Tally bandit!";
        case RadioCallType::NoJoy:      return "No joy";
        case RadioCallType::Engage:     return "Engaging";
        case RadioCallType::Rejoin:     return "Rejoining";
        case RadioCallType::RTB:        return "RTB";
        case RadioCallType::Bingo:      return "Bingo";
        case RadioCallType::Joker:      return "Joker";
        case RadioCallType::Winchester: return "Winchester";
        case RadioCallType::Splash:     return "Splash one";
        case RadioCallType::Missile:    return "Missile!";
        case RadioCallType::SAM:        return "SAM!";
        case RadioCallType::InPosition: return "In position";
        case RadioCallType::ClearSix:   return "Clear six";
        case RadioCallType::BreakLeft:  return "Break left!";
        case RadioCallType::BreakRight: return "Break right!";
        case RadioCallType::Mayday:     return "Mayday";
        case RadioCallType::Contact:    return "Contact";
        case RadioCallType::Disconnect: return "Disconnect";
    }
    return "";
}

// ===========================================================================
// RadioCall — a single radio call event.
//
// The brain generates these at key events. The host reads them from
// state_.comm.pendingRadioCalls (a small ring buffer) each frame and
// displays/logs/forwards them.
// ===========================================================================
struct RadioCall {
    RadioCallType type{RadioCallType::None};
    double        time{0.0};         // sim time of the call
    EntityId      senderId{0};       // who made the call
    EntityId      targetId{0};       // who the call is about (0 = N/A)

    bool isValid() const { return type != RadioCallType::None; }
    void clear() { type = RadioCallType::None; time = 0.0; targetId = 0; }
};

// ===========================================================================
// RadioCallQueue — a small ring buffer for pending radio calls.
//
// The brain pushes calls here; the host drains them each frame. The buffer
// is small (4 calls) to prevent spam — if the brain generates calls faster
// than the host drains them, the oldest calls are dropped.
// ===========================================================================
struct RadioCallQueue {
    static constexpr int kCapacity = 4;

    RadioCall calls[kCapacity];
    int       head{0};  // next slot to write
    int       tail{0};  // next slot to read
    int       count{0}; // number of valid calls
    double    lastCallTime{0.0};  // for throttling

    void reset() noexcept {
        for (int i = 0; i < kCapacity; ++i) calls[i].clear();
        head = tail = count = 0;
        lastCallTime = 0.0;
    }

    // Push a radio call. Returns true if the call was queued, false if it
    // was dropped (buffer full). The caller is responsible for throttling.
    bool push(RadioCallType type, double simTime, EntityId senderId,
              EntityId targetId = 0) {
        if (count >= kCapacity) return false;  // full — drop
        calls[head] = RadioCall{type, simTime, senderId, targetId};
        head = (head + 1) % kCapacity;
        ++count;
        lastCallTime = simTime;
        return true;
    }

    // Pop the oldest radio call. Returns true if a call was popped.
    bool pop(RadioCall& out) {
        if (count == 0) return false;
        out = calls[tail];
        calls[tail].clear();
        tail = (tail + 1) % kCapacity;
        --count;
        return true;
    }

    // Check if a call can be made (throttled to one per interval).
    bool canCall(double simTime, double intervalSec = 5.0) const {
        return simTime - lastCallTime >= intervalSec;
    }
};

// ===========================================================================
// makeRadioCall — helper to make a throttled radio call.
//
// Checks the throttle interval and pushes the call if enough time has
// passed since the last call. Returns true if the call was made.
// ===========================================================================
inline bool makeRadioCall(RadioCallQueue& queue, RadioCallType type,
                           double simTime, EntityId senderId,
                           EntityId targetId = 0,
                           double intervalSec = 5.0) {
    if (type == RadioCallType::None) return false;
    if (!queue.canCall(simTime, intervalSec)) return false;
    return queue.push(type, simTime, senderId, targetId);
}

} // namespace digi
} // namespace f4flight
