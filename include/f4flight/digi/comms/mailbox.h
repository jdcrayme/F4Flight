// f4flight - digi/comms/mailbox.h
//
// Mailbox — per-entity message queue.
//
// Each DigiBrain owns a Mailbox. The MessageBus delivers messages into it;
// the brain reads and processes them each frame. This decouples senders
// from receivers — a sender doesn't need to know if the recipient is even
// alive, and the recipient processes messages at its own pace.
//
// Design:
//   - Fixed-capacity ring buffer (no heap allocation per message).
//   - Messages are processed in FIFO order.
//   - The brain can peek at the next message without removing it, or pop it.
//   - When full, oldest messages are dropped (messages are ephemeral — if
//     you need guaranteed delivery, the host should track acknowledgements).

#pragma once

#include "f4flight/digi/comms/message.h"

#include <array>
#include <cstdint>
#include <optional>

namespace f4flight {
namespace digi {

class Mailbox {
public:
    static constexpr std::size_t kCapacity = 64;

    // Push a message into the queue. Returns false if the mailbox was full
    // before the push (in which case the oldest message is dropped to make
    // room and the new message is still appended). Returns true otherwise.
    bool push(const Message& msg) {
        const bool wasFull = (count_ >= kCapacity);
        if (wasFull) {
            // Drop oldest
            head_ = (head_ + 1) % kCapacity;
            --count_;
        }
        buffer_[tail_] = msg;
        tail_ = (tail_ + 1) % kCapacity;
        ++count_;
        return !wasFull;
    }

    // Peek at the next message without removing it.
    std::optional<Message> peek() const {
        if (count_ == 0) return std::nullopt;
        return buffer_[head_];
    }

    // Pop and return the next message. Returns nullopt if empty.
    std::optional<Message> pop() {
        if (count_ == 0) return std::nullopt;
        Message msg = buffer_[head_];
        head_ = (head_ + 1) % kCapacity;
        --count_;
        return msg;
    }

    // Remove all messages.
    void clear() {
        head_ = tail_ = count_ = 0;
    }

    std::size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }
    bool full() const { return count_ >= kCapacity; }

private:
    std::array<Message, kCapacity> buffer_{};
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t count_{0};
};

} // namespace digi
} // namespace f4flight
