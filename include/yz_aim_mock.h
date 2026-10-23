#pragma once
#include "yz_aim.h"
#include <deque>
#include <functional>
#include <vector>

namespace yz_aim {
// Synthetic bench transport. Never communicates with real hardware.
class MockTransport : public Transport {
public:
    std::vector<Frame> sent;
    std::deque<Frame> replies;
    std::function<void(const Frame&, MockTransport&)> on_send;
    bool send(const Frame& frame) override {
        sent.push_back(frame);
        if (on_send) on_send(frame, *this);
        return true;
    }
    bool receive(Frame& frame, Deadline) override {
        if (replies.empty()) return false;
        frame = replies.front();
        replies.pop_front();
        return true;
    }
};
} // namespace yz_aim
