#pragma once
#include "yz_aim.h"
#include <array>
#include <deque>

namespace yz_aim {
// A single USB handle/receive queue, with independent logical queues per motor.
// All calls stay on the CLI CAN worker; only diagnostic printing has a thread.
class SharedCanBus {
public:
    class Port final : public Transport {
    public:
        Port(SharedCanBus& bus, uint8_t node) : bus_(bus), node_(node) {}
        bool send(const Frame& frame) override { return !failed() && bus_.physical_.send(frame); }
        bool receive(Frame& frame, Deadline deadline) override;
        bool failed() const override { return bus_.overflow_ || bus_.physical_.failed(); }
        bool accept_sdo_replies(uint8_t primary, uint8_t alternate) override;
    private:
        friend class SharedCanBus;
        SharedCanBus& bus_;
        uint8_t node_, alternate_ = 0;
        std::deque<Frame> sdo_;
        std::deque<Frame> telemetry_;
    };
    SharedCanBus(Transport& physical, uint8_t first, uint8_t second)
        : physical_(physical), first_(*this, first), second_(*this, second) {}
    Port& first() { return first_; }
    Port& second() { return second_; }
    uint64_t telemetry_coalesced() const { return telemetry_coalesced_; }
private:
    bool route(const Frame& frame);
    Transport& physical_;
    Port first_, second_;
    bool overflow_ = false;
    uint64_t telemetry_coalesced_ = 0;
};
} // namespace yz_aim
