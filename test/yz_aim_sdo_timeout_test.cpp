#include "yz_aim.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <thread>

using namespace yz_aim;
using namespace std::chrono;

namespace {
int checks = 0;
void require(bool value, const char* text) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", text); std::exit(1); }
}

class DelayedTransport final : public Transport {
public:
    milliseconds reply_delay{1};
    unsigned backlog = 0, sends = 0;
    bool pending = false;
    Frame request;
    Clock::time_point sent_at;
    double first_reply_budget_ms = -1;

    bool send(const Frame& frame) override {
        ++sends;
        request = frame;
        pending = true;
        sent_at = Clock::now();
        return true;
    }
    bool receive(Frame& frame, Deadline until) override {
        if (!pending) {
            if (backlog) {
                --backlog;
                std::this_thread::sleep_until(std::min(until, Clock::now() + microseconds(700)));
                frame = {0x181, 6, {}};
                return true;
            }
            return false;
        }
        if (first_reply_budget_ms < 0)
            first_reply_budget_ms = duration<double, std::milli>(until - sent_at).count();
        const auto ready = sent_at + reply_delay;
        if (ready > until) {
            std::this_thread::sleep_until(until);
            return false;
        }
        std::this_thread::sleep_until(ready);
        pending = false;
        frame = {0x581, 8, {static_cast<uint8_t>(request.data[0] == 0x40 ? 0x4b : 0x60),
                           request.data[1], request.data[2], request.data[3]}};
        if (request.data[0] == 0x40) frame.data[4] = 123;
        return true;
    }
};

Deadline budget(unsigned ms = 250) { return Clock::now() + milliseconds(ms); }
}

int main() {
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(40);
        SdoClient client(wire, 1, {milliseconds(20), milliseconds(100)});
        uint32_t value = 0;
        auto result = client.read(0x606c, 0, 2, value, budget());
        require(result.error == Error::timeout, "40ms delayed read rejected by 20ms cap");
        require(result.context.find("reply_timeout_ms=20") != std::string::npos,
                "failure records selected read limit");
        require(wire.first_reply_budget_ms > 10 && wire.first_reply_budget_ms < 25,
                "read transport receives an approximately 20ms reply deadline");
        require(wire.sends == 1, "timed-out read sends exactly one request");
        require(client.read(0x606c, 0, 2, value, budget()).error == Error::unsafe && wire.sends == 1,
                "late reply does not revive poisoned session or replay request");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(60);
        SdoClient client(wire, 1, {milliseconds(20), milliseconds(100)});
        require(static_cast<bool>(client.write(0x6040, 0, 2, 0x100, budget())),
                "60ms delayed write accepted inside 100ms write cap");
        require(wire.first_reply_budget_ms > 85 && wire.first_reply_budget_ms < 105,
                "write receives 100ms cap, not read's 20ms cap");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(130);
        SdoClient client(wire, 1, {milliseconds(20), milliseconds(100)});
        auto result = client.write(0x6040, 0, 2, 0x100, budget());
        require(result.error == Error::timeout && wire.sends == 1,
                "130ms delayed write times out without write retry");
        require(result.context.find("reply_timeout_ms=100") != std::string::npos,
                "failure records selected write limit");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(15);
        wire.backlog = 25;
        SdoClient client(wire, 1, {milliseconds(20), milliseconds(100)});
        uint32_t value = 0;
        require(static_cast<bool>(client.read(0x606c, 0, 2, value, budget())) && value == 123,
                "prequeue drain is separate from 20ms post-send reply window");
        require(wire.backlog == 0 && wire.sends == 1, "drains unrelated PDOs before exactly one read");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(35);
        SdoClient legacy(wire, 1);
        uint32_t value = 0;
        require(static_cast<bool>(legacy.read(0x606c, 0, 2, value, budget(90))),
                "default zero limits preserve legacy caller budget");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(25);
        SdoClient client(wire, 1, {milliseconds(20), milliseconds(100)});
        uint32_t value = 0;
        require(client.read(0x606c, 0, 2, value, budget(8)).error == Error::timeout,
                "read cap never extends a shorter caller deadline");
        require(wire.first_reply_budget_ms < 12, "short caller budget reaches transport");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(40);
        SdoClient client(wire, 1, {milliseconds(100), milliseconds(20)});
        uint32_t value = 0;
        require(static_cast<bool>(client.read(0x606c, 0, 2, value, budget())),
                "read and write values can be independently reversed");
        require(client.write(0x6040, 0, 2, 0x100, budget()).error == Error::timeout,
                "short write window independent of longer read window");
    }
    {
        DelayedTransport wire;
        bool rejected = false;
        try { SdoClient invalid(wire, 1, {milliseconds(-1), milliseconds(100)}); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected && wire.sends == 0, "invalid negative timeout rejected without transmission");
    }
    {
        DelayedTransport wire;
        wire.reply_delay = milliseconds(40);
        Motor motor(wire, 1, {milliseconds(20), milliseconds(100)});
        auto snapshot = motor.snapshot(budget());
        require(snapshot.result.error == Error::timeout && wire.sends == 1,
                "Motor passes configured read timeout to its own SDO client");
        require(snapshot.result.context.find("reply_timeout_ms=20") != std::string::npos,
                "Motor preserves timeout diagnostic context");
    }
    std::printf("yz_aim_sdo_timeout_test: %d checks passed (simulated, no CAN/USB)\n", checks);
}
