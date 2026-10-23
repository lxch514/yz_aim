#include "yz_aim.h"
#include "yz_aim_mock.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <deque>
#include <cstdio>
#include <cstdlib>

namespace {
int checks = 0;
void require(bool value, const char* message) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
yz_aim::Deadline deadline() { return yz_aim::Clock::now() + std::chrono::milliseconds(30); }
yz_aim::Frame reply(uint8_t command, uint16_t index, uint8_t subindex, uint32_t data = 0) {
    yz_aim::Frame frame{0x581, 8, {command, static_cast<uint8_t>(index),
        static_cast<uint8_t>(index >> 8), subindex}};
    for (unsigned i = 0; i < 4; ++i) frame.data[i + 4] = static_cast<uint8_t>(data >> (8 * i));
    return frame;
}
}
int main() {
    require(yz_aim::kMotorRevolutionsPerOutputRevolution == 51 &&
            yz_aim::kOutputPulsesPerRevolution == 1671168,
            "user-configured 51:1 output ratio and 32768 motor pulses per revolution");
    require(yz_aim::kMaxPositionPulses == 1966080000 &&
            yz_aim::kMinPositionPulses == -1966080000 &&
            yz_aim::kMaxRelativePositionPulses == std::numeric_limits<int32_t>::max() &&
            yz_aim::kMaxMotorRpm == 3000 && yz_aim::kPositionProfileRpm == 30,
            "manual motor turn range and speed maximum; relative target remains signed32");
    require(yz_aim::position_action_budget(23211) == std::chrono::milliseconds(6537) &&
            yz_aim::position_action_budget(-417792) == std::chrono::milliseconds(30620) &&
            yz_aim::position_action_budget(417792, 1) == std::chrono::milliseconds(770004) &&
            yz_aim::position_action_budget(417792, 6) == std::chrono::milliseconds(132524),
            "travel plus ramp-up, ramp-down and existing setup margin");
    require(yz_aim::position_action_budget(3932160000LL, 3000) ==
                std::chrono::milliseconds(2417000) &&
            yz_aim::position_action_budget(-3932160000LL, 1) ==
                std::chrono::milliseconds(7200005004LL) &&
            yz_aim::position_action_budget(std::numeric_limits<int64_t>::min(), 3000) ==
                std::chrono::milliseconds(2417000) &&
            yz_aim::position_action_budget(std::numeric_limits<int64_t>::max(), 3000) ==
                std::chrono::milliseconds(2417000),
            "complete endpoint traverse and extreme arguments stay finite without signed overflow");
    require(yz_aim::kStopActionBudget == std::chrono::seconds(10) &&
            yz_aim::jog_action_budget(std::chrono::seconds(5)) == std::chrono::seconds(20),
            "bounded jog includes independent confirmed-stop window");
    require(yz_aim::motor_to_output_rpm(51.0) == 1.0 &&
            yz_aim::motor_to_output_rpm(-51.0) == -1.0,
            "positive and negative motor RPM conversion");
    require(yz_aim::pulses_to_output_degrees(1671168) == 360.0 &&
            yz_aim::pulses_to_output_degrees(-1671168) == -360.0,
            "positive and negative output angle conversion");
    int32_t pulses = 123;
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(1.0, pulses)) &&
            pulses == 1671168, "one output revolution is 51 motor revolutions");
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(-1.0, pulses)) &&
            pulses == -1671168, "negative output revolution conversion");
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(5.0 / 360.0, pulses)) &&
            pulses == 23211, "default 5 output degrees rounds to 23211 motor pulses");
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(15.0 / 360.0, pulses)) &&
            pulses == 69632, "15 output degrees is 69632 motor pulses");
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(30.0 / 360.0, pulses)) &&
            pulses == 139264, "30 output degrees is 139264 motor pulses");
    require(static_cast<bool>(yz_aim::output_revolutions_to_pulses(90.0 / 360.0, pulses)) &&
            pulses == 417792, "90 output degrees converts without a per-move cap");
    const int32_t previous = pulses;
    require(yz_aim::output_revolutions_to_pulses(std::numeric_limits<double>::infinity(), pulses).error ==
            yz_aim::Error::invalid_argument && pulses == previous,
            "nonfinite conversion cannot change destination");
    require(yz_aim::output_revolutions_to_pulses(10000.0, pulses).error ==
            yz_aim::Error::invalid_argument && pulses == previous,
            "large output position rejected without signed32 truncation");
    require(yz_aim::output_revolutions_to_pulses(-10000.0, pulses).error ==
            yz_aim::Error::invalid_argument && pulses == previous,
            "negative output position overflow rejected");
    yz_aim::MockTransport wire;
    yz_aim::SdoClient sdo(wire, 1);
    std::deque<yz_aim::Frame> planned;
    wire.on_send = [&](const yz_aim::Frame&, yz_aim::MockTransport& target) {
        while (!planned.empty()) { target.replies.push_back(planned.front()); planned.pop_front(); }
    };
    uint32_t value = 999;
    planned.push_back(reply(0x4b, 0x6040, 0)); // unrelated object must not satisfy 606c
    planned.push_back(reply(0x4b, 0x606c, 0, 0xff9c)); // signed -100 /10 = -10RPM
    require(static_cast<bool>(sdo.read(0x606c, 0, 2, value, deadline())), "correlated upload");
    require(static_cast<int16_t>(value) == -100, "signed speed bits");
    require(wire.sent.back().id == 0x601 && wire.sent.back().dlc == 8 && wire.sent.back().data[0] == 0x40,
            "classic SDO upload encoding");
    planned.push_back(reply(0x60, 0x6040, 0));
    planned.push_back(reply(0x60, 0x2615, 0));
    require(static_cast<bool>(sdo.write(0x2615, 0, 2, 0x1234, deadline())), "ignore copied 6040 ACK");
    require(wire.sent.back().data[0] == 0x2b && wire.sent.back().data[1] == 0x15 &&
            wire.sent.back().data[2] == 0x26 && wire.sent.back().data[4] == 0x34 &&
            wire.sent.back().data[5] == 0x12, "write16 little endian");
    planned.push_back(reply(0x60, 0x60ff, 0));
    require(static_cast<bool>(sdo.write(0x60ff, 0, 4, static_cast<uint32_t>(-30), deadline())), "write signed32");
    require(wire.sent.back().data[0] == 0x23 && wire.sent.back().data[4] == 0xe2 &&
            wire.sent.back().data[7] == 0xff, "signed32 encoding");
    planned.push_back(reply(0x4b, 0x606c, 0, 0xffffffff));
    require(static_cast<bool>(sdo.read(0x606c, 0, 2, value, deadline())) &&
            static_cast<int16_t>(value) == -1 && value == 0xffff,
            "real 0x606C signed16 -0.1 RPM has FF unused bytes; decode only two declared bytes");
    planned.push_back(reply(0x4f, 0x6060, 0, 3));
    require(static_cast<bool>(sdo.read(0x6060, 0, 1, value, deadline())) && value == 3, "read8");
    yz_aim::Frame malformed = reply(0x4f, 0x606c, 0, 8);
    planned.push_back(malformed);
    require(sdo.read(0x606c, 0, 2, value, deadline()).error == yz_aim::Error::malformed_reply,
            "reject mismatched width");
    malformed = reply(0x60, 0x2615, 0); malformed.dlc = 5;
    yz_aim::SdoClient recovered(wire, 1); // synthetic new transport session after malformed reply
    planned.push_back(malformed);
    require(recovered.write(0x2615, 0, 2, 5, deadline()).error == yz_aim::Error::malformed_reply,
            "reject truncated ACK");
    yz_aim::SdoClient aborted(wire, 1);
    planned.push_back(reply(0x80, 0x2615, 0, 0x06090011));
    auto result = aborted.write(0x2615, 0, 2, 5, deadline());
    require(result.error == yz_aim::Error::abort_reply && result.abort_code == 0x06090011 &&
            result.message == "SDO abort" &&
            result.context.find("SDO write 0x2615:00 phase=reply") != std::string::npos &&
            result.context.find("elapsed_ms=") != std::string::npos &&
            result.context.find("budget_ms=") != std::string::npos,
            "preserve abort code and original message with structured SDO failure context");
    const auto timed_out = aborted.read(0x6079, 0, 2, value, deadline());
    require(timed_out.error == yz_aim::Error::timeout &&
            timed_out.context.find("SDO read 0x6079:00 phase=reply") != std::string::npos,
            "bounded reply timeout reports object and phase");
    const size_t sent_before = wire.sent.size();
    planned.push_back(reply(0x4b, 0x6079, 0, 88)); // late ACK must not revive a poisoned SDO session
    require(aborted.read(0x6079, 0, 2, value, deadline()).error == yz_aim::Error::unsafe &&
            wire.sent.size() == sent_before, "reject late reply without sending fresh read");
    require(aborted.write(0x607a, 0, 2, 65536, deadline()).error == yz_aim::Error::invalid_argument,
            "reject data truncation");
    yz_aim::MockTransport backlogged;
    yz_aim::SdoClient backlog_sdo(backlogged, 1);
    // PDOs accumulate while the operator spends time in a modal confirmation.
    // An old same-object SDO reply may be hidden behind them in the adapter FIFO.
    for (unsigned i = 0; i < 750; ++i) {
        backlogged.replies.push_back({static_cast<uint16_t>(i & 1 ? 0x181 : 0x381), 6, {}});
        if (i == 73) backlogged.replies.push_back(reply(0x4b, 0x606c, 0, 0x7777));
    }
    backlogged.on_send = [](const yz_aim::Frame&, yz_aim::MockTransport& target) {
        target.replies.push_back(reply(0x4b, 0x606c, 0, 0xff9c));
    };
    require(static_cast<bool>(backlog_sdo.read(0x606c, 0, 2, value,
                yz_aim::Clock::now() + std::chrono::milliseconds(250))) &&
            value == 0xff9c && backlogged.replies.empty() && backlogged.sent.size() == 1,
            "drain >30 seconds of PDOs and buried stale same-object reply before new request");
    yz_aim::MockTransport flood;
    yz_aim::SdoClient flood_sdo(flood, 1);
    for (unsigned i = 0; i < 4100; ++i) flood.replies.push_back({0x181, 6, {}});
    const auto flood_result = flood_sdo.read(0x606c, 0, 2, value,
                yz_aim::Clock::now() + std::chrono::milliseconds(250));
    require(flood_result.error == yz_aim::Error::unsafe &&
            flood_result.context.find("SDO read 0x606c:00 phase=prequeue") != std::string::npos &&
            flood.sent.empty() && !flood.replies.empty(),
            "unbounded RX flood rejects before any SDO TX");
    require(flood_sdo.read(0x606c, 0, 2, value, deadline()).error == yz_aim::Error::unsafe &&
            flood.sent.empty(), "flooded prequeue poisons session until reconnect");
    std::printf("yz_aim_protocol_test: %d checks passed (synthetic, no device)\n", checks);
}
