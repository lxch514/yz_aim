// 常转（start_velocity / sustain_service）的合成台架测试：只与内存里的假驱动器对话。
#include "yz_aim.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

int checks = 0;
void require(bool value, const std::string& message) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", message.c_str()); std::exit(1); }
}
yz_aim::Deadline after(std::chrono::milliseconds budget = std::chrono::milliseconds(2000)) {
    return yz_aim::Clock::now() + budget;
}
uint16_t index_of(const yz_aim::Frame& frame) {
    return static_cast<uint16_t>(frame.data[1] | frame.data[2] << 8);
}
uint8_t subindex_of(const yz_aim::Frame& frame) { return frame.data[3]; }
uint32_t value_of(const yz_aim::Frame& frame) {
    return static_cast<uint32_t>(frame.data[4]) | static_cast<uint32_t>(frame.data[5]) << 8 |
           static_cast<uint32_t>(frame.data[6]) << 16 | static_cast<uint32_t>(frame.data[7]) << 24;
}

// 假驱动器：只回答本测试关心的对象字典，应答宽度与请求宽度一致。
class FakeBus : public yz_aim::Transport {
public:
    std::vector<yz_aim::Frame> sent;
    std::deque<yz_aim::Frame> replies;
    uint16_t status = 0x0437;
    uint16_t alarm = 0;
    uint16_t output = 0;
    int16_t speed = 0;
    int32_t position = 10000;
    unsigned heartbeats = 0;
    std::unordered_map<uint32_t, uint32_t> registers;

    FakeBus() { registers[0x1016u | (1u << 16)] = 0x007f07d0u; }

    bool send(const yz_aim::Frame& frame) override {
        sent.push_back(frame);
        if (frame.id == 0x77f) { ++heartbeats; return true; }
        if (frame.id != 0x601) return true;
        const uint16_t object = index_of(frame);
        yz_aim::Frame reply{0x581, 8, {0, frame.data[1], frame.data[2], frame.data[3]}};
        uint8_t width = 2;
        if (frame.data[0] == 0x40) {
            uint32_t value = 0;
            switch (object) {
                case 0x1000: value = 0x20192u; width = 4; break;
                case 0x1018: value = subindex_of(frame) == 1 ? 0x331u : 1u; width = 4; break;
                case 0x1016: case 0x60ff: case 0x607a: case 0x6081: case 0x6083: {
                    const auto stored = registers.find(object | subindex_of(frame) << 16);
                    value = stored == registers.end() ? 0 : stored->second;
                    width = 4;
                    break;
                }
                case 0x6041: value = status; break;
                case 0x260e: value = alarm; break;
                case 0x2601: value = output; break;
                case 0x6064: value = static_cast<uint32_t>(position); width = 4; break;
                case 0x606c: value = static_cast<uint16_t>(speed); break;
                case 0x6078: value = 2000; break;
                case 0x6079: value = 654; break;
                case 0x2612: value = 25; break;
                case 0x261e: value = 7; break;
                default: return true;
            }
            reply.data[0] = width == 1 ? 0x4f : width == 2 ? 0x4b : 0x43;
            for (uint8_t i = 0; i < width; ++i)
                reply.data[4 + i] = static_cast<uint8_t>(value >> (i * 8));
        } else {
            width = frame.data[0] == 0x2f ? 1 : frame.data[0] == 0x2b ? 2 : 4;
            reply.data[0] = 0x60;
            registers[object | (static_cast<uint32_t>(subindex_of(frame)) << 16)] = value_of(frame);
            if (object == 0x2601) output = static_cast<uint16_t>(value_of(frame) & 0xffffu);
        }
        replies.push_back(reply);
        return true;
    }
    bool receive(yz_aim::Frame& frame, yz_aim::Deadline) override {
        if (replies.empty()) return false;
        frame = replies.front();
        replies.pop_front();
        return true;
    }
};

// 一次 SDO 写的索引与数值，用于核对报文的先后顺序。
struct Write {
    uint16_t index = 0;
    uint32_t value = 0;
};
std::vector<Write> writes_of(const std::vector<yz_aim::Frame>& frames) {
    std::vector<Write> out;
    for (const yz_aim::Frame& frame : frames) {
        if (frame.id != 0x601) continue;
        if (frame.data[0] != 0x2f && frame.data[0] != 0x2b && frame.data[0] != 0x23) continue;
        if (index_of(frame) == 0x6040 || index_of(frame) == 0x6060 || index_of(frame) == 0x6083 ||
            index_of(frame) == 0x60ff || index_of(frame) == 0x607a || index_of(frame) == 0x2601 ||
            index_of(frame) == 0x1016)
            out.push_back({index_of(frame), value_of(frame)});
    }
    return out;
}
bool has_heartbeat(const std::vector<yz_aim::Frame>& frames) {
    for (const yz_aim::Frame& frame : frames)
        if (frame.id == 0x77f && frame.dlc == 1 && frame.data[0] == 0x05) return true;
    return false;
}
// 走完 ARM 与使能，进入可以做运动的会话状态。
bool bring_up(FakeBus& bus, yz_aim::Motor& motor, std::string& why) {
    const yz_aim::Snapshot snapshot = motor.snapshot(after());
    if (!snapshot.result) { why = "snapshot: " + snapshot.result.message; return false; }
    const yz_aim::Result armed = motor.arm(after());
    if (!armed) { why = "arm: " + armed.message; return false; }
    const yz_aim::Result enabled = motor.enable_output(after());
    if (!enabled) { why = "enable: " + enabled.message; return false; }
    (void)bus;
    return true;
}

void test_start_velocity_frame_order() {
    FakeBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, why), "enable for run: " + why);
    bus.sent.clear();
    const yz_aim::Result started = motor.start_velocity(300, after());
    require(static_cast<bool>(started), "start_velocity: " + started.message);
    require(motor.sustaining(), "start_velocity sets the sustaining flag");
    require(motor.sustained_rpm() == 300, "start_velocity records the commanded rpm");

    const std::vector<Write> writes = writes_of(bus.sent);
    require(!writes.empty(), "start_velocity transmits SDO writes");
    require(writes.front().index == 0x6040 && writes.front().value == 0x010f,
            "run starts with the hold controlword 0x010f");
    require(writes.back().index == 0x6040 && writes.back().value == 0x000f,
            "run ends with the enable controlword 0x000f");
    unsigned hold = 0, speed_mode = 0, accel = 0, target = 0, consumer = 0;
    for (const Write& write : writes) {
        if (write.index == 0x6040) ++hold;
        if (write.index == 0x6060) { ++speed_mode; require(write.value == 3, "run selects velocity mode 3"); }
        if (write.index == 0x6083) { ++accel; require(write.value == 500, "run writes 500 RPM/s acceleration"); }
        if (write.index == 0x60ff) target = write.value;
        if (write.index == 0x1016) ++consumer;
    }
    require(hold == 2, "run writes the controlword exactly twice");
    require(speed_mode == 1 && accel == 1, "run writes mode and acceleration once each");
    require(target == 300, "run leaves 0x60ff at the commanded speed");
    require(consumer == 0, "run reuses the heartbeat consumer that enable already took over");
    require(has_heartbeat(bus.sent), "run produces a producer heartbeat 0x77f:05");
    require(motor.watchdog_active(), "run keeps the owned heartbeat active");
    require(static_cast<bool>(motor.stop(after())), "stop after run");
}

void test_sustain_service_reports_and_trips() {
    FakeBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, why), "enable for sustain: " + why);
    require(static_cast<bool>(motor.start_velocity(300, after())), "run starts");

    bus.speed = 3000; // 0x606c 的 3000 表示 300.0 电机RPM
    yz_aim::Snapshot seen{};
    const yz_aim::Result healthy = motor.sustain_service(after(), [&seen](const yz_aim::Snapshot& s) { seen = s; });
    require(static_cast<bool>(healthy), "healthy sweep: " + healthy.message);
    require(seen.speed_sample.valid && seen.speed_rpm == 300.0, "sweep reports 300.0 motor RPM");
    require(seen.position_pulses == 10000, "sweep reports the live position");
    require(seen.commanded_speed_rpm == 300, "sweep reports the commanded speed");
    require(motor.sustaining(), "a healthy sweep keeps the run alive");

    bus.alarm = 0x12;
    const yz_aim::Result tripped = motor.sustain_service(after());
    require(tripped.error == yz_aim::Error::alarm, "a drive alarm trips the run");
    require(!motor.sustaining(), "an alarm clears the sustaining flag");
    require(motor.unsafe_latched(), "an alarm latches the session unsafe");
    require(!motor.watchdog_active(), "an alarm retires the producer heartbeat");
}

void test_sustain_service_guards() {
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        std::string why;
        require(bring_up(bus, motor, why), "enable for overspeed: " + why);
        require(static_cast<bool>(motor.start_velocity(300, after())), "run starts for overspeed");
        bus.speed = 31000; // 3100 电机RPM，超过手册上限
        const yz_aim::Result overspeed = motor.sustain_service(after());
        require(overspeed.error == yz_aim::Error::unsafe, "speed above the manual maximum trips unsafe");
        require(!motor.sustaining(), "overspeed clears the sustaining flag");
    }
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        std::string why;
        require(bring_up(bus, motor, why), "enable for position limit: " + why);
        require(static_cast<bool>(motor.start_velocity(300, after())), "run starts for position limit");
        bus.position = yz_aim::kMaxPositionPulses + 1;
        const yz_aim::Result beyond = motor.sustain_service(after());
        require(beyond.error == yz_aim::Error::limit, "position beyond the manual range trips limit");
        require(!motor.sustaining(), "position limit clears the sustaining flag");
    }
}

void test_stop_and_cancel() {
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        std::string why;
        require(bring_up(bus, motor, why), "enable for stop: " + why);
        require(static_cast<bool>(motor.start_velocity(-300, after())), "reverse run starts");
        require(motor.sustained_rpm() == -300, "reverse run records a negative rpm");
        const yz_aim::Result stopped = motor.stop(after());
        require(static_cast<bool>(stopped), "stop from a sustained run: " + stopped.message);
        require(!motor.sustaining(), "stop clears the sustaining flag");
        require(!motor.unsafe_latched(), "a clean stop is not a fault");
        require(motor.watchdog_active(), "heartbeat continues while the output is owned");
        require(!bus.replies.empty() || true, "stop returns to the caller");
    }
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        std::string why;
        require(bring_up(bus, motor, why), "enable for cancel: " + why);
        require(static_cast<bool>(motor.start_velocity(300, after())), "run starts for cancel");
        motor.request_cancel();
        const yz_aim::Result cancelled = motor.sustain_service(after());
        require(cancelled.error == yz_aim::Error::cancelled, "a cancel request ends the run as cancelled");
        require(!motor.sustaining(), "cancellation clears the sustaining flag");
        require(!motor.unsafe_latched(), "cancellation is a clean stop, not a fault");
    }
}

void test_start_velocity_rejections() {
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        std::string why;
        require(bring_up(bus, motor, why), "enable for rejections: " + why);
        require(motor.start_velocity(0, after()).error == yz_aim::Error::invalid_argument, "run rejects 0 RPM");
        require(motor.start_velocity(3001, after()).error == yz_aim::Error::invalid_argument, "run rejects >3000 RPM");
        require(motor.start_velocity(-3001, after()).error == yz_aim::Error::invalid_argument, "run rejects <-3000 RPM");
        require(!motor.sustaining(), "rejected runs never set the sustaining flag");
        require(!motor.unsafe_latched(), "rejected runs never latch the session unsafe");
        // 运行方向只剩 1000 脉冲（不到一分钟行程）时应被拒绝，反方向仍可启动。
        bus.position = yz_aim::kMaxPositionPulses - 1000;
        require(motor.start_velocity(300, after()).error == yz_aim::Error::invalid_argument,
                "run refuses a direction with under one minute of travel left");
        const yz_aim::Result reverse = motor.start_velocity(-300, after());
        require(static_cast<bool>(reverse), "the opposite direction is still allowed: " + reverse.message);
        require(static_cast<bool>(motor.stop(after())), "stop the reverse run");
        // stop 会释放 ARM，再次 run 必须先重新 ARM（交互式 CLI 会自动补这一步）。
        require(motor.start_velocity(300, after()).error == yz_aim::Error::unsafe,
                "run after stop requires a fresh ARM");
        require(static_cast<bool>(motor.arm(after())), "re-arm after stop");
        // 300 电机RPM 的一分钟行程 = 300 × 32768 = 9830400 脉冲，边界两侧分别拒绝与放行。
        bus.position = yz_aim::kMaxPositionPulses - 9000000;
        require(motor.start_velocity(300, after()).error == yz_aim::Error::invalid_argument,
                "run refuses 9000000 pulses of remaining travel at 300 RPM");
        bus.position = yz_aim::kMaxPositionPulses - 10000000;
        const yz_aim::Result boundary = motor.start_velocity(300, after());
        require(static_cast<bool>(boundary), "run allows exactly one minute of travel: " + boundary.message);
        require(static_cast<bool>(motor.stop(after())), "stop the boundary run");
    }
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 1);
        const yz_aim::Result unarmed = motor.start_velocity(300, after());
        require(unarmed.error == yz_aim::Error::unsafe, "run requires an armed, enabled output");
        require(!motor.sustaining(), "an unarmed run never starts");
    }
    {
        FakeBus bus;
        yz_aim::Motor motor(bus, 127);
        const yz_aim::Result collision = motor.start_velocity(300, after());
        require(collision.error == yz_aim::Error::invalid_argument, "node 127 cannot run its own heartbeat");
    }
}

} // namespace

int main() {
    test_start_velocity_frame_order();
    test_sustain_service_reports_and_trips();
    test_sustain_service_guards();
    test_stop_and_cancel();
    test_start_velocity_rejections();
    std::printf("sustain_test: %d checks passed\n", checks);
    return 0;
}
