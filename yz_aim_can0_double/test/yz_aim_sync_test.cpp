// 双轴同步原语（stage/trigger/abort/halt/poll_safety）的合成台架测试：只与内存里的假驱动器对话。
#include "yz_aim.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

int checks = 0;
void require(bool value, const std::string& message) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", message.c_str()); std::exit(1); }
}
yz_aim::Deadline after(std::chrono::milliseconds budget = std::chrono::milliseconds(3000)) {
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

// 一条总线上的假驱动器：节点 1 与节点 2 各自独立应答，便于验证两轴共线时的报文顺序。
class DualBus : public yz_aim::Transport {
public:
    struct Node {
        uint16_t status = 0x0437;
        uint16_t alarm = 0;
        uint16_t output = 0;
        int16_t speed = 0;
        int32_t position = 0;
        std::unordered_map<uint32_t, uint32_t> registers;
        Node() { registers[0x1016u | (1u << 16)] = 0x007f07d0u; }
    };
    std::vector<yz_aim::Frame> sent;
    std::deque<yz_aim::Frame> replies;
    Node node[3]; // 下标即节点号，只用 1 与 2
    unsigned drop_reads = 0;  // 前 N 个读应答直接丢弃，模拟真实总线丢帧
    unsigned drop_writes = 0; // 前 N 个写应答直接丢弃

    bool send(const yz_aim::Frame& frame) override {
        sent.push_back(frame);
        if (frame.id == 0x77f) return true;
        if (frame.id != 0x601 && frame.id != 0x602) return true;
        const unsigned id = frame.id - 0x600;
        Node& state = node[id];
        const uint16_t object = index_of(frame);
        yz_aim::Frame reply;
        reply.id = static_cast<uint16_t>(0x580 + id);
        reply.dlc = 8;
        reply.data[1] = frame.data[1];
        reply.data[2] = frame.data[2];
        reply.data[3] = frame.data[3];
        uint8_t width = 2;
        if (frame.data[0] == 0x40) {
            uint32_t value = 0;
            switch (object) {
                case 0x1000: value = 0x20192u; width = 4; break;
                case 0x1018: value = subindex_of(frame) == 1 ? 0x331u : 1u; width = 4; break;
                case 0x1016: case 0x60ff: case 0x607a: case 0x6081: case 0x6083: {
                    const auto stored = state.registers.find(object | subindex_of(frame) << 16);
                    value = stored == state.registers.end() ? 0 : stored->second;
                    width = 4;
                    break;
                }
                case 0x6041: value = state.status; break;
                case 0x260e: value = state.alarm; break;
                case 0x2601: value = state.output; break;
                case 0x6064: value = static_cast<uint32_t>(state.position); width = 4; break;
                case 0x6060: value = state.registers[0x6060]; width = 1; break;
                case 0x606c: value = static_cast<uint16_t>(state.speed); break;
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
            reply.data[0] = 0x60;
            state.registers[object | (static_cast<uint32_t>(subindex_of(frame)) << 16)] = value_of(frame);
            if (object == 0x2601) state.output = static_cast<uint16_t>(value_of(frame) & 0xffffu);
        }
        // 按需丢弃应答：读丢帧必须靠重发恢复，写丢帧必须如实失败且不重发。
        if (frame.data[0] == 0x40) {
            if (drop_reads) { --drop_reads; return true; }
        } else if (drop_writes) {
            --drop_writes;
            return true;
        }
        replies.push_back(reply);
        return true;
    }
    // 真机 socket 在队列为空时会一直阻塞到截止时间，而不是立刻返回"没有帧"；
    // 只有打开这个开关，才能暴露"重试沿用一个已经耗尽的截止时间"这类问题。
    bool block_when_empty = false;
    bool receive(yz_aim::Frame& frame, yz_aim::Deadline deadline) override {
        if (replies.empty()) {
            if (block_when_empty) std::this_thread::sleep_until(deadline);
            return false;
        }
        frame = replies.front();
        replies.pop_front();
        return true;
    }
};

struct Write {
    unsigned node = 0;
    uint16_t index = 0;
    uint32_t value = 0;
};
std::vector<Write> writes_of(const std::vector<yz_aim::Frame>& frames) {
    std::vector<Write> out;
    for (const yz_aim::Frame& frame : frames) {
        if (frame.id != 0x601 && frame.id != 0x602) continue;
        if (frame.data[0] != 0x2f && frame.data[0] != 0x2b && frame.data[0] != 0x23) continue;
        const uint16_t object = index_of(frame);
        if (object == 0x6040 || object == 0x6060 || object == 0x6083 || object == 0x60ff ||
            object == 0x607a || object == 0x6081 || object == 0x2601 || object == 0x1016)
            out.push_back({static_cast<unsigned>(frame.id - 0x600), object, value_of(frame)});
    }
    return out;
}

// 走完快照、ARM 与使能，进入可以做运动的会话状态。
bool bring_up(DualBus& bus, yz_aim::Motor& motor, unsigned id, std::string& why) {
    const yz_aim::Snapshot snapshot = motor.snapshot(after());
    if (!snapshot.result) { why = "snapshot: " + snapshot.result.message; return false; }
    const yz_aim::Result armed = motor.arm(after());
    if (!armed) { why = "arm: " + armed.message; return false; }
    const yz_aim::Result enabled = motor.enable_output(after());
    if (!enabled) { why = "enable: " + enabled.message; return false; }
    (void)bus;
    (void)id;
    return true;
}

// 位置模式预置必须写完全部设定点，并在 0x6040=0x004F 停住、不发触发帧。
void test_stage_relative_order() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    bus.node[1].position = 10000;
    std::string why;
    require(bring_up(bus, motor, 1, why), "stage_relative 前置使能：" + why);
    bus.sent.clear();

    const yz_aim::Result staged = motor.stage_relative(5000, after(), 120);
    require(static_cast<bool>(staged), "stage_relative 成功：" + staged.message);
    require(motor.staged(), "stage_relative 置位预置标志");
    require(motor.staged_stationary(), "stage_relative 属于位置模式预置");
    require(motor.staged_target_pulses() == 15000, "stage_relative 记录的目标 = 实测位置 + 增量");

    const std::vector<Write> writes = writes_of(bus.sent);
    require(!writes.empty(), "stage_relative 发出 SDO 写");
    require(writes.front().index == 0x6040 && writes.front().value == 0x010f,
            "预置以 0x6040=0x010F 开始");
    require(writes.back().index == 0x6040 && writes.back().value == 0x004f,
            "预置以 0x6040=0x004F 结束，等待触发");
    unsigned mode = 0, target = 0, profile = 0, triggers = 0;
    for (const Write& write : writes) {
        if (write.index == 0x6060) { ++mode; require(write.value == 1, "预置选择位置模式 1"); }
        if (write.index == 0x607a) { ++target; require(write.value == 5000, "预置写相对增量 0x607A"); }
        if (write.index == 0x6081) { ++profile; require(write.value == 120, "预置写轮廓速度 0x6081"); }
        if (write.index == 0x6040 && write.value == 0x005f) ++triggers;
    }
    require(mode == 1 && target == 1 && profile == 1, "模式/目标/轮廓速度各只写一次");
    require(triggers == 0, "预置阶段不发触发帧");

    bus.sent.clear();
    const yz_aim::Result triggered = motor.trigger(after());
    require(static_cast<bool>(triggered), "trigger 成功：" + triggered.message);
    require(!motor.staged(), "trigger 清掉预置标志");
    const std::vector<Write> after_trigger = writes_of(bus.sent);
    require(after_trigger.size() == 1 && after_trigger[0].index == 0x6040 &&
            after_trigger[0].value == 0x005f,
            "trigger 只发一条 0x6040=0x005F");
    const yz_aim::Result again = motor.trigger(after());
    require(!again && again.error == yz_aim::Error::unsafe, "没有预置时 trigger 被拒绝");
}

// 预置的参数与状态门禁。
void test_stage_relative_guards() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    const yz_aim::Result too_early = motor.stage_relative(100, after());
    require(!too_early && too_early.error == yz_aim::Error::unsafe,
            "未使能时 stage_relative 被拒绝");

    std::string why;
    require(bring_up(bus, motor, 1, why), "guards 前置使能：" + why);
    require(!motor.stage_relative(0, after()), "增量为 0 被拒绝");
    require(!motor.stage_relative(100, after(), 0), "轮廓速度 0 被拒绝");
    require(!motor.stage_relative(100, after(), yz_aim::kMaxMotorRpm + 1), "轮廓速度超上限被拒绝");

    bus.node[1].position = yz_aim::kMaxPositionPulses - 10;
    const yz_aim::Result beyond = motor.stage_relative(100, after());
    require(!beyond && beyond.error == yz_aim::Error::invalid_argument,
            "目标越过手册坐标上限被拒绝");
    bus.node[1].position = 0;

    // 节点 127 与自己的心跳生产者冲突，必须在写任何控制字之前拒绝。
    yz_aim::Motor producer(bus, 127);
    const yz_aim::Result clash = producer.stage_relative(100, after());
    require(!clash && clash.error == yz_aim::Error::invalid_argument,
            "节点 127 不能做预置运动");
}

// 放弃预置要回到停止并清掉缓存；速度模式预置则要清速度目标。
void test_abort_staged() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, 1, why), "abort 前置使能：" + why);

    require(static_cast<bool>(motor.stage_relative(-4000, after())), "位置预置成功");
    bus.sent.clear();
    require(static_cast<bool>(motor.abort_staged(after())), "放弃位置预置成功");
    require(!motor.staged(), "放弃后预置标志清空");
    const std::vector<Write> writes = writes_of(bus.sent);
    require(!writes.empty() && writes[0].index == 0x6040 && writes[0].value == 0x010f,
            "放弃预置先回到 0x6040=0x010F");
    require(writes.back().index == 0x607a, "位置模式放弃时把 0x607A 复原为实测位置");

    bus.node[1].position = 0;
    require(static_cast<bool>(motor.stage_velocity(200, std::chrono::seconds(60), after())),
            "速度预置成功");
    require(motor.staged() && !motor.staged_stationary(), "速度预置标记为非位置预置");
    bus.sent.clear();
    require(static_cast<bool>(motor.abort_staged(after())), "放弃速度预置成功");
    const std::vector<Write> speed_writes = writes_of(bus.sent);
    require(!speed_writes.empty() && speed_writes.back().index == 0x60ff &&
            speed_writes.back().value == 0,
            "速度模式放弃时把 0x60FF 清零");
}

// 速度模式预置：写模式 3 与加速度，但不写 0x607A 也不启动。
void test_stage_velocity_order() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, 1, why), "stage_velocity 前置使能：" + why);
    require(!motor.stage_velocity(0, std::chrono::seconds(60), after()), "速度 0 被拒绝");
    require(!motor.stage_velocity(3001, std::chrono::seconds(60), after()), "速度超 3000 被拒绝");

    bus.node[1].position = 0;
    bus.sent.clear();
    require(static_cast<bool>(motor.stage_velocity(300, std::chrono::seconds(60), after())),
            "速度预置成功");
    const std::vector<Write> writes = writes_of(bus.sent);
    require(!writes.empty() && writes.front().index == 0x6040 && writes.front().value == 0x010f,
            "速度预置以 0x6040=0x010F 开始");
    bool wrote_target = false, wrote_mode3 = false, started = false;
    for (const Write& write : writes) {
        if (write.index == 0x607a) wrote_target = true;
        if (write.index == 0x6060 && write.value == 3) wrote_mode3 = true;
        if (write.index == 0x6040 && write.value == 0x000f) started = true;
    }
    require(wrote_mode3, "速度预置选择速度模式 3");
    require(!wrote_target, "速度预置不写 0x607A");
    require(!started, "速度预置不发启动控制字");

    bus.sent.clear();
    require(static_cast<bool>(motor.trigger_velocity(after())), "触发速度运动成功");
    const std::vector<Write> start = writes_of(bus.sent);
    require(start.size() >= 2 && start.front().index == 0x60ff && start.front().value == 300,
            "启动先写 0x60FF=300");
    require(start.back().index == 0x6040 && start.back().value == 0x000f,
            "启动最后写 0x6040=0x000F");

    bus.node[1].position = yz_aim::kMaxPositionPulses - 1000;
    const yz_aim::Result short_range = motor.stage_velocity(300, std::chrono::seconds(60), after());
    require(!short_range && short_range.error == yz_aim::Error::invalid_argument,
            "剩余行程不足整段常转与刹车距离时被拒绝");
    bus.node[1].position = yz_aim::kMinPositionPulses + 1000;
    const yz_aim::Result short_reverse = motor.stage_velocity(-300, std::chrono::seconds(60), after());
    require(!short_reverse && short_reverse.error == yz_aim::Error::invalid_argument,
            "反转方向的剩余行程同样受保护");
}

// 未确认急停：只发一帧 0x6040=0x0100，不等应答，并清掉预置。
void test_halt_unconfirmed() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, 1, why), "halt 前置使能：" + why);
    require(static_cast<bool>(motor.stage_relative(2000, after())), "halt 前置预置成功");

    bus.sent.clear();
    const unsigned pending = static_cast<unsigned>(bus.replies.size());
    require(static_cast<bool>(motor.halt_unconfirmed(after())), "未确认急停发送成功");
    require(!motor.staged(), "未确认急停清掉预置标志");
    require(bus.sent.size() == 1, "未确认急停只发一条 CAN 帧");
    require(bus.sent[0].id == 0x601 && bus.sent[0].dlc == 8 && bus.sent[0].data[0] == 0x2b &&
            index_of(bus.sent[0]) == 0x6040 && value_of(bus.sent[0]) == 0x0100,
            "未确认急停的内容是 0x6040<-0x0100");
    require(static_cast<unsigned>(bus.replies.size()) == pending + 1,
            "未确认急停不等应答：驱动器回的那条应答仍留在队列里没人读");
}

// 双轴监视：只读四项并做与运动循环相同的安全检查。
void test_poll_safety() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, 1, why), "poll_safety 前置使能：" + why);
    bus.node[1].position = -12345;

    bus.sent.clear();
    yz_aim::Snapshot sample{};
    require(static_cast<bool>(motor.poll_safety(sample, after())), "poll_safety 正常返回");
    require(sample.position_pulses == -12345, "poll_safety 读到实际位置");
    require(sample.statusword == 0x0437, "poll_safety 读到状态字");
    require(sample.alarm_code == 0, "poll_safety 读到报警码");
    unsigned reads = 0;
    for (const yz_aim::Frame& frame : bus.sent) {
        if (frame.id != 0x601 || frame.data[0] != 0x40) continue;
        const uint16_t object = index_of(frame);
        if (object == 0x6041 || object == 0x260e || object == 0x606c || object == 0x6064) ++reads;
    }
    require(reads == 4, "poll_safety 恰好只读状态字/报警/转速/位置四项");

    bus.node[1].alarm = 0x20;
    require(!motor.poll_safety(sample, after()), "有报警时 poll_safety 失败");
    require(sample.alarm_code == 0x20, "失败样本里带出报警码");
    require(motor.unsafe_latched(), "poll_safety 失败后会话锁止");
}

// 一轴异常、另一轴正常的偏差异常判据，由 CLI 计算；这里验证两轴共用一条总线时的报文顺序。
void test_two_axes_share_one_bus() {
    DualBus bus;
    yz_aim::Motor a(bus, 1);
    yz_aim::Motor b(bus, 2);
    std::string why;
    require(bring_up(bus, a, 1, why), "A 轴使能：" + why);
    require(bring_up(bus, b, 2, why), "B 轴使能：" + why);
    bus.node[1].position = 10000;
    bus.node[2].position = 10400;

    // 两轴顺序预置：任何一个 SDO 都在另一个节点发请求之前完成应答。
    const yz_aim::Result staged_a = a.stage_relative(5000, after(), 60);
    require(static_cast<bool>(staged_a), "A 轴预置成功：" + staged_a.message);
    const yz_aim::Result staged_b = b.stage_relative(5000, after(), 60);
    require(static_cast<bool>(staged_b), "B 轴预置成功：" + staged_b.message);
    require(a.staged_target_pulses() - b.staged_target_pulses() == -400,
            "两轴同增量预置后目标差等于初始位置差");

    bus.sent.clear();
    require(static_cast<bool>(a.trigger(after())), "A 轴触发成功");
    require(static_cast<bool>(b.trigger(after())), "B 轴触发成功");
    // 同步点的真正要求：两条触发帧之间没有任何 SDO 请求/应答，心跳是单向帧可以穿插。
    unsigned sdo_frames = 0, heartbeats = 0, others = 0;
    std::vector<unsigned> trigger_positions;
    for (unsigned i = 0; i < bus.sent.size(); ++i) {
        const yz_aim::Frame& frame = bus.sent[i];
        if (frame.id == 0x77f) { ++heartbeats; continue; }
        if (frame.id == 0x601 || frame.id == 0x602) {
            ++sdo_frames;
            if (index_of(frame) == 0x6040 && value_of(frame) == 0x005f) trigger_positions.push_back(i);
            continue;
        }
        ++others;
    }
    require(sdo_frames == 2, "同步点只有两条 SDO 帧");
    require(others == 0, "同步点没有其它类型的帧");
    require(trigger_positions.size() == 2, "两条触发帧都是 0x6040<-0x005F");
    require(bus.sent[trigger_positions[0]].id == 0x601 && bus.sent[trigger_positions[1]].id == 0x602,
            "两条触发帧分属 A、B 两轴");
    for (unsigned i = trigger_positions[0] + 1; i < trigger_positions[1]; ++i)
        require(bus.sent[i].id == 0x77f, "两条触发帧之间只允许出现心跳");
    require(heartbeats + 2 == bus.sent.size(), "报文流只由两条触发帧与心跳组成");
}

// stop 会清掉未触发的预置，避免下一次运动带着旧设定点启动。
void test_stop_clears_staged() {
    DualBus bus;
    yz_aim::Motor motor(bus, 1);
    std::string why;
    require(bring_up(bus, motor, 1, why), "stop 前置使能：" + why);
    require(static_cast<bool>(motor.stage_relative(3000, after())), "停止前预置成功");
    require(static_cast<bool>(motor.stop(after())), "停止成功");
    require(!motor.staged(), "stop 清掉预置标志");
}

// 真实总线上偶发丢一帧，不能让整个会话被 unsafe_ 锁死：
// 读是幂等的，必须自动重发并成功；写不能重发（可能被驱动器执行两次），必须如实失败。
void test_sdo_read_retries_but_write_never_does() {
    DualBus bus;
    yz_aim::SdoClient sdo(bus, 1);

    bus.drop_reads = 1;
    uint32_t value = 0;
    const yz_aim::Result recovered = sdo.read(0x1000, 0, 4, value, after());
    require(static_cast<bool>(recovered),
            "丢一帧读应答后 SDO 读自动重发并成功：" + recovered.message);
    require(value == 0x20192u, "重试后读到的仍是本对象的值");

    // 会话必须还能用：不能因为一次丢帧就留下不可信的 SDO 会话。
    uint32_t again = 0;
    require(static_cast<bool>(sdo.read(0x1000, 0, 4, again, after())),
            "丢帧重试后会话仍可用");
    require(again == 0x20192u, "会话恢复后读到同样的值");

    bus.sent.clear();
    bus.drop_writes = 1;
    const yz_aim::Result failed = sdo.write(0x2601, 0, 2, 0, after());
    require(!failed, "丢一帧写应答时 SDO 写如实失败");
    unsigned requests = 0;
    for (const yz_aim::Frame& frame : bus.sent)
        if (frame.id == 0x601) ++requests;
    require(requests == 1, "写失败时绝不重发，只发出一次请求");
}

// 监控回路（poll_safety / watch_motion）只给每次读 100 ms 的收帧桶，而真机 socket 会把这个桶
// 阻塞满。这种情况下重试必须自己再要一个新的收帧窗口 —— 沿用那个已经耗尽的截止时间，
// exchange 连预收帧队列都排不空，会直接以 unsafe 失败，重发帧根本发不出去。
void test_sdo_read_retry_gets_a_fresh_window_in_a_short_bucket() {
    DualBus bus;
    bus.block_when_empty = true;
    yz_aim::SdoClient sdo(bus, 1);

    bus.drop_reads = 1;
    uint32_t value = 0;
    const yz_aim::Result recovered =
        sdo.read(0x1000, 0, 4, value, after(std::chrono::milliseconds(100)));
    require(static_cast<bool>(recovered),
            "阻塞式收帧 + 100 ms 桶里丢一帧，读仍应重发成功：" + recovered.message);
    require(value == 0x20192u, "短桶重试后读到的仍是本对象的值");

    unsigned requests = 0;
    for (const yz_aim::Frame& frame : bus.sent)
        if (frame.id == 0x601 && frame.data[0] == 0x40) ++requests;
    require(requests == 2, "短桶里只重发一次即恢复，共两次读请求");
}

// 只读命令不该把整个会话搞死：读超时只是一次没收到应答，迟到的只可能是本对象的 upload 应答，
// 冒充不了控制写的确认。所以连续丢满三次读之后，会话必须还能继续用。
void test_read_timeout_does_not_poison_the_session() {
    DualBus bus;
    bus.block_when_empty = true;
    yz_aim::SdoClient sdo(bus, 1);

    bus.drop_reads = 3; // 三次尝试全部丢帧 → 这一次读彻底失败
    uint32_t value = 0;
    const yz_aim::Result failed = sdo.read(0x1000, 0, 4, value, after(std::chrono::milliseconds(300)));
    require(!failed, "三次尝试都丢帧时读如实失败");
    require(failed.error == yz_aim::Error::timeout, "失败原因是超时，而不是会话不可信");

    bus.drop_reads = 0;
    bus.block_when_empty = false;
    uint32_t recovered = 0;
    const yz_aim::Result again = sdo.read(0x1000, 0, 4, recovered, after());
    require(static_cast<bool>(again), "读超时之后会话仍然可用：" + again.message);
    require(recovered == 0x20192u, "会话恢复后读到正确的值");
}

// 写超时必须继续锁存：迟到的 download 确认可能让下一次写被误认为已经生效。
void test_write_timeout_still_poisons_the_session() {
    DualBus bus;
    yz_aim::SdoClient sdo(bus, 1);

    bus.drop_writes = 1;
    const yz_aim::Result failed = sdo.write(0x2601, 0, 2, 0, after(std::chrono::milliseconds(300)));
    require(!failed, "丢一帧写应答时写如实失败");

    uint32_t value = 0;
    const yz_aim::Result refused = sdo.read(0x1000, 0, 4, value, after(std::chrono::milliseconds(300)));
    require(!refused, "写失败之后读也必须被拒绝");
    require(refused.message.find("ambiguous") != std::string::npos,
            "写失败后会话保持不可信（ambiguous SDO session）：" + refused.message);
}

} // namespace

int main() {
    test_sdo_read_retries_but_write_never_does();
    test_sdo_read_retry_gets_a_fresh_window_in_a_short_bucket();
    test_read_timeout_does_not_poison_the_session();
    test_write_timeout_still_poisons_the_session();
    test_stage_relative_order();
    test_stage_relative_guards();
    test_abort_staged();
    test_stage_velocity_order();
    test_halt_unconfirmed();
    test_poll_safety();
    test_two_axes_share_one_bus();
    test_stop_clears_staged();
    std::printf("双轴同步合成台架测试通过：%d 项检查\n", checks);
    return 0;
}
