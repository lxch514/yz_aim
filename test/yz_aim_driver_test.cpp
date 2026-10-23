#include "yz_aim.h"
#include "yz_aim_mock.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <deque>
#include <unordered_map>

namespace {
int checks = 0;
void require(bool value, const char* message) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
yz_aim::Deadline deadline() { return yz_aim::Clock::now() + std::chrono::milliseconds(180); }
uint16_t index(const yz_aim::Frame& f) { return static_cast<uint16_t>(f.data[1] | f.data[2] << 8); }
uint32_t raw_value(const yz_aim::Frame& f) {
    return static_cast<uint32_t>(f.data[4]) | static_cast<uint32_t>(f.data[5]) << 8 |
           static_cast<uint32_t>(f.data[6]) << 16 | static_cast<uint32_t>(f.data[7]) << 24;
}
struct Synthetic {
    yz_aim::MockTransport wire;
    uint16_t status = 0x0437;
    uint16_t alarm = 0;
    uint16_t output = 0, output_flags = 0;
    int16_t speed = 0;
    int32_t position = 10000;
    uint32_t vendor = 0x331;
    bool drop_next = false, suppress_position_move = false, ignore_config_write = false;
    bool abort_battery = false, abort_output_read = false, ignore_output_write = false;
    bool inject_output_mismatch = false, mismatch_next_output_read = false;
    bool keep_speed_after_hold = false, drop_hold_speed_read = false, hold_acked = false;
    std::deque<int16_t> hold_speed_samples;
    unsigned hold_speed_reads = 0;
    unsigned hold_speed_delay_ms = 0;
    bool alarm_on_hold = false;
    bool heartbeat_safe = true;
    unsigned heartbeat_count = 0;
    bool delayed_position_move = false;
    int32_t pending_target = 0;
    unsigned motion_position_reads = 0;
    std::unordered_map<uint32_t, uint32_t> registers;
    Synthetic() {
        registers[0x1016 | (1U << 16)] = 0x007f07d0;
        wire.on_send = [this](const yz_aim::Frame& request, yz_aim::MockTransport& transport) {
            if (drop_next && request.data[0] != 0x40) { drop_next = false; return; }
            if (request.id == 0x77f) {
                ++heartbeat_count;
                if (alarm == 0x20) {
                    const bool safe = (output & 1U) == 0 &&
                        (registers[0x6040] & 0x0fU) == 0 &&
                        registers[0x60ff] == 0 &&
                        registers[0x607a] == static_cast<uint32_t>(position);
                    heartbeat_safe = heartbeat_safe && safe;
                    if (safe) alarm = 0;
                }
                return;
            }
            if (request.id != 0x601 || request.data[0] == 0x05) return;
            const uint16_t object = index(request);
            if (hold_acked && request.data[0] == 0x40 && object == 0x606c) {
                if (drop_hold_speed_read) return;
                if (hold_speed_delay_ms)
                    std::this_thread::sleep_for(std::chrono::milliseconds(hold_speed_delay_ms));
            }
            yz_aim::Frame response{0x581, 8, {0, request.data[1], request.data[2], request.data[3]}};
            if (request.data[0] == 0x40 &&
                ((object == 0x261e && abort_battery) ||
                 (object == 0x2601 && abort_output_read))) {
                response.data[0] = 0x80;
                response.data[6] = 0x02;
                response.data[7] = 0x06;
                transport.replies.push_back(response);
                return;
            }
            if (request.data[0] == 0x40) {
                uint32_t value = 0;
                unsigned width = 2;
                const uint32_t key = object | (static_cast<uint32_t>(request.data[3]) << 16);
                const auto stored = registers.find(key);
                switch (object) {
                    case 0x1000: value = 0x20192; width = 4; break;
                    case 0x1018: value = request.data[3] == 1 ? vendor : 1; width = 4; break;
                    case 0x1016: case 0x6081: case 0x6083: value = stored == registers.end() ? 0 : stored->second; width = 4; break;
                    case 0x261d: case 0x2615: value = stored == registers.end() ? 0 : stored->second; break;
                    case 0x6041: value = status; break;
                    case 0x260e: value = alarm; break;
                    case 0x2601:
                        value = output | output_flags;
                        if (mismatch_next_output_read) {
                            value ^= 1U;
                            mismatch_next_output_read = false;
                        }
                        break;
                    case 0x6064:
                        if (delayed_position_move && motion_position_reads > 0) {
                            const int64_t remaining = static_cast<int64_t>(pending_target) - position;
                            position = static_cast<int32_t>(static_cast<int64_t>(position) +
                                (remaining / 2 == 0 ? remaining : remaining / 2));
                            if (position == pending_target) { speed = 0; status |= 1U << 10; }
                        }
                        if (delayed_position_move && motion_position_reads > 0) ++motion_position_reads;
                        value = static_cast<uint32_t>(position); width = 4; break;
                    case 0x606c:
                        if (hold_acked) {
                            ++hold_speed_reads;
                            if (!hold_speed_samples.empty()) {
                                speed = hold_speed_samples.front();
                                hold_speed_samples.pop_front();
                            }
                        }
                        value = static_cast<uint16_t>(speed);
                        break;
                    case 0x6078: value = 2000; break;
                    case 0x6079: value = 654; break;
                    case 0x2612: value = 25; break;
                    case 0x261e: value = 7; break;
                    case 0x2614: value = 2; break;
                    default: return;
                }
                response.data[0] = width == 4 ? 0x43 : 0x4b;
                for (unsigned i = 0; i < width; ++i) response.data[4 + i] = static_cast<uint8_t>(value >> (i * 8));
            } else {
                response.data[0] = 0x60;
                if (!(ignore_config_write && object == 0x6081))
                    registers[object | (static_cast<uint32_t>(request.data[3]) << 16)] = raw_value(request);
                if (object == 0x2601) {
                    if (!ignore_output_write) {
                        // Bench: 0x2601=0 ACKs but low-bit remains enabled while
                        // 0x6040 still has all four operation bits set.
                        const bool operation_latched = (registers[0x6040] & 0x0fU) == 0x0fU;
                        output = raw_value(request) == 0 && operation_latched
                            ? 1 : static_cast<uint16_t>(raw_value(request));
                    }
                    if (inject_output_mismatch) mismatch_next_output_read = true;
                }
                if (object == 0x6040) {
                    const uint32_t cw = raw_value(request);
                    if (cw == 0x0100) { output = 0; status = 0x0414; }
                    if (cw == 0x000f) { output = 1; status = 0x0437; }
                    if (cw == 0x010f) {
                        status = 0x0437;
                        hold_acked = true;
                        if (!keep_speed_after_hold) speed = 0;
                        if (alarm_on_hold) { alarm = 0x21; status |= 1U << 3; }
                    }
                }
                if (object == 0x6040 && raw_value(request) == 0x005f &&
                    registers[0x6060] == 1 && output && !suppress_position_move) {
                    if (delayed_position_move) {
                        pending_target = static_cast<int32_t>(static_cast<int64_t>(position) +
                            static_cast<int32_t>(registers[0x607a]));
                        motion_position_reads = 1;
                        speed = 20;
                        status &= ~(1U << 10);
                    } else position = static_cast<int32_t>(static_cast<int64_t>(position) +
                        static_cast<int32_t>(registers[0x607a]));
                }
                if (object == 0x607a && registers[0x6040] == 0x002f &&
                    registers[0x6060] == 1 && output && !suppress_position_move) {
                    if (delayed_position_move) {
                        pending_target = static_cast<int32_t>(raw_value(request));
                        motion_position_reads = 1;
                        speed = 20;
                        status &= ~(1U << 10);
                    } else position = static_cast<int32_t>(raw_value(request));
                }
            }
            transport.replies.push_back(response);
        };
    }
    bool wrote(uint16_t object) const {
        return std::any_of(wire.sent.begin(), wire.sent.end(), [object](const yz_aim::Frame& f) {
            return f.id == 0x601 && f.data[0] != 0x40 && index(f) == object;
        });
    }
    std::vector<uint16_t> writes() const {
        std::vector<uint16_t> result;
        for (const auto& f : wire.sent) if (f.id == 0x601 && f.data[0] != 0x40) result.push_back(index(f));
        return result;
    }
};
}
int main() {
    require(!yz_aim::output_is_enabled(0) && !yz_aim::output_is_enabled(6) &&
            !yz_aim::output_is_enabled(0x8000) && yz_aim::output_is_enabled(1) &&
            yz_aim::output_is_enabled(7) && yz_aim::output_is_enabled(0x8001),
            "0x2601 only bit0 determines enabled; all other readback bits stay opaque");
    Synthetic test;
    {
        yz_aim::Motor motor(test.wire);
        auto state = motor.snapshot(deadline());
        require(static_cast<bool>(state.result), "snapshot succeeds");
        require(state.identity_sample.valid && state.vendor_id == 0x331 && state.device_type == 0x20192 &&
                state.speed_sample.valid && state.position_sample.valid && state.status_sample.valid,
                "snapshot fields sampled");
        require(state.current_a == 1.0 && state.voltage_v == 2.0 && state.battery_tenths == 7,
                "units and battery range");
        require(!test.wrote(0x6040) && test.writes().empty(), "read-only operation emits no writes");
        require(static_cast<bool>(motor.stop(deadline())) && test.writes().empty(),
                "explicit stop on disabled stationary output emits no enabling control word");
        require(static_cast<bool>(motor.arm(deadline())), "arm stationary");
        require(motor.move_relative(0, deadline()).error == yz_aim::Error::invalid_argument &&
                motor.move_relative(100, deadline(), 3001).error == yz_aim::Error::invalid_argument &&
                !test.wrote(0x1016), "zero delta and speed over manual maximum are rejected without watchdog");
        require(static_cast<bool>(motor.enable_output(deadline())), "explicit enable");
        auto sequence = test.writes();
        require(sequence.size() >= 7 && sequence[0] == 0x6040 && sequence[1] == 0x6040 &&
                sequence[2] == 0x60ff && sequence[3] == 0x607a &&
                sequence[4] == 0x1016 && sequence[5] == 0x6040 && sequence[6] == 0x2601,
                "hold, clear operation and stale targets, own heartbeat, then enable output");
        require(static_cast<bool>(motor.move_relative(100, deadline())), "bounded relative motion completes");
        sequence = test.writes();
        std::vector<uint32_t> consumer_writes;
        for (const auto& frame : test.wire.sent) {
            if (frame.id == 0x601 && frame.data[0] == 0x23 &&
                index(frame) == 0x1016 && frame.data[3] == 1)
                consumer_writes.push_back(raw_value(frame));
        }
        require(consumer_writes.size() == 1 &&
                consumer_writes[0] == 0x007f03e8 && motor.watchdog_active() &&
                test.registers.at(0x1016 | (1U << 16)) == 0x007f03e8,
                "successful STOP retains owned heartbeat and temporary consumer until output disable");
        require(std::any_of(test.wire.sent.begin(), test.wire.sent.end(), [](const yz_aim::Frame& f) {
            return f.id == 0x77f && f.dlc == 1 && f.data[0] == 5;
        }), "motion emits producer heartbeat");
        require(!motor.armed(), "completion requires rearm");
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_relative(-100, deadline())) &&
                test.position == 10000,
                "opposite motion returns to original raw position before cleanup");
        std::this_thread::sleep_for(std::chrono::milliseconds(430));
        require(static_cast<bool>(motor.disable_output(deadline())) && test.output == 0 &&
                !motor.watchdog_active() && test.wrote(0x2601) &&
                test.registers.at(0x1016 | (1U << 16)) == 0x007f07d0,
                "cleanup clears control bits, confirms disabled output, then restores old consumer");
    }
    Synthetic comm_loss;
    comm_loss.alarm = 0x20;
    comm_loss.output = 1;
    comm_loss.output_flags = 6;
    comm_loss.registers[0x60ff] = 20; // previous command must be cleared before restoring heartbeat.
    comm_loss.registers[0x607a] = 100;
    {
        yz_aim::Motor motor(comm_loss.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                comm_loss.writes().empty() && comm_loss.heartbeat_count == 0,
                "communication alarm 0x20 permits read-only arm, never sends recovery heartbeat");
        require(static_cast<bool>(motor.enable_output(deadline())) &&
                comm_loss.heartbeat_safe && comm_loss.alarm == 0 &&
                yz_aim::output_is_enabled(comm_loss.output | comm_loss.output_flags) &&
                motor.watchdog_active(),
                "explicit enable first clears operation and old targets, then heartbeat clears alarm");
        require(static_cast<bool>(motor.move_relative(100, deadline())) &&
                motor.watchdog_active() &&
                comm_loss.registers[0x1016 | (1U << 16)] == 0x007f03e8,
                "motion STOP holds output while preserving owned consumer and heartbeat");
        const unsigned before_idle = comm_loss.heartbeat_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(225));
        require(static_cast<bool>(motor.service_watchdog(deadline())) &&
                comm_loss.heartbeat_count > before_idle && motor.watchdog_active(),
                "worker services watchdog during an idle confirmation prompt");
        require(static_cast<bool>(motor.disable_output(deadline())) &&
                comm_loss.output == 0 && !motor.watchdog_active() &&
                comm_loss.registers[0x1016 | (1U << 16)] == 0x007f07d0,
                "after confirmed output bit clears, old consumer is restored and heartbeat retired");
        const unsigned after_disable = comm_loss.heartbeat_count;
        require(static_cast<bool>(motor.service_watchdog(deadline())) &&
                comm_loss.heartbeat_count == after_disable,
                "watchdog service after disable emits no producer frame");
    }
    Synthetic latched_operation;
    latched_operation.output = 1;
    latched_operation.output_flags = 6;
    {
        yz_aim::SdoClient sdo(latched_operation.wire, 1);
        uint32_t raw = 0;
        require(static_cast<bool>(sdo.write(0x6040, 0, 2, 0x010f, deadline())) &&
                static_cast<bool>(sdo.write(0x2601, 0, 2, 0, deadline())) &&
                static_cast<bool>(sdo.read(0x2601, 0, 2, raw, deadline())) && raw == 7,
                "0x2601=0 ACK alone leaves output enabled while controlword operation bits latch");
        require(static_cast<bool>(sdo.write(0x6040, 0, 2, 0x0100, deadline())) &&
                static_cast<bool>(sdo.read(0x2601, 0, 2, raw, deadline())) && raw == 6,
                "documented STOP with operation bits cleared yields disabled readback");
    }
    Synthetic absolute;
    {
        yz_aim::Motor motor(absolute.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "absolute fixture owns output after guarded enable");
        const auto count = absolute.wire.sent.size();
        require(static_cast<bool>(motor.move_absolute(13100, deadline())) &&
                absolute.position == 13100 && motor.watchdog_active(),
                "absolute position reaches requested signed32 target and keeps owned watchdog");
        std::vector<std::pair<uint16_t, uint32_t>> writes;
        for (size_t i = count; i < absolute.wire.sent.size(); ++i) {
            const auto& frame = absolute.wire.sent[i];
            if (frame.id == 0x601 && frame.data[0] != 0x40)
                writes.emplace_back(index(frame), raw_value(frame));
        }
        const auto trigger = std::find(writes.begin(), writes.end(), std::make_pair(uint16_t{0x6040}, uint32_t{0x002f}));
        require(trigger != writes.end() && trigger + 1 != writes.end() &&
                (trigger + 1)->first == 0x607a && (trigger + 1)->second == 13100 &&
                std::find(writes.begin(), trigger, std::make_pair(uint16_t{0x607a}, uint32_t{10000})) != trigger,
                "absolute mode clears old position before 0x002f and writes target only after immediate-mode control");
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_absolute(10000, deadline())) && absolute.position == 10000,
                "signed absolute position returns to original value");
        const int32_t wide_target = 10000 + static_cast<int32_t>(yz_aim::kOutputPulsesPerRevolution);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_absolute(wide_target, deadline(), 3000)) &&
                absolute.position == wide_target && absolute.registers[0x6081] == 3000,
                "absolute output revolution exceeds 90 degrees and accepts 3000 motor RPM");
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_absolute(10000, deadline(), 3000)) && absolute.position == 10000,
                "opposite full-revolution absolute position reaches original coordinate");
        const auto writes_before_reject = absolute.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.move_absolute(yz_aim::kMaxPositionPulses + 1, deadline()).error == yz_aim::Error::invalid_argument &&
                motor.move_absolute(yz_aim::kMinPositionPulses - 1, deadline()).error == yz_aim::Error::invalid_argument &&
                motor.move_absolute(10000, deadline()).error == yz_aim::Error::invalid_argument &&
                motor.move_absolute(wide_target, deadline(), 3001).error == yz_aim::Error::invalid_argument &&
                absolute.writes().size() == writes_before_reject,
                "out-of-manual endpoints, zero absolute distance and 3001 RPM reject before control TX");
        require(static_cast<bool>(motor.disable_output(deadline())),
                "absolute test cleans up with confirmed output disable");
    }
    Synthetic wide_relative;
    {
        yz_aim::Motor motor(wide_relative.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "wide relative fixture owns output");
        const int32_t full_output_turn = static_cast<int32_t>(yz_aim::kOutputPulsesPerRevolution);
        require(static_cast<bool>(motor.move_relative(-full_output_turn, deadline(), 3000)) &&
                wide_relative.position == 10000 - full_output_turn &&
                wide_relative.registers[0x6081] == 3000,
                "negative relative full output revolution accepted at 3000 RPM");
        const auto writes_before_reject = wide_relative.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.move_relative(std::numeric_limits<int32_t>::min(), deadline()).error ==
                    yz_aim::Error::invalid_argument &&
                wide_relative.writes().size() == writes_before_reject,
                "signed32 relative delta with out-of-manual endpoint rejects before TX");
        require(static_cast<bool>(motor.disable_output(deadline())),
                "wide relative fixture disables output safely");
    }
    Synthetic endpoints;
    endpoints.position = yz_aim::kMinPositionPulses;
    {
        yz_aim::Motor motor(endpoints.wire);
        require(static_cast<bool>(motor.arm(deadline())) && static_cast<bool>(motor.enable_output(deadline())),
                "endpoint fixture controls output from lower manual bound");
        require(static_cast<bool>(motor.move_absolute(yz_aim::kMaxPositionPulses, deadline(), 3000)) &&
                endpoints.position == yz_aim::kMaxPositionPulses,
                "absolute target crosses complete 3932160000 pulse span using int64 delta");
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_absolute(yz_aim::kMinPositionPulses, deadline(), 3000)) &&
                endpoints.position == yz_aim::kMinPositionPulses,
                "absolute reverse crossing reaches lower manual endpoint");
        const auto count = endpoints.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.move_relative(-1, deadline(), 3000).error == yz_aim::Error::invalid_argument &&
                endpoints.writes().size() == count,
                "one pulse past lower manual bound rejects before control TX");
        require(static_cast<bool>(motor.disable_output(deadline())), "endpoints fixture safely disables output");
    }
    Synthetic positive_relative;
    positive_relative.position = -200000000;
    {
        yz_aim::Motor motor(positive_relative.wire);
        require(static_cast<bool>(motor.arm(deadline())) && static_cast<bool>(motor.enable_output(deadline())) &&
                static_cast<bool>(motor.move_relative(yz_aim::kMaxRelativePositionPulses, deadline(), 3000)) &&
                positive_relative.position == 1947483647,
                "maximum signed32 positive relative delta reaches legal manual coordinate");
        const size_t count = positive_relative.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.move_relative(20000000, deadline(), 3000).error == yz_aim::Error::invalid_argument &&
                positive_relative.writes().size() == count,
                "relative overshoot of upper manual bound rejects before TX");
        require(static_cast<bool>(motor.disable_output(deadline())), "positive relative fixture cleanup");
    }
    Synthetic negative_relative;
    negative_relative.position = 200000000;
    {
        yz_aim::Motor motor(negative_relative.wire);
        require(static_cast<bool>(motor.arm(deadline())) && static_cast<bool>(motor.enable_output(deadline())) &&
                static_cast<bool>(motor.move_relative(std::numeric_limits<int32_t>::min(), deadline(), 3000)) &&
                negative_relative.position == -1947483648,
                "INT32_MIN relative delta is legal when the resulting coordinate remains in range");
        require(static_cast<bool>(motor.disable_output(deadline())), "negative relative fixture cleanup");
    }
    Synthetic measured_max_speed;
    {
        yz_aim::Motor motor(measured_max_speed.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "maximum measured speed fixture owns output");
        measured_max_speed.delayed_position_move = true;
        bool sampled_max = false;
        const auto original = measured_max_speed.wire.on_send;
        measured_max_speed.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& wire) {
            original(frame, wire);
            if (!sampled_max && measured_max_speed.motion_position_reads &&
                frame.id == 0x601 && frame.data[0] == 0x40 && index(frame) == 0x606c &&
                !wire.replies.empty()) {
                wire.replies.back().data[4] = 0x30;
                wire.replies.back().data[5] = 0x75; // signed16 raw 30000 = 3000.0 RPM
                sampled_max = true;
            }
        };
        require(static_cast<bool>(motor.move_relative(100, yz_aim::Clock::now() +
                    std::chrono::seconds(3), 3000)) && sampled_max &&
                !motor.unsafe_latched(),
                "3000 RPM measured during position motion is within the manual speed bound");
        require(static_cast<bool>(motor.disable_output(deadline())), "maximum speed fixture cleanup");
    }
    Synthetic jog_endpoint;
    jog_endpoint.position = yz_aim::kMaxPositionPulses;
    {
        yz_aim::Motor motor(jog_endpoint.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "jog endpoint fixture owns output");
        const size_t count = jog_endpoint.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.jog(1, std::chrono::seconds(5),
                    yz_aim::Clock::now() + yz_aim::jog_action_budget(std::chrono::seconds(5))).error ==
                    yz_aim::Error::invalid_argument &&
                motor.jog(3001, std::chrono::milliseconds(250),
                    yz_aim::Clock::now() + yz_aim::kStopActionBudget).error ==
                    yz_aim::Error::invalid_argument &&
                motor.jog(0, std::chrono::milliseconds(250),
                    yz_aim::Clock::now() + yz_aim::kStopActionBudget).error ==
                    yz_aim::Error::invalid_argument &&
                jog_endpoint.writes().size() == count,
                "jog reserves stopping distance and rejects 0 or 3001 RPM without TX");
        require(static_cast<bool>(motor.disable_output(deadline())), "jog endpoint fixture cleanup");
    }
    Synthetic measured_jog;
    {
        yz_aim::Motor motor(measured_jog.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "jog 3000 fixture owns output");
        const auto original = measured_jog.wire.on_send;
        measured_jog.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& wire) {
            original(frame, wire);
            if (frame.id == 0x601 && frame.data[0] != 0x40 &&
                index(frame) == 0x6040 && raw_value(frame) == 0x000f)
                measured_jog.speed = 30000;
        };
        require(static_cast<bool>(motor.jog(3000, std::chrono::milliseconds(250),
                    yz_aim::Clock::now() + yz_aim::jog_action_budget(std::chrono::milliseconds(250)))) &&
                measured_jog.registers.at(0x60ff) == 3000 && !motor.unsafe_latched(),
                "3000 RPM jog and its measured velocity do not falsely trip old 35 RPM gate");
        require(static_cast<bool>(motor.disable_output(deadline())), "jog 3000 fixture cleanup");
    }
    Synthetic jog_edge;
    {
        yz_aim::Motor motor(jog_edge.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "jog boundary fixture owns output");
        bool jogging = false;
        const auto original_on_send = jog_edge.wire.on_send;
        jog_edge.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& target) {
            if (frame.id == 0x601 && frame.data[0] != 0x40 &&
                index(frame) == 0x6040 && raw_value(frame) == 0x000f)
                jogging = true;
            if (jogging && frame.id == 0x601 && frame.data[0] == 0x40 &&
                (index(frame) == 0x6041 || index(frame) == 0x260e || index(frame) == 0x606c))
                std::this_thread::sleep_for(std::chrono::milliseconds(45));
            original_on_send(frame, target);
        };
        require(static_cast<bool>(motor.arm(yz_aim::Clock::now() + std::chrono::seconds(2))) &&
                static_cast<bool>(motor.jog(1, std::chrono::milliseconds(250),
                                            yz_aim::Clock::now() + std::chrono::seconds(3))) &&
                !motor.unsafe_latched() && motor.watchdog_active(),
                "jog near 250ms boundary completes without expiring an in-flight SDO at the stop boundary");
        require(static_cast<bool>(motor.disable_output(yz_aim::Clock::now() + std::chrono::seconds(2))),
                "jog edge fixture confirms output disabled");
    }
    Synthetic settling;
    {
        yz_aim::Motor motor(settling.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "settling fixture owns output before motor deceleration");
        settling.keep_speed_after_hold = true;
        settling.speed = 30;
        settling.hold_speed_delay_ms = 65;
        settling.hold_speed_samples = {30, 20, 10, 0};
        settling.hold_acked = false;
        settling.hold_speed_reads = 0;
        const auto initial_heartbeat = settling.heartbeat_count;
        const auto start = yz_aim::Clock::now();
        require(static_cast<bool>(motor.stop(start + std::chrono::milliseconds(800))) &&
                settling.hold_speed_reads >= 4 &&
                settling.heartbeat_count > initial_heartbeat &&
                motor.watchdog_active() && !motor.unsafe_latched(),
                "ACKed hold waits for successive decaying speed readings while servicing owned watchdog");
        // The first hold has consumed the simulated decay and the motor is now
        // stationary; a later hold starts a new observation of that state.
        settling.hold_speed_samples.clear();
        settling.speed = 0; // confirmed final sample persists as physical state for this fixture
        settling.keep_speed_after_hold = false;
        settling.hold_speed_delay_ms = 0;
        const auto refreshed = motor.snapshot(yz_aim::Clock::now() + std::chrono::milliseconds(500));
        require(static_cast<bool>(refreshed.result) && refreshed.output_sample.valid &&
                yz_aim::output_is_enabled(refreshed.output_enabled) &&
                refreshed.speed_sample.valid && refreshed.speed_rpm == 0,
                "repeat stop sees a fresh enabled and stationary output after the first hold");
        settling.hold_speed_reads = 0;
        settling.hold_acked = false;
        const auto already_stopped = yz_aim::Clock::now();
        const auto repeat_stop = motor.stop(already_stopped + std::chrono::milliseconds(500));
        const auto repeat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            yz_aim::Clock::now() - already_stopped);
        const bool immediate = repeat_stop && settling.hold_speed_reads == 1 &&
            repeat_elapsed < std::chrono::milliseconds(250);
        if (!immediate)
            std::fprintf(stderr, "repeat stop: error=%d context=%s speed_raw=%d reads=%u elapsed_ms=%lld "
                                 "output=%u heartbeat=%u\n",
                static_cast<int>(repeat_stop.error), repeat_stop.context.c_str(),
                static_cast<int>(settling.speed), settling.hold_speed_reads,
                static_cast<long long>(repeat_elapsed.count()), settling.output, settling.heartbeat_count);
        require(immediate, "stationary first readback returns immediately without consuming settling window");
        require(static_cast<bool>(motor.disable_output(yz_aim::Clock::now() + std::chrono::milliseconds(800))),
                "settling fixture disables output after speed reaches zero");
    }
    Synthetic high_speed_settling;
    {
        yz_aim::Motor motor(high_speed_settling.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "high-speed settling owns output");
        high_speed_settling.keep_speed_after_hold = true;
        high_speed_settling.speed = 30000;
        high_speed_settling.hold_speed_delay_ms = 80; // each SDO must finish within its 120ms reply window
        high_speed_settling.hold_speed_samples = {30000, 27000, 24000, 21000, 18000, 15000,
                                                  12000, 9000, 6000, 3000, 10, 0};
        high_speed_settling.hold_acked = false;
        high_speed_settling.hold_speed_reads = 0; // exclude enable_output() stop confirmations
        const unsigned heartbeat_before = high_speed_settling.heartbeat_count;
        const auto started = yz_aim::Clock::now();
        const auto stopped = motor.stop(started + std::chrono::seconds(8));
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            yz_aim::Clock::now() - started);
        const bool confirmed = stopped && elapsed > std::chrono::seconds(1) &&
            high_speed_settling.hold_speed_reads == 12 &&
            high_speed_settling.heartbeat_count > heartbeat_before &&
            motor.watchdog_active() && !motor.unsafe_latched();
        if (!confirmed)
            std::fprintf(stderr, "high-speed stop: error=%d context=%s reads=%u "
                                 "elapsed_ms=%lld heartbeats=%u->%u watchdog=%d unsafe=%d\n",
                static_cast<int>(stopped.error), stopped.context.c_str(),
                high_speed_settling.hold_speed_reads, static_cast<long long>(elapsed.count()),
                heartbeat_before, high_speed_settling.heartbeat_count,
                motor.watchdog_active(), motor.unsafe_latched());
        require(confirmed,
                "3000 RPM deceleration can take over one second and retains heartbeat until confirmed stop");
        high_speed_settling.hold_speed_delay_ms = 0;
        require(static_cast<bool>(motor.disable_output(deadline())), "high-speed settling cleanup");
    }
    Synthetic never_settles;
    {
        yz_aim::Motor motor(never_settles.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "never-settles fixture owns output");
        never_settles.keep_speed_after_hold = true;
        never_settles.speed = 35;
        never_settles.hold_acked = false;
        never_settles.hold_speed_reads = 0;
        const auto start = yz_aim::Clock::now();
        const auto result = motor.stop(start + std::chrono::milliseconds(220));
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(yz_aim::Clock::now() - start);
        require(result.error == yz_aim::Error::moving &&
                result.context.find("last_rpm=3.5") != std::string::npos &&
                result.context.find("elapsed_ms=") != std::string::npos &&
                result.context.find("budget_ms=") != std::string::npos &&
                elapsed.count() < 440 && never_settles.hold_speed_reads > 1 &&
                motor.unsafe_latched() && !motor.watchdog_active() &&
                never_settles.writes().back() == 0x6040,
                "stationary timeout is bounded by caller and records last RPM; failure latches and holds");
        const unsigned before = never_settles.heartbeat_count;
        (void)motor.service_watchdog(yz_aim::Clock::now() + std::chrono::seconds(1));
        require(never_settles.heartbeat_count == before, "failed stop never restarts heartbeat");
    }
    Synthetic delayed_failure;
    {
        yz_aim::Motor motor(delayed_failure.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "delayed failure fixture owns output");
        delayed_failure.keep_speed_after_hold = true;
        delayed_failure.speed = 30000;
        // The 30ms polling interval alone spans the caller deadline. Do not
        // make an individual mock SDO reply straddle that deadline.
        delayed_failure.hold_acked = false;
        delayed_failure.hold_speed_reads = 0;
        const unsigned heartbeat_before = delayed_failure.heartbeat_count;
        const auto started = yz_aim::Clock::now();
        const auto result = motor.stop(started + std::chrono::milliseconds(1400));
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            yz_aim::Clock::now() - started);
        const bool latched = result.error == yz_aim::Error::moving &&
            elapsed > std::chrono::seconds(1) &&
            delayed_failure.heartbeat_count > heartbeat_before &&
            motor.unsafe_latched() && !motor.watchdog_active() &&
            delayed_failure.writes().back() == 0x6040;
        if (!latched)
            std::fprintf(stderr, "delayed failure: error=%d context=%s reads=%u "
                                 "elapsed_ms=%lld heartbeats=%u->%u watchdog=%d unsafe=%d\n",
                static_cast<int>(result.error), result.context.c_str(),
                delayed_failure.hold_speed_reads, static_cast<long long>(elapsed.count()),
                heartbeat_before, delayed_failure.heartbeat_count,
                motor.watchdog_active(), motor.unsafe_latched());
        require(latched,
                "deceleration still moving at caller deadline latches failure after sustained heartbeat");
    }
    Synthetic overdue_heartbeat;
    {
        yz_aim::Motor motor(overdue_heartbeat.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "watchdog-overdue fixture owns output");
        overdue_heartbeat.keep_speed_after_hold = true;
        overdue_heartbeat.speed = 20;
        overdue_heartbeat.hold_acked = false;
        overdue_heartbeat.hold_speed_reads = 0;
        const auto original_on_send = overdue_heartbeat.wire.on_send;
        bool delayed = false;
        overdue_heartbeat.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& target) {
            if (!delayed && overdue_heartbeat.hold_acked && frame.id == 0x601 &&
                frame.data[0] == 0x40 && index(frame) == 0x6041) {
                delayed = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(730));
            }
            original_on_send(frame, target);
        };
        const unsigned before = overdue_heartbeat.heartbeat_count;
        const auto result = motor.stop(yz_aim::Clock::now() + std::chrono::milliseconds(950));
        require(result.error == yz_aim::Error::timeout && delayed &&
                result.context.find("SDO read 0x6041:00 phase=reply") != std::string::npos &&
                overdue_heartbeat.heartbeat_count == before &&
                motor.unsafe_latched() && !motor.watchdog_active(),
                "blocked SDO reports its 120ms reply timeout and never sends a late heartbeat");
    }
    Synthetic settle_alarm;
    {
        yz_aim::Motor motor(settle_alarm.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "settle alarm fixture owns output");
        settle_alarm.keep_speed_after_hold = true;
        settle_alarm.speed = 20;
        settle_alarm.hold_acked = false;
        settle_alarm.hold_speed_reads = 0;
        settle_alarm.alarm_on_hold = true;
        const auto start = yz_aim::Clock::now();
        require(motor.stop(start + std::chrono::milliseconds(700)).error == yz_aim::Error::alarm &&
                settle_alarm.hold_speed_reads == 0 && motor.unsafe_latched() &&
                !motor.watchdog_active() &&
                yz_aim::Clock::now() - start < std::chrono::milliseconds(200),
                "non-communication alarm after hold fails immediately before speed settling");
    }
    Synthetic settling_loss;
    {
        yz_aim::Motor motor(settling_loss.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "transport-loss fixture owns output");
        settling_loss.keep_speed_after_hold = true;
        settling_loss.speed = 20;
        settling_loss.hold_acked = false;
        settling_loss.hold_speed_reads = 0;
        settling_loss.drop_hold_speed_read = true;
        const auto result = motor.stop(yz_aim::Clock::now() + std::chrono::milliseconds(450));
        require(result.error == yz_aim::Error::timeout &&
                result.context.find("SDO read 0x606c:00 phase=reply") != std::string::npos &&
                motor.unsafe_latched() && !motor.watchdog_active() &&
                settling_loss.writes().back() == 0x6040,
                "lost speed reply stops polling, latches, and emits an unconfirmed hold");
        const unsigned before = settling_loss.heartbeat_count;
        const size_t sent_before = settling_loss.wire.sent.size();
        (void)motor.service_watchdog(yz_aim::Clock::now() + std::chrono::seconds(1));
        require(settling_loss.heartbeat_count == before && settling_loss.wire.sent.size() == sent_before,
                "poisoned settle never retries SDO or restarts heartbeat");
    }
    Synthetic slow_snapshot;
    {
        yz_aim::Motor motor(slow_snapshot.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "slow snapshot fixture owns output");
        const unsigned before = slow_snapshot.heartbeat_count;
        const auto original_on_send = slow_snapshot.wire.on_send;
        slow_snapshot.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& target) {
            if (frame.id == 0x601 && frame.data[0] == 0x40)
                std::this_thread::sleep_for(std::chrono::milliseconds(65));
            original_on_send(frame, target);
        };
        const auto observed = motor.snapshot(yz_aim::Clock::now() + std::chrono::milliseconds(1050));
        slow_snapshot.wire.on_send = original_on_send;
        require(observed.result && slow_snapshot.heartbeat_count > before &&
                !observed.unsafe_latched && motor.watchdog_active(),
                "owned snapshot services producer between slow SDOs without a late heartbeat");
        require(static_cast<bool>(motor.disable_output(deadline())),
                "slow snapshot cleans up confirmed disabled output");
    }
    Synthetic lost_read;
    {
        yz_aim::Motor motor(lost_read.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "lost read fixture owns output");
        const auto original_on_send = lost_read.wire.on_send;
        bool drop = true;
        lost_read.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& target) {
            if (drop && frame.id == 0x601 && frame.data[0] == 0x40 && index(frame) == 0x1000) {
                drop = false;
                return;
            }
            original_on_send(frame, target);
        };
        const auto observed = motor.snapshot(deadline());
        require(observed.result.error == yz_aim::Error::timeout && observed.unsafe_latched &&
                !motor.watchdog_active() && lost_read.writes().back() == 0x6040,
                "lost owned SDO latches unsafe, stops heartbeat and emits one unconfirmed hold");
        const auto count = lost_read.wire.sent.size();
        (void)motor.snapshot(deadline());
        require(lost_read.wire.sent.size() == count,
                "ambiguous SDO never causes a second hold or late heartbeat");
    }
    Synthetic other_alarm;
    other_alarm.alarm = 0x21;
    other_alarm.status |= 1U << 3;
    {
        yz_aim::Motor motor(other_alarm.wire);
        require(motor.arm(deadline()).error == yz_aim::Error::alarm &&
                other_alarm.writes().empty() && other_alarm.heartbeat_count == 0,
                "only exact communication-loss alarm is eligible for guarded recovery");
    }
    Synthetic fault;
    {
        yz_aim::Motor motor(fault.wire);
        require(static_cast<bool>(motor.arm(deadline())), "initial arm");
        require(static_cast<bool>(motor.enable_output(deadline())), "initial output");
        fault.drop_next = true;
        auto result = motor.move_relative(100, deadline());
        require(result.error == yz_aim::Error::timeout && motor.unsafe_latched(), "ACK timeout latches unsafe");
        const size_t before_rearm = fault.wire.sent.size();
        require(!motor.arm(deadline()) && !motor.move_relative(100, deadline()) &&
                fault.wire.sent.size() == before_rearm, "fault does not auto rearm or transmit");
        auto sequence = fault.writes();
        require(!sequence.empty() && sequence.back() == 0x6040, "fault attempts terminal hold");
    }
    Synthetic setup;
    {
        yz_aim::Motor motor(setup.wire);
        require(static_cast<bool>(motor.stage_node_id(2, deadline())), "node stage/save polling");
        auto sequence = setup.writes();
        require(sequence.size() == 2 && sequence[0] == 0x2615 && sequence[1] == 0x2614,
                "node stage followed by generic save1");
        require(static_cast<bool>(motor.stage_bitrate(500000, deadline())), "documented bitrate sequence");
        sequence = setup.writes();
        require(sequence.size() == 6 && sequence[2] == 0x2600 && sequence[3] == 0x6083 &&
                sequence[4] == 0x2604 && sequence[5] == 0x2600, "bitrate four ordered writes no save");
        require(static_cast<bool>(motor.set_profile_speed(0, deadline())) &&
                setup.registers[0x6081] == 0 &&
                static_cast<bool>(motor.set_profile_speed(3000, deadline())) &&
                setup.registers[0x6081] == 3000 &&
                motor.set_profile_speed(3001, deadline()).error == yz_aim::Error::invalid_argument,
                "stationary setup accepts manual 0..3000 profile; 3001 rejected");
        require(static_cast<bool>(motor.set_acceleration(500, deadline())), "safe volatile acceleration");
        require(!motor.stage_bitrate(200000, deadline()), "reject unsupported bitrate");
    }
    Synthetic readonly;
    readonly.output_flags = 6; // raw 6 means disabled; no spurious STOP or write.
    {
        yz_aim::Motor motor(readonly.wire);
        require(static_cast<bool>(motor.stop(deadline())) && readonly.writes().empty(),
                "raw6 disabled output needs no hold");
        require(static_cast<bool>(motor.arm(deadline())), "read-only arm with raw6 disabled output");
        require(static_cast<bool>(motor.disarm(deadline())) && readonly.writes().empty(),
                "armed-only disarm emits no control word");
    }
    Synthetic setup_failure;
    {
        yz_aim::Motor motor(setup_failure.wire);
        setup_failure.drop_next = false;
        require(static_cast<bool>(motor.arm(deadline())), "arm before config failure");
        setup_failure.drop_next = true;
        require(!motor.set_profile_speed(10, deadline()) && motor.unsafe_latched(),
                "configuration ACK timeout latches unsafe");
        require(!setup_failure.wrote(0x6040), "configuration failure emits no unintended hold");
    }
    Synthetic unsupported_battery;
    unsupported_battery.abort_battery = true;
    {
        yz_aim::Motor motor(unsupported_battery.wire);
        auto state = motor.snapshot(deadline());
        require(static_cast<bool>(state.result) && !state.battery_sample.valid &&
                state.battery_result.error == yz_aim::Error::abort_reply &&
                state.battery_result.abort_code == 0x06020000,
                "optional unsupported battery preserves abort without blocking critical telemetry");
        require(static_cast<bool>(motor.arm(deadline())) && unsupported_battery.writes().empty(),
                "optional battery abort does not expand motion safety gate");
    }
    Synthetic external_enabled;
    external_enabled.output = 1;
    external_enabled.output_flags = 6; // raw7 enabled; high bits unrelated to output control.
    {
        yz_aim::Motor motor(external_enabled.wire);
        auto state = motor.snapshot(deadline());
        require(state.result && state.output_sample.valid && state.output_enabled == 7 &&
                yz_aim::output_is_enabled(state.output_enabled) && external_enabled.writes().empty(),
                "vendor raw7 is preserved and indicates enabled without read-only writes");
        require(static_cast<bool>(motor.arm(deadline())) && external_enabled.writes().empty(),
                "arming an already-enabled stationary drive remains read-only");
        require(static_cast<bool>(motor.enable_output(deadline())) && external_enabled.output == 1,
                "explicit enable adopts already-enabled output after hold and setpoint clearing");
        const auto sequence = external_enabled.writes();
        require(sequence.size() >= 7 && sequence[0] == 0x6040 && sequence[1] == 0x6040 &&
                sequence[2] == 0x60ff && sequence[3] == 0x607a &&
                sequence[4] == 0x1016 && sequence[5] == 0x6040 && sequence[6] == 0x2601,
                "external enable adoption first confirms disabled control and clears targets");
        require(static_cast<bool>(motor.stop(deadline())) && external_enabled.output == 1,
                "raw7 owned output still requires acknowledged STOP");
        require(static_cast<bool>(motor.disable_output(deadline())) && external_enabled.output == 0,
                "raw6 disabled readback confirms output-disable cleanup");
        std::vector<uint32_t> control_values;
        for (const auto& frame : external_enabled.wire.sent)
            if (frame.id == 0x601 && frame.data[0] == 0x2b && index(frame) == 0x2601)
                control_values.push_back(raw_value(frame));
        require(control_values.size() == 2 && control_values[0] == 1 && control_values[1] == 0,
                "high bits in readback never appear in 0x2601 control writes");
    }
    Synthetic flagged_disabled;
    flagged_disabled.output_flags = 6;
    {
        yz_aim::Motor motor(flagged_disabled.wire);
        require(static_cast<bool>(motor.set_profile_speed(12, deadline())) &&
                !flagged_disabled.wrote(0x6040),
                "raw6 disabled output still permits stationary volatile setup");
    }
    Synthetic unreadable_output;
    unreadable_output.abort_output_read = true;
    {
        yz_aim::Motor motor(unreadable_output.wire);
        const auto state = motor.snapshot(deadline());
        require(state.result.error == yz_aim::Error::abort_reply &&
                !state.output_sample.valid && unreadable_output.writes().empty(),
                "failed 0x2601 read remains unavailable instead of fabricating bit0");
        require(!motor.arm(deadline()) && !motor.stop(deadline()) &&
                unreadable_output.writes().empty(),
                "missing output evidence blocks arm and unowned explicit stop without writes");
    }
    Synthetic owned_unreadable;
    {
        yz_aim::Motor motor(owned_unreadable.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())) &&
                static_cast<bool>(motor.stop(deadline())),
                "owned output established and previous hold confirmed");
        owned_unreadable.abort_output_read = true;
        const size_t before_loss = owned_unreadable.writes().size();
        const auto observed = motor.snapshot(deadline());
        require(observed.result.error == yz_aim::Error::abort_reply &&
                !observed.output_sample.valid && observed.unsafe_latched &&
                owned_unreadable.writes().size() == before_loss + 1 &&
                owned_unreadable.writes().back() == 0x6040,
                "owned output losing readable state immediately emits one unconfirmed hold");
        const size_t writes_before = owned_unreadable.writes().size();
        require(motor.stop(deadline()).error == yz_aim::Error::abort_reply &&
                motor.unsafe_latched() &&
                owned_unreadable.writes().size() == writes_before &&
                owned_unreadable.writes().back() == 0x6040,
                "further unreadable stop cannot treat old hold as acknowledged or emit duplicate emergency");
    }
    Synthetic enable_mismatch;
    enable_mismatch.output_flags = 6;
    enable_mismatch.ignore_output_write = true;
    enable_mismatch.inject_output_mismatch = true;
    {
        yz_aim::Motor motor(enable_mismatch.wire);
        require(static_cast<bool>(motor.arm(deadline())), "enable mismatch starts stationary");
        require(motor.enable_output(deadline()).error == yz_aim::Error::stale &&
                motor.unsafe_latched() && enable_mismatch.output == 0 &&
                enable_mismatch.writes().back() == 0x6040 &&
                raw_value(enable_mismatch.wire.sent.back()) == 0x0100,
                "0x2601 enable mismatch stops with operation bits clear");
    }
    Synthetic disable_mismatch;
    disable_mismatch.output_flags = 6;
    {
        yz_aim::Motor motor(disable_mismatch.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "disable mismatch fixture confirms enable from raw7");
        disable_mismatch.ignore_output_write = true;
        disable_mismatch.inject_output_mismatch = true;
        require(motor.disable_output(deadline()).error == yz_aim::Error::stale &&
                motor.unsafe_latched() && disable_mismatch.output == 0 &&
                disable_mismatch.writes().back() == 0x6040 &&
                raw_value(disable_mismatch.wire.sent.back()) == 0x0100,
                "disable readback mismatch holds without reasserting operation bits");
    }
    Synthetic wrong_identity;
    wrong_identity.vendor = 0x1234;
    {
        yz_aim::Motor motor(wrong_identity.wire);
        require(motor.arm(deadline()).error == yz_aim::Error::unsafe &&
                wrong_identity.writes().empty(), "unknown vendor cannot arm or write");
    }
    Synthetic alarm_reset;
    alarm_reset.alarm = 1;
    alarm_reset.status |= 1U << 3;
    {
        yz_aim::Motor motor(alarm_reset.wire);
        require(motor.reset_alarm(deadline()).error == yz_aim::Error::unsafe &&
                alarm_reset.writes().empty(), "unverified fault reset cannot energize output");
    }
    Synthetic bad_config;
    bad_config.ignore_config_write = true;
    {
        yz_aim::Motor motor(bad_config.wire);
        require(motor.set_profile_speed(15, deadline()).error == yz_aim::Error::stale &&
                motor.unsafe_latched() && !bad_config.wrote(0x6040),
                "config ACK without matching readback latches without stop write");
        require(motor.set_acceleration(1, deadline()).error == yz_aim::Error::invalid_argument,
                "limit-enable alias is not accepted as acceleration");
    }
    Synthetic overflow;
    overflow.position = std::numeric_limits<int32_t>::max() - 20;
    {
        yz_aim::Motor motor(overflow.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "overflow fixture controls output");
        const size_t writes_before = overflow.writes().size();
        require(motor.move_relative(100, deadline()).error == yz_aim::Error::invalid_argument &&
                motor.move_absolute(0, deadline()).error == yz_aim::Error::invalid_argument &&
                overflow.writes().size() == writes_before,
                "actual position already outside manual range rejects both move modes before TX");
    }
    Synthetic stale_position;
    stale_position.suppress_position_move = true;
    {
        yz_aim::Motor motor(stale_position.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "stale target fixture controls output");
        require(motor.move_relative(100, deadline()).error == yz_aim::Error::timeout &&
                motor.unsafe_latched(), "old attained bit with unchanged position never passes");
    }
    Synthetic outside_during_move;
    {
        yz_aim::Motor motor(outside_during_move.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "out-of-range readback fixture owns output");
        const auto original = outside_during_move.wire.on_send;
        bool moving = false;
        outside_during_move.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& wire) {
            original(frame, wire);
            if (frame.id == 0x601 && frame.data[0] != 0x40 &&
                index(frame) == 0x6040 && raw_value(frame) == 0x005f)
                moving = true;
            if (moving && frame.id == 0x601 && frame.data[0] == 0x40 &&
                index(frame) == 0x6064 && !wire.replies.empty()) {
                const uint32_t outside = static_cast<uint32_t>(yz_aim::kMaxPositionPulses + 1);
                for (unsigned i = 0; i < 4; ++i)
                    wire.replies.back().data[4 + i] = static_cast<uint8_t>(outside >> (i * 8));
            }
        };
        require(motor.move_relative(100, deadline()).error == yz_aim::Error::limit &&
                motor.unsafe_latched() && !motor.watchdog_active() &&
                outside_during_move.writes().back() == 0x6040,
                "motion actual coordinate beyond manual endpoint latches and attempts emergency hold");
    }
    Synthetic cancellation;
    cancellation.registers[0x1016 | (1U << 16)] = 0;
    {
        yz_aim::Motor motor(cancellation.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())), "cancellation fixture controls output");
        cancellation.hold_acked = false;
        cancellation.hold_speed_reads = 0;
        cancellation.keep_speed_after_hold = true;
        cancellation.hold_speed_delay_ms = 80; // 12 samples exceed 1s without any SDO timeout
        cancellation.hold_speed_samples = {30000, 27000, 24000, 21000, 18000, 15000,
                                           12000, 9000, 6000, 3000, 100, 0};
        const unsigned heartbeat_before_cancel = cancellation.heartbeat_count;
        const auto original_on_send = cancellation.wire.on_send;
        cancellation.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& wire) {
            original_on_send(frame, wire);
            if (frame.id == 0x601 && frame.data[0] == 0x2b &&
                index(frame) == 0x6040 && raw_value(frame) == 0x005f) {
                cancellation.speed = 30000;
                motor.request_cancel(); // cancel after a real motion trigger, with watchdog active
            }
        };
        const auto cancelled_started = yz_aim::Clock::now();
        const auto cancelled = motor.move_relative(100, deadline());
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            yz_aim::Clock::now() - cancelled_started);
        const bool stopped_after_cancel = cancelled.error == yz_aim::Error::cancelled &&
            elapsed > std::chrono::seconds(1) && !motor.unsafe_latched() &&
            motor.watchdog_active() && cancellation.heartbeat_count > heartbeat_before_cancel &&
            cancellation.hold_speed_reads == 12 && !motor.armed();
        if (!stopped_after_cancel)
            std::fprintf(stderr, "cancel stop: error=%d context=%s reads=%u "
                                 "elapsed_ms=%lld heartbeats=%u->%u watchdog=%d unsafe=%d\n",
                static_cast<int>(cancelled.error), cancelled.context.c_str(),
                cancellation.hold_speed_reads, static_cast<long long>(elapsed.count()),
                heartbeat_before_cancel, cancellation.heartbeat_count,
                motor.watchdog_active(), motor.unsafe_latched());
        require(stopped_after_cancel,
                "cancel uses independent bounded confirmed stop beyond exhausted move deadline");
        cancellation.wire.on_send = original_on_send;
        cancellation.keep_speed_after_hold = false;
        cancellation.hold_speed_delay_ms = 0;
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_relative(-100, deadline())) &&
                cancellation.position == 10000 && !motor.unsafe_latched(),
                "confirmed cancellation clears pending flag and permits rearm and opposite move");
        require(static_cast<bool>(motor.disable_output(deadline())) && cancellation.output == 0 &&
                cancellation.registers.at(0x1016 | (1U << 16)) == 0 && !motor.unsafe_latched(),
                "confirmed hold fulfils cancellation, restores original zero consumer and disables output");
        std::vector<uint32_t> restored;
        for (const auto& frame : cancellation.wire.sent)
            if (frame.id == 0x601 && frame.data[0] == 0x23 &&
                index(frame) == 0x1016 && frame.data[3] == 1) restored.push_back(raw_value(frame));
        require(restored.size() == 2 && restored[0] == 0x007f03e8 && restored[1] == 0,
                "cancel cleanup writes exact old consumer after confirmed stationary hold");
    }
    Synthetic live_motion;
    {
        yz_aim::Motor motor(live_motion.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "live fixture enables and owns output");
        live_motion.hold_acked = false;
        live_motion.hold_speed_reads = 0;
        live_motion.delayed_position_move = true;
        const unsigned heartbeat_before = live_motion.heartbeat_count;
        std::vector<yz_aim::Snapshot> progress;
        const auto started = yz_aim::Clock::now();
        auto capture = [&](const yz_aim::Snapshot& state) { progress.push_back(state); };
        require(static_cast<bool>(motor.move_relative(1024,
                    started + std::chrono::seconds(4), 1, capture)) &&
                live_motion.position == 11024 && live_motion.registers.at(0x6081) == 1 &&
                live_motion.heartbeat_count > heartbeat_before && motor.watchdog_active(),
                "slow selected 1RPM is written and preserves owned heartbeat through measured movement");
        bool changed_early = false;
        unsigned intermediate = 0;
        const auto original_optional = progress.front().current_sample.sampled_at;
        for (size_t i = 0; i < progress.size(); ++i) {
            const auto& state = progress[i];
            require(state.target_position_valid && state.target_position_pulses == 11024 &&
                    state.commanded_speed_rpm == 1 && state.current_sample.valid &&
                    state.current_sample.sampled_at == original_optional,
                    "command target is metadata while optional reading retains original sample time");
            if (state.position_pulses > 10000 && state.position_pulses < 11024) {
                require(state.position_sample.valid && state.status_sample.valid &&
                        state.alarm_sample.valid && state.speed_sample.valid &&
                        state.position_sample.sampled_at >= started &&
                        state.output_position_degrees ==
                            yz_aim::pulses_to_output_degrees(state.position_pulses),
                        "progress position is a fresh measured 6064 sample and derived output angle");
                ++intermediate;
                if (i > 0 && progress[i - 1].position_pulses != state.position_pulses)
                    changed_early = true;
            }
        }
        require(intermediate >= 2 && changed_early &&
                progress.back().position_pulses == 11024 && !progress.back().unsafe_latched,
                "multiple distinct intermediate actual positions arrive before move returns");
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.move_absolute(10000,
                    yz_aim::Clock::now() + std::chrono::seconds(4), 6)) &&
                live_motion.registers.at(0x6081) == 6,
                "absolute move uses the selected 6RPM profile at the wire");
        const size_t writes_before = live_motion.writes().size();
        require(static_cast<bool>(motor.arm(deadline())) &&
                motor.move_relative(100, deadline(), 0).error == yz_aim::Error::invalid_argument &&
                motor.move_absolute(10100, deadline(), yz_aim::kMaxMotorRpm + 1).error == yz_aim::Error::invalid_argument &&
                live_motion.writes().size() == writes_before,
                "invalid position RPM rejected before any motor writes");
        require(static_cast<bool>(motor.disable_output(deadline())),
                "live fixture disables output after selected-speed commands");
    }
    Synthetic cancel_failure;
    {
        yz_aim::Motor motor(cancel_failure.wire);
        require(static_cast<bool>(motor.arm(deadline())) &&
                static_cast<bool>(motor.enable_output(deadline())),
                "cancel-stop failure fixture owns output");
        cancel_failure.hold_acked = false;
        cancel_failure.hold_speed_reads = 0;
        const auto original = cancel_failure.wire.on_send;
        cancel_failure.wire.on_send = [&](const yz_aim::Frame& frame, yz_aim::MockTransport& transport) {
            original(frame, transport);
            if (frame.id == 0x601 && frame.data[0] == 0x2b &&
                index(frame) == 0x6040 && raw_value(frame) == 0x005f) {
                motor.request_cancel();
                cancel_failure.alarm_on_hold = true;
            }
        };
        require(motor.move_relative(100, deadline()).error == yz_aim::Error::alarm &&
                motor.unsafe_latched() && !motor.watchdog_active() &&
                cancel_failure.hold_speed_reads == 0,
                "failed cancellation stop latches and cannot resume producer");
    }
    Synthetic stale;
    stale.speed = 20;
    {
        yz_aim::Motor motor(stale.wire);
        require(motor.arm(deadline()).error == yz_aim::Error::moving, "moving motor cannot arm");
        require(stale.writes().empty(), "failed gate is read-only");
    }
    std::printf("yz_aim_driver_test: %d checks passed (synthetic, no device)\n", checks);
}
