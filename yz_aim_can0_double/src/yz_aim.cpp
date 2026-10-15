#include "yz_aim.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace yz_aim {
namespace {
Result error(Error code, const char* description, uint32_t abort = 0) {
    return {code, abort, description};
}
Deadline bounded(Deadline deadline, std::chrono::milliseconds interval = std::chrono::milliseconds(120)) {
    return std::min(deadline, Clock::now() + interval);
}
std::string sdo_context(bool is_read, uint16_t index, uint8_t subindex,
                        const char* phase, Clock::time_point started, Deadline deadline) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started);
    const auto budget = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - started);
    std::ostringstream detail;
    detail << "SDO " << (is_read ? "read" : "write") << " 0x" << std::hex
           << std::setw(4) << std::setfill('0') << index << ":" << std::setw(2)
           << static_cast<unsigned>(subindex) << std::dec << " phase=" << phase
           << " elapsed_ms=" << elapsed.count() << " budget_ms=" << std::max<int64_t>(0, budget.count());
    return detail.str();
}
Result sdo_failure(Result r, bool is_read, uint16_t index, uint8_t subindex,
                   const char* phase, Clock::time_point started, Deadline deadline) {
    r.context = sdo_context(is_read, index, subindex, phase, started, deadline);
    return r;
}
uint16_t load16(uint32_t value) { return static_cast<uint16_t>(value); }
bool fresh(const Reading& reading) {
    return reading.valid && Clock::now() >= reading.sampled_at &&
           Clock::now() - reading.sampled_at < std::chrono::milliseconds(400);
}
} // namespace

Result output_revolutions_to_pulses(double revolutions, int32_t& raw_pulses) {
    if (!std::isfinite(revolutions))
        return error(Error::invalid_argument, "output revolutions must be finite");
    const long double raw = static_cast<long double>(revolutions) * kOutputPulsesPerRevolution;
    if (raw < static_cast<long double>(std::numeric_limits<int32_t>::min()) - 0.5L ||
        raw > static_cast<long double>(std::numeric_limits<int32_t>::max()) + 0.5L)
        return error(Error::invalid_argument, "converted pulse count exceeds signed32 OD");
    const auto rounded = std::llround(raw);
    if (rounded < std::numeric_limits<int32_t>::min() ||
        rounded > std::numeric_limits<int32_t>::max())
        return error(Error::invalid_argument, "rounded pulse count exceeds signed32 OD");
    raw_pulses = static_cast<int32_t>(rounded);
    return {};
}

SdoClient::SdoClient(Transport& transport, uint8_t node) : transport_(transport), node_(node) {}

Result SdoClient::read(uint16_t index, uint8_t subindex, uint8_t width,
                       uint32_t& value, Deadline deadline) {
    if (width != 1 && width != 2 && width != 4)
        return sdo_failure(error(Error::invalid_argument, "SDO width must be 1, 2 or 4"),
                           true, index, subindex, "prequeue", Clock::now(), deadline);
    Frame frame{static_cast<uint16_t>(0x600 + node_), 8, {0x40,
        static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), subindex}};
    // 调用方给的大多是一个 100/120 ms 的收帧桶，桶内丢一帧读就失败，而读失败会被
    // Motor::write/fail 升级成 unsafe_ 永久锁存，整个会话报废（双轴共线时尤其容易）。
    // 读是幂等的，所以在"调用方截止时间 + 240 ms"内重发同一请求再试。
    // 写不重试：重发可能被驱动器执行两次。
    const Deadline retry_until = std::min(hard_deadline_, deadline + std::chrono::milliseconds(240));
    read_retried_ = false;
    Result last = sdo_failure(error(Error::timeout, "SDO read did not complete"),
                              true, index, subindex, "prequeue", Clock::now(), deadline);
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        if (attempt) {
            // 上一次超时可能残留一个迟到应答，它与本请求同索引，被当成本次应答也只是同一个值。
            poisoned_ = false;
            if (Clock::now() >= retry_until) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        // 重试必须自带一个新的收帧窗口：第一次尝试已经把调用方给的桶耗完，沿用那个过去的
        // 截止时间会让 exchange 连预收帧队列都排不空，直接以 unsafe 失败、根本发不出重发帧。
        const Deadline attempt_deadline = attempt == 0
            ? deadline
            : std::min(retry_until, Clock::now() + std::chrono::milliseconds(120));
        last = exchange(frame, index, subindex, width, &value, bounded(attempt_deadline));
        if (last) {
            read_retried_ = attempt != 0;
            sample_at_ = transmitted_at_ + (Clock::now() - transmitted_at_) / 2;
            return last;
        }
        if (last.error != Error::timeout) return last; // abort/malformed/transport 一律不重试
    }
    if (Clock::now() >= hard_deadline_)
        return error(Error::cancelled, "timed motion elapsed");
    return last;
}
Result SdoClient::write(uint16_t index, uint8_t subindex, uint8_t width,
                        uint32_t value, Deadline deadline) {
    if (width != 1 && width != 2 && width != 4)
        return sdo_failure(error(Error::invalid_argument, "SDO width must be 1, 2 or 4"),
                           false, index, subindex, "prequeue", Clock::now(), deadline);
    if ((width == 1 && value > 0xff) || (width == 2 && value > 0xffff))
        return sdo_failure(error(Error::invalid_argument, "SDO value overflows width"),
                           false, index, subindex, "prequeue", Clock::now(), deadline);
    Frame frame{static_cast<uint16_t>(0x600 + node_), 8, {
        static_cast<uint8_t>(width == 1 ? 0x2f : width == 2 ? 0x2b : 0x23),
        static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), subindex}};
    for (unsigned i = 0; i < width; ++i) frame.data[4 + i] = static_cast<uint8_t>(value >> (i * 8));
    return exchange(frame, index, subindex, width, nullptr, deadline);
}
Result SdoClient::write_pair(SdoClient& other, uint16_t index, uint8_t width,
                             uint32_t value, uint32_t other_value, Deadline deadline) {
    if (&other == this || node_ == other.node_ || (width != 2 && width != 4) ||
        (width == 2 && (value > 0xffff || other_value > 0xffff)))
        return error(Error::invalid_argument, "invalid paired SDO write");
    deadline = std::min({deadline, hard_deadline_, other.hard_deadline_,
                         Clock::now()+std::chrono::milliseconds(120)});
    auto frame = [index, width](uint8_t node, uint32_t v) {
        Frame f{static_cast<uint16_t>(0x600+node),8,{static_cast<uint8_t>(width==2 ? 0x2b : 0x23),
            static_cast<uint8_t>(index),static_cast<uint8_t>(index>>8),0,0,0,0,0}};
        for(unsigned i=0;i<width;++i) f.data[4+i]=static_cast<uint8_t>(v>>(i*8));
        return f;
    };
    const Frame a=frame(node_,value), b=frame(other.node_,other_value);
    Result r=exchange(a,index,0,width,nullptr,deadline,ExchangePhase::prepare_only);
    if(!r) return r;
    r=other.exchange(b,index,0,width,nullptr,deadline,ExchangePhase::prepare_only);
    if(!r) return r;
    if(Clock::now()>=deadline)
        return error(Clock::now()>=hard_deadline_ ? Error::cancelled : Error::timeout,
                     "paired write preparation deadline expired");
    // Drain both queues BEFORE either send. No SDO ACK wait between these two frames.
    transmitted_at_=Clock::now();
    if(!transport_.send(a)) { poisoned_=true; return error(Error::transport,"paired A send failed"); }
    other.transmitted_at_=Clock::now();
    if(!other.transport_.send(b)) { other.poisoned_=true; return error(Error::transport,"paired B send failed"); }
    r=exchange(a,index,0,width,nullptr,deadline,ExchangePhase::already_sent);
    if(!r) { other.poisoned_=true; r.message="paired A: "+r.message; return r; }
    r=other.exchange(b,index,0,width,nullptr,deadline,ExchangePhase::already_sent);
    if(!r) r.message="paired B: "+r.message;
    return r;
}
Result SdoClient::exchange(const Frame& request, uint16_t index, uint8_t subindex,
                           uint8_t width, uint32_t* value, Deadline deadline, ExchangePhase phase) {
    deadline = std::min(deadline, hard_deadline_);
    const auto started = Clock::now();
    const bool is_read = value != nullptr;
    auto failed = [&](Result r, const char* phase) {
        return sdo_failure(std::move(r), is_read, index, subindex, phase, started, deadline);
    };
    if (Clock::now() >= hard_deadline_) {
        if (phase == ExchangePhase::already_sent) {
            poisoned_ = true;
            return failed(error(Error::timeout, "paired write unconfirmed at motion deadline"), "reply");
        }
        return failed(error(Error::cancelled, "timed motion elapsed"), "prequeue");
    }
    if (node_ < 1 || node_ > 127) return failed(error(Error::invalid_argument, "invalid CANopen node"), "prequeue");
    if (poisoned_) return failed(error(Error::unsafe, "ambiguous SDO session; reconnect transport"), "prequeue");
    if (phase != ExchangePhase::already_sent) {
        // Drain all queued frames, including any stale ACK buried behind periodic PDOs.
        // At the observed ~20 PDO/s a 30-second confirmation can queue hundreds of
        // frames. Both time and frame bounds prevent a continuously flooded bus from
        // delaying STOP indefinitely; never transmit if the queue is not empty.
        // A delayed ACK arriving after an empty queue remains intrinsically ambiguous.
        constexpr unsigned kMaximumPrequeueFrames = 4096;
        const Deadline drain_until = std::min(deadline, Clock::now() + std::chrono::milliseconds(200));
        bool drained_to_empty = false;
        for (unsigned drained = 0; drained < kMaximumPrequeueFrames && Clock::now() < drain_until; ++drained) {
            if (service_) { Result r = service_(deadline); if (!r) return r; }
            Frame queued;
            if (!transport_.receive(queued, std::min(drain_until, Clock::now() + std::chrono::milliseconds(1)))) {
                if (transport_.failed()) { poisoned_ = true; return failed(error(Error::transport, "receive failed during prequeue drain"), "prequeue"); }
                drained_to_empty = Clock::now() < drain_until;
                break;
            }
        }
        if (Clock::now() >= hard_deadline_)
            return failed(error(Error::cancelled, "timed motion elapsed"), "prequeue");
        if (!drained_to_empty) {
            poisoned_ = true;
            return failed(error(Error::unsafe, "pre-transaction receive queue did not drain within bounded budget"), "prequeue");
        }
        if (Clock::now() >= deadline) return failed(error(Error::timeout, "SDO deadline expired before send"), "send");
        if (phase == ExchangePhase::prepare_only) return {};
        transmitted_at_ = Clock::now();
        if (!transport_.send(request)) { poisoned_ = true; return failed(error(Error::transport, "CAN send failed"), "send"); }
    }
    // Ignore unrelated traffic, including other SDO objects, with both time and frame bounds.
    unsigned seen = 0;
    while (seen < 128 && Clock::now() < deadline) {
        if (service_) {
            Result r = service_(deadline);
            if (!r) { if (!is_read) poisoned_ = true; return r; }
        }
        Frame reply;
        if (!transport_.receive(reply, bounded(deadline, std::chrono::milliseconds(10)))) {
            if (transport_.failed()) { poisoned_ = true; return failed(error(Error::transport, "CAN receive failed"), "reply"); }
            if (Clock::now() < deadline) continue;
            break;
        }
        ++seen;
        if (reply.id != 0x580 + node_ &&
            !(reply_node_alt_ && reply.id == 0x580 + reply_node_alt_)) continue;
        if (reply.dlc < 4) { poisoned_ = true; return failed(error(Error::malformed_reply, "truncated SDO reply"), "reply"); }
        if (reply.data[1] != static_cast<uint8_t>(index) ||
            reply.data[2] != static_cast<uint8_t>(index >> 8) || reply.data[3] != subindex) continue;
        if (reply.dlc != 8) { poisoned_ = true; return failed(error(Error::malformed_reply, "SDO DLC must be 8"), "reply"); }
        if (reply.data[0] == 0x80) {
            const uint32_t code = static_cast<uint32_t>(reply.data[4]) |
                (static_cast<uint32_t>(reply.data[5]) << 8) |
                (static_cast<uint32_t>(reply.data[6]) << 16) |
                (static_cast<uint32_t>(reply.data[7]) << 24);
            return failed(error(Error::abort_reply, "SDO abort", code), "reply");
        }
        if (!value) {
            if (reply.data[0] != 0x60 || reply.data[4] || reply.data[5] ||
                reply.data[6] || reply.data[7])
                { poisoned_ = true; return failed(error(Error::malformed_reply, "invalid SDO download acknowledgement"), "reply"); }
            return {};
        }
        const uint8_t expected = width == 1 ? 0x4f : width == 2 ? 0x4b : 0x43;
        if (reply.data[0] != expected) { poisoned_ = true; return failed(error(Error::malformed_reply, "SDO upload width mismatch"), "reply"); }
        // Expedited upload size is defined by its command byte. Unused payload
        // bytes have no defined value; this drive sign-extends negative 0x606C
        // into both unused bytes of its 0x4B (signed16) reply.
        uint32_t decoded = 0;
        for (unsigned i = 0; i < width; ++i) decoded |= static_cast<uint32_t>(reply.data[4 + i]) << (8 * i);
        *value = decoded;
        return {};
    }
    // 读超时不锁存会话：迟到的只可能是本对象的 upload 应答（0x43/0x4B/0x4F），它与任何
    // download 确认（0x60）格式不相交，冒充不了一次控制写；最坏情况只是下一次读同一个对象
    // 取到一个旧值，而下一次写同一个对象会因帧型不符被判 malformed 并如实失败 —— 不会被误判成功。
    // 写超时必须锁存：迟到的确认可能让下一次写被误认为已经生效。
    // 这样一条只读的 sync-status 在空总线上超时，就不会把整个会话搞成不可用。
    if (!is_read) poisoned_ = true;
    if (is_read && Clock::now() >= hard_deadline_)
        return failed(error(Error::cancelled, "timed motion elapsed"), "reply");
    return failed(error(Error::timeout, "SDO reply timed out or receive limit exhausted; reconnect"), "reply");
}

Motor::Motor(Transport& transport, uint8_t node, std::shared_ptr<SharedHeartbeat> heartbeat)
    : transport_(transport), sdo_(transport, node), node_(node),
      heartbeat_(heartbeat ? std::move(heartbeat) : std::make_shared<SharedHeartbeat>()) {
    if (node < 1 || node > 127) throw std::invalid_argument("YZ-AIM node must be 1..127");
    sdo_.set_service([this](Deadline deadline) -> Result {
        if (coordinated_motion_ && (cancel_requested_.load() || heartbeat_->cancel_requested.load()))
            return error(Error::cancelled, "operator cancelled timed motion");
        return service_watchdog(deadline);
    });
}
Motor::~Motor() {
    if (heartbeat_registered_ && heartbeat_->users) --heartbeat_->users;
}
void Motor::set_motion_deadline(Deadline deadline) {
    coordinated_motion_ = true;
    motion_deadline_ = deadline;
    sdo_.set_hard_deadline(deadline);
}
void Motor::clear_motion_deadline() {
    coordinated_motion_ = false;
    motion_deadline_ = Deadline::max();
    sdo_.set_hard_deadline(Deadline::max());
}
void Motor::request_cancel() noexcept {
    cancel_requested_.store(true);
    heartbeat_->cancel_requested.store(true);
}

Snapshot Motor::snapshot(Deadline deadline) {
    Snapshot output;
    output.unsafe_latched = unsafe_;
    // All SDOs and watchdog frames share one worker. A complete snapshot can
    // span many transactions; service the owned producer between reads rather
    // than waiting for the worker's idle timer to run after the snapshot.
    auto service_between_reads = [&]() {
        if (!watchdog_active()) return true;
        output.result = service_watchdog(deadline);
        return static_cast<bool>(output.result);
    };
    auto sample = [&](uint16_t index, uint8_t width, Reading& reading, uint32_t& value) {
        value = 0;
        if (Clock::now() >= deadline) { output.result = error(Error::timeout, "snapshot deadline expired"); return; }
        if (!service_between_reads()) return;
        Result r = sdo_.read(index, 0, width, value, bounded(deadline));
        if (!r) { output.result = r; return; }
        reading = {true, Clock::now()};
    };
    uint32_t raw = 0;
    Reading device_sample, vendor_sample, product_sample;
    sample(0x1000, 4, device_sample, raw); output.device_type = raw;
    auto identity_subindex = [&](uint8_t subindex, Reading& reading, uint32_t& target) {
        if (!output.result) return;
        if (Clock::now() >= deadline) { output.result = error(Error::timeout, "identity deadline expired"); return; }
        if (!service_between_reads()) return;
        raw = 0;
        output.result = sdo_.read(0x1018, subindex, 4, raw, bounded(deadline));
        if (output.result) { reading = {true, Clock::now()}; target = raw; }
    };
    identity_subindex(1, vendor_sample, output.vendor_id);
    identity_subindex(2, product_sample, output.product_code);
    output.identity_sample = {device_sample.valid && vendor_sample.valid && product_sample.valid,
        product_sample.sampled_at};
    if (output.result) { sample(0x6041, 2, output.status_sample, raw); output.statusword = load16(raw); }
    if (output.result) { sample(0x260e, 2, output.alarm_sample, raw); output.alarm_code = load16(raw); }
    if (output.result) { sample(0x2601, 2, output.output_sample, raw); output.output_enabled = load16(raw); }
    if (output.result) {
        sample(0x6064, 4, output.position_sample, raw);
        if (output.position_sample.valid) {
            output.position_pulses = static_cast<int32_t>(raw);
            output.output_position_degrees = pulses_to_output_degrees(output.position_pulses);
        }
    }
    if (output.result) {
        sample(0x606c, 2, output.speed_sample, raw);
        if (output.speed_sample.valid) {
            output.speed_rpm = static_cast<int16_t>(raw) / 10.0;
            output.output_speed_rpm = motor_to_output_rpm(output.speed_rpm);
        }
    }
    // Unsupported optional ODs are unavailable, not a critical motion gate.
    // Transport/timeout/malformed failures still poison the SDO session.
    auto optional = [&](uint16_t index, uint8_t width, Reading& reading,
                        uint32_t& value, Result& field_error) {
        if (!output.result) return;
        value = 0;
        if (Clock::now() >= deadline) {
            output.result = error(Error::timeout, "optional telemetry deadline expired");
            field_error = output.result;
            return;
        }
        if (!service_between_reads()) { field_error = output.result; return; }
        Result r = sdo_.read(index, 0, width, value, bounded(deadline));
        if (!r) {
            field_error = r;
            if (r.error != Error::abort_reply) output.result = r;
            return;
        }
        reading = {true, Clock::now()};
    };
    optional(0x6078, 2, output.current_sample, raw, output.current_result);
    if (output.current_sample.valid) output.current_a = load16(raw) / 2000.0;
    optional(0x6079, 2, output.voltage_sample, raw, output.voltage_result);
    if (output.voltage_sample.valid) output.voltage_v = load16(raw) / 327.0;
    optional(0x2612, 2, output.temperature_sample, raw, output.temperature_result);
    if (output.temperature_sample.valid) output.temperature_c = load16(raw);
    optional(0x261e, 2, output.battery_sample, raw, output.battery_result);
    if (output.battery_sample.valid) output.battery_tenths = load16(raw);
    // A failed read while we own enabled output is a lost safety observation:
    // stop the producer and attempt one unconfirmed hold. Never resume a stale
    // producer when USB/SDO communication returns.
    if (!output.result && (watchdog_started_ || output_enabled_ || control_attempted_))
        output.result = fail(output.result, deadline);
    output.unsafe_latched = unsafe_;
    last_ = output;
    return output;
}

Result Motor::verify_value(uint16_t index, uint8_t subindex, uint8_t width,
                           uint32_t expected, Deadline deadline) {
    uint32_t observed = 0;
    Result r = sdo_.read(index, subindex, width, observed, bounded(deadline));
    if (!r) return r;
    if (observed != expected) return error(Error::stale, "post-write readback mismatches requested value");
    return {};
}
Result Motor::verify_output(bool expected_enabled, Deadline deadline) {
    uint32_t observed = 0;
    Result r = sdo_.read(0x2601, 0, 2, observed, bounded(deadline));
    if (!r) return r;
    if (output_is_enabled(static_cast<uint16_t>(observed)) != expected_enabled)
        return error(Error::stale, "output-enable bit in 0x2601 readback mismatches requested value");
    return {};
}
Result Motor::verify_stopped(Deadline deadline, bool allow_communication_alarm) {
    // A hold ACK confirms the command, not that a coasting motor is stationary.
    // At 500 RPM/s, a 3000 RPM hold may take six seconds to decelerate.
    // Reserve another second for measured confirmation, capped by caller deadline.
    const auto started = Clock::now();
    const Deadline until = std::min(deadline, started + std::chrono::milliseconds(7000));
    double last_rpm = 0;
    bool speed_observed = false;
    auto unsettled = [&]() -> Result {
        Result result = error(speed_observed ? Error::moving : Error::timeout,
                              "hold ACK received but stationary readback unconfirmed");
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started);
        const auto budget = std::chrono::duration_cast<std::chrono::milliseconds>(until - started);
        std::ostringstream detail;
        detail << "stop settling last_rpm=";
        if (speed_observed) detail << last_rpm;
        else detail << "unobserved";
        detail << " elapsed_ms=" << elapsed.count()
               << " budget_ms=" << std::max<int64_t>(0, budget.count());
        result.context = detail.str();
        return result;
    };
    auto read = [&](uint16_t object, uint32_t& value) -> Result {
        // A near-expired settling window must not initiate a doomed SDO and
        // poison an otherwise healthy transport merely to await its deadline.
        if (until - Clock::now() < std::chrono::milliseconds(20)) return unsettled();
        if (watchdog_active()) {
            Result serviced = service_watchdog(until);
            if (!serviced) return serviced;
        }
        if (until - Clock::now() < std::chrono::milliseconds(20)) return unsettled();
        return sdo_.read(object, 0, 2, value, bounded(until));
    };
    while (Clock::now() < until) {
        uint32_t status = 0, speed = 0, alarm = 0;
        Result r = read(0x6041, status);
        if (!r) return r;
        if (status & 0xc000) return error(Error::limit, "limit observed after hold");
        r = read(0x260e, alarm);
        if (!r) return r;
        if (alarm != 0 && !(allow_communication_alarm && alarm == 0x20))
            return error(Error::alarm, "hold observed a non-communication alarm");
        if ((status & (1U << 3)) && !(allow_communication_alarm && alarm == 0x20))
            return error(Error::alarm, "alarm observed after hold");
        r = read(0x606c, speed);
        if (!r) return r;
        last_rpm = static_cast<int16_t>(speed) / 10.0;
        speed_observed = true;
        if (std::abs(last_rpm) <= 0.5) return {};
        std::this_thread::sleep_until(std::min(until, Clock::now() + std::chrono::milliseconds(30)));
    }
    return unsettled();
}
Result Motor::require_stationary(Deadline deadline, bool require_disabled,
                                  bool allow_communication_alarm) {
    if (unsafe_) return error(Error::unsafe, "unsafe state latched; operator intervention required");
    Snapshot s = snapshot(deadline);
    if (!s.result) return control_attempted_ ? fail(s.result, deadline) : s.result;
    if (!fresh(s.identity_sample) || s.vendor_id != 0x331 || s.product_code != 1 ||
        s.device_type != 0x20192) return error(Error::unsafe, "unrecognized YZ-AIM vendor/product/device type");
    if (!fresh(s.status_sample) || !fresh(s.position_sample) || !fresh(s.speed_sample) ||
        !fresh(s.alarm_sample) || !fresh(s.output_sample)) {
        Result bad = error(Error::stale, "stale motor evidence");
        return control_attempted_ ? fail(bad, deadline) : bad;
    }
    const bool recoverable_communication_alarm = allow_communication_alarm && s.alarm_code == 0x20;
    if ((s.statusword & (1U << 3) || s.alarm_code) && !recoverable_communication_alarm) {
        Result bad = error(Error::alarm, "drive reports alarm");
        return control_attempted_ ? fail(bad, deadline) : bad;
    }
    if (s.statusword & 0xc000) {
        Result bad = error(Error::limit, "drive reports position limit");
        return control_attempted_ ? fail(bad, deadline) : bad;
    }
    if (std::abs(s.speed_rpm) > 0.5) {
        Result bad = error(Error::moving, "motor is moving");
        return control_attempted_ ? fail(bad, deadline) : bad;
    }
    if (require_disabled && output_is_enabled(s.output_enabled))
        return error(Error::unsafe, "disable drive output first");
    return {};
}

Result Motor::hold(Deadline deadline) {
    // 0x0100 is the confirmed safe stop after operation-clear was attempted;
    // 0x010F could turn operation bits back on during disable cleanup.
    return sdo_.write(0x6040, 0, 2, operation_clear_attempted_ ? 0x0100 : 0x010f, deadline);
}
Result Motor::fail(Result result, Deadline /*deadline*/) {
    if (unsafe_) return result; // do not repeat an unconfirmed emergency frame
    unsafe_ = true;
    if (heartbeat_registered_) heartbeat_->faulted = true;
    velocity_running_ = false;
    armed_ = false;
    // Setup and read-only failures do not authorize a motor-control write.
    // If we may have taken control, emit a single UNCONFIRMED emergency hold frame.
    // Never treat a late SDO ACK as proof that this hold was accepted.
    if (control_attempted_ || output_enabled_) {
        // After 0x0100 was attempted, reverting to 0x010F would reassert
        // operation bits and could re-enable a motor with 0x2601=0 ACKed.
        const uint8_t low = operation_clear_attempted_ ? 0x00 : 0x0f;
        Frame emergency{static_cast<uint16_t>(0x600 + node_), 8, {0x2b, 0x40, 0x60, 0, low, 0x01}};
        (void)transport_.send(emergency);
    }
    watchdog_started_ = false;
    sustaining_ = false;
    staged_ = false;
    return result;
}
Result Motor::write(uint16_t index, uint8_t subindex, uint8_t width,
                    uint32_t value, Deadline deadline) {
    if ((cancel_requested_.load() || (coordinated_motion_ && heartbeat_->cancel_requested.load())) && control_attempted_) {
        if (coordinated_motion_) return error(Error::cancelled, "operator cancelled coordinated motion");
        return cancel_motion(deadline);
    }
    if (watchdog_started_ && Clock::now() - heartbeat_->last_sent >= std::chrono::milliseconds(200)) {
        Result heart = heartbeat(deadline);
        if (!heart) return heart;
    }
    Result r = sdo_.write(index, subindex, width, value, bounded(deadline));
    if (r.error == Error::cancelled && r.message == "timed motion elapsed") return r;
    return r ? r : fail(r, deadline);
}
Result Motor::arm(Deadline deadline) {
    // Alarm 0x20 is vendor-documented loss of producer heartbeat. ARM itself
    // stays read-only; only a later explicit ENABLE can attempt safe recovery.
    Result r = require_stationary(deadline, false, true);
    if (!r) return r;
    armed_ = true;
    return {};
}
Result Motor::stop(Deadline deadline) {
    clear_motion_deadline();
    velocity_running_ = false;
    sustaining_ = false;
    staged_ = false;
    if (!control_attempted_) {
        // A previous, acknowledged motion hold releases control ownership, but
        // explicit output enable may remain in effect. Refresh old observations
        // before deciding whether another hold can be sent safely.
        if (!fresh(last_.identity_sample) || !fresh(last_.output_sample) ||
            !fresh(last_.speed_sample)) {
            const Snapshot observed = snapshot(deadline);
            if (!observed.result) {
                if (!output_enabled_) return observed.result;
                // Our output remains owned, but its fresh state is unavailable.
                // Report the read failure and send only an unconfirmed hold.
                control_attempted_ = true;
                return fail(observed.result, deadline);
            }
        }
        if (!fresh(last_.identity_sample) || !fresh(last_.output_sample) ||
            !fresh(last_.speed_sample) || last_.vendor_id != 0x331 ||
            last_.product_code != 1 || last_.device_type != 0x20192)
            return error(Error::unsafe, "fresh identity/output/speed evidence required before explicit stop");
        if (!output_is_enabled(last_.output_enabled)) {
            armed_ = false;
            if (std::abs(last_.speed_rpm) > 0.5)
                return error(Error::moving, "output disabled but motor still moving; use physical stop");
            cancel_requested_.store(false); // an already-stopped request is fulfilled
            return {};
        }
        control_attempted_ = true; // explicit operator stop on an observed enabled output
    }
    armed_ = false;
    Result r = hold(bounded(std::max(deadline, Clock::now() + std::chrono::milliseconds(150))));
    if (!r) {
        unsafe_ = true; watchdog_started_ = false; heartbeat_->faulted = true;
        const uint8_t low = operation_clear_attempted_ ? 0x00 : 0x0f;
        Frame emergency{static_cast<uint16_t>(0x600 + node_), 8, {0x2b, 0x40, 0x60, 0, low, 0x01}};
        (void)transport_.send(emergency); // unconfirmed; SDO session may be poisoned
        return r;
    }
    Result observed = verify_stopped(deadline, true);
    if (!observed) return fail(observed, deadline);
    cancel_requested_.store(false); // verified hold fulfils any cancellation request
    heartbeat_->cancel_requested.store(false);
    // STOP remains held while the output belongs to this session. Keep the
    // producer heartbeat until output-disable is read back as bit 0 = 0;
    // restoring a monitored consumer here would raise alarm 0x20 in ~1 second.
    if (!consumer_overridden_) watchdog_started_ = false;
    control_attempted_ = false;
    return {};
}
Result Motor::disarm(Deadline deadline) {
    if (!control_attempted_ && !watchdog_started_) { armed_ = false; return {}; }
    return stop(deadline);
}
Result Motor::enable_output(Deadline deadline) {
    if (!armed_ || unsafe_) return error(Error::unsafe, "arm with fresh stationary evidence first");
    Result r = require_stationary(deadline, false, true);
    if (!r) return r;
    // The device may already be enabled with comm-loss alarm 0x20. Confirm
    // acknowledged hold and stationary readback before producing any heartbeat:
    // the manual warns a resumed heartbeat can resume the previous motion.
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = verify_stopped(deadline, true))) return fail(r, deadline);
    // Real bench: 0x010F self-clears bit8 and leaves 0x000F/2601 raw7;
    // a write of 2601=0 alone still reads raw7. Clearing operation bits
    // with documented controlword 0x0100 produced 6040=0, raw6 and zero RPM.
    operation_clear_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x0100, deadline))) return r;
    if (!(r = verify_stopped(deadline, true))) return fail(r, deadline);
    if (!(r = verify_output(false, deadline))) return fail(r, deadline);
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x607a, 0, 4, static_cast<uint32_t>(last_.position_pulses), deadline))) return r;
    // Only now is it safe to restore a failed producer heartbeat: the drive
    // has confirmed its output is disabled and both old setpoints are cleared.
    if (!(r = start_watchdog(deadline))) return r;
    // Receive-only alarm polling is bounded and the owned heartbeat continues.
    const Deadline recover_until = std::min(deadline, Clock::now() + std::chrono::milliseconds(500));
    bool recovered = false;
    while (Clock::now() < recover_until) {
        if (!(r = service_watchdog(deadline))) return r;
        uint32_t alarm = 0;
        if (!(r = sdo_.read(0x260e, 0, 2, alarm, bounded(recover_until)))) return fail(r, deadline);
        if (alarm == 0) { recovered = true; break; }
        if (alarm != 0x20) return fail(error(Error::alarm, "unexpected alarm during heartbeat recovery"), deadline);
        std::this_thread::sleep_until(std::min(recover_until, Clock::now() + std::chrono::milliseconds(30)));
    }
    if (!recovered) return fail(error(Error::alarm, "communication alarm did not clear after guarded heartbeat"), deadline);
    // p15: start operation only after the guarded heartbeat recovered alarm0;
    // bit4 remains clear so this is not a position trigger.
    if (!(r = write(0x6040, 0, 2, 0x000f, deadline))) return r;
    if (!(r = write(0x2601, 0, 2, 1, deadline))) return r;
    if (!(r = verify_output(true, deadline))) return fail(r, deadline);
    if (!(r = verify_stopped(deadline))) return fail(r, deadline);
    output_enabled_ = true;
    operation_clear_attempted_ = false;
    return {};
}
Result Motor::disable_output(Deadline deadline) {
    Result stop_result = stop(deadline);
    if (!stop_result) return stop_result;
    control_attempted_ = true; // output disable ACK may be ambiguous; retain emergency hold ownership
    // Bench evidence: with 6040 still at 0x000F, an acknowledged 2601=0
    // leaves bit0 enabled. Clear start/voltage/operation bits first.
    operation_clear_attempted_ = true;
    Result r = write(0x6040, 0, 2, 0x0100, deadline);
    if (!r) return r;
    if (!(r = verify_stopped(deadline, true))) return fail(r, deadline);
    if (!(r = write(0x2601, 0, 2, 0, deadline))) return r;
    if (!(r = verify_output(false, deadline))) return fail(r, deadline);
    output_enabled_ = false;
    control_attempted_ = false;
    // Only the confirmed disabled state permits retiring our heartbeat and
    // restoring the exact previous consumer, even after a cancelled motion.
    if (consumer_overridden_) return finish_motion(deadline);
    watchdog_started_ = false;
    return {};
}
Result Motor::reset_alarm(Deadline deadline) {
    (void)deadline;
    // Vendor examples do not establish a safe reset controlword while output is
    // disabled: 0x018F would set voltage/operation-enable bits. Gate it completely.
    return error(Error::unsafe, "alarm reset gated: disabled-output controlword semantics unverified");
}
Result Motor::set_profile_speed(uint32_t rpm, Deadline deadline) {
    if (rpm > kMaxMotorRpm) return error(Error::invalid_argument, "profile speed must be 0..3000 RPM");
    Result r = require_stationary(deadline, true);
    if (!r) return r;
    if (!(r = write(0x6081, 0, 4, rpm, deadline))) return r;
    r = verify_value(0x6081, 0, 4, rpm, deadline);
    return r ? r : fail(r, deadline);
}
Result Motor::set_acceleration(uint32_t acceleration, Deadline deadline) {
    if (acceleration < 10 || acceleration > 1000)
        return error(Error::invalid_argument, "acceleration must be 10..1000 RPM/s; low values overlap limit setup");
    Result r = require_stationary(deadline, true);
    if (!r) return r;
    if (!(r = write(0x6083, 0, 4, acceleration, deadline))) return r;
    r = verify_value(0x6083, 0, 4, acceleration, deadline);
    return r ? r : fail(r, deadline);
}
Result Motor::set_current_limit(uint16_t milliamps, Deadline deadline) {
    if (milliamps < 10 || milliamps > 10000 || milliamps % 10)
        return error(Error::invalid_argument, "current limit must be 10..10000mA, multiples of 10");
    Result r = require_stationary(deadline, true);
    if (!r) return r;
    if (!(r = write(0x261d, 0, 2, milliamps, deadline))) return r;
    r = verify_value(0x261d, 0, 2, milliamps, deadline);
    return r ? r : fail(r, deadline);
}
Result Motor::heartbeat(Deadline deadline) {
    if (!heartbeat_->users || unsafe_) return {};
    if (heartbeat_->faulted)
        return fail(error(Error::unsafe, "shared watchdog fault latched; stop both axes"), deadline);
    const auto now = Clock::now();
    if (now >= deadline) return fail(error(Error::timeout, "watchdog heartbeat deadline expired"), deadline);
    // A late heartbeat may resume motion after the motor has already timed out.
    // Refuse to resume and latch unsafe well before the temporary 1000ms consumer.
    if (heartbeat_->last_sent != Clock::time_point{} &&
        now - heartbeat_->last_sent > std::chrono::milliseconds(700))
        return fail(error(Error::unsafe, "watchdog service overdue; no heartbeat sent"), deadline);
    Frame frame{0x77f, 1, {0x05}};
    if (!transport_.send(frame)) return fail(error(Error::transport, "watchdog heartbeat send failed"), deadline);
    heartbeat_->last_sent = Clock::now();
    return {};
}
Result Motor::service_watchdog(Deadline deadline) {
    if (!heartbeat_->users) return {};
    if (heartbeat_->faulted) {
        if (watchdog_started_)
            return fail(error(Error::unsafe, "shared watchdog fault latched; stop both axes"), deadline);
        return {};
    }
    if (Clock::now() - heartbeat_->last_sent < std::chrono::milliseconds(200)) return {};
    return heartbeat(deadline);
}
Result Motor::start_watchdog(Deadline deadline) {
    if (consumer_overridden_) {
        if (!watchdog_active())
            return fail(error(Error::unsafe, "owned watchdog stopped; reconnect and inspect motor"), deadline);
        return service_watchdog(deadline);
    }
    Result r = sdo_.read(0x1016, 1, 4, previous_consumer_, bounded(deadline));
    if (!r) return fail(r, deadline);
    if ((previous_consumer_ & 0xffffU) && ((previous_consumer_ >> 16) & 0x7fU) != 0x7fU)
        return fail(error(Error::unsafe, "existing watchdog producer is not node 127"), deadline);
    constexpr uint32_t consumer = (0x7fU << 16) | 1000U;
    r = write(0x1016, 1, 4, consumer, deadline);
    if (!r) return r;
    consumer_overridden_ = true;
    if (!(r = verify_value(0x1016, 1, 4, consumer, deadline))) return fail(r, deadline);
    if (!heartbeat_->users) {
        heartbeat_->last_sent = {}; heartbeat_->faulted = false;
        heartbeat_->cancel_requested.store(false);
    }
    if (!heartbeat_registered_) { ++heartbeat_->users; heartbeat_registered_ = true; }
    watchdog_started_ = true;
    return heartbeat(deadline);
}
Result Motor::finish_motion(Deadline deadline) {
    // Called only after confirmed hold AND bit0-confirmed output disable.
    // Stop owned heartbeat before restoring the exact old consumer; never
    // return to normal until the previous value has been read back.
    watchdog_started_ = false;
    if (heartbeat_registered_) {
        if (heartbeat_->users) --heartbeat_->users;
        heartbeat_registered_ = false;
    }
    Result r = sdo_.write(0x1016, 1, 4, previous_consumer_,
                          bounded(std::max(deadline, Clock::now() + std::chrono::milliseconds(150))));
    if (!r) return fail(r, deadline);
    r = verify_value(0x1016, 1, 4, previous_consumer_, deadline);
    if (!r) return fail(r, deadline);
    consumer_overridden_ = false;
    armed_ = false;
    control_attempted_ = false;
    return {};
}
Result Motor::cancel_motion(Deadline deadline) {
    // The cancellation request is fulfilled by the confirmed stop itself. Clear
    // it before hold so that neither the stop nor a later move inherits it.
    cancel_requested_.store(false);
    (void)deadline; // cancellation gets its own bounded confirmed-hold window
    const Deadline stop_until = Clock::now() + kStopActionBudget;
    Result stopped = stop(stop_until);
    if (!stopped) return stopped; // stop already latched and ceased heartbeat on failure
    return error(Error::cancelled, "operator cancelled motion");
}
Result Motor::publish_progress(const ProgressCallback& progress, Snapshot sample) {
    sample.unsafe_latched = unsafe_;
    last_ = sample;
    if (!progress) return {};
    try {
        progress(sample);
    } catch (...) {
        return fail(error(Error::unsafe, "motion progress callback failed"),
                    Clock::now() + std::chrono::milliseconds(150));
    }
    return {};
}
Result Motor::watch_motion(Deadline deadline, int64_t expected_position,
                           const ProgressCallback& progress) {
    Snapshot sample = last_;
    sample.target_position_valid = true;
    sample.target_position_pulses = static_cast<int32_t>(expected_position);
    // commanded_speed_rpm was set at the motion entry point.
    Result r = publish_progress(progress, sample); // pre-trigger actual remains measured
    if (!r) return r;
    auto cancelled = [&]() { return cancel_requested_.load(); };
    auto failed_sample = [&](Result cause, Reading* reading = nullptr) -> Result {
        if (reading) *reading = {};
        Result first = fail(cause, deadline);
        sample.result = first;
        (void)publish_progress(progress, sample); // preserve the original safety/transport cause
        return first;
    };
    auto read = [&](uint16_t object, uint8_t width, uint32_t& value, Reading& reading) -> Result {
        if (cancelled()) return cancel_motion(deadline);
        Result serviced = service_watchdog(deadline);
        if (!serviced) return serviced;
        Result observed = sdo_.read(object, 0, width, value,
                                    bounded(deadline, std::chrono::milliseconds(100)));
        if (observed) reading = {true, Clock::now()};
        return observed;
    };
    while (Clock::now() < deadline) {
        if (cancelled()) {
            r = cancel_motion(deadline);
            sample.result = r;
            (void)publish_progress(progress, sample);
            return r;
        }
        uint32_t status = 0, alarm = 0, speed = 0, position = 0;
        r = read(0x6041, 2, status, sample.status_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.status_sample);
        }
        sample.statusword = load16(status);
        if (status & ((1U << 3) | 0xc000))
            return failed_sample(error((status & 0xc000) ? Error::limit : Error::alarm,
                                       "motor status fault/limit"));
        r = read(0x260e, 2, alarm, sample.alarm_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.alarm_sample);
        }
        sample.alarm_code = load16(alarm);
        if (alarm) return failed_sample(error(Error::alarm, "motor alarm during motion"));
        r = read(0x606c, 2, speed, sample.speed_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.speed_sample);
        }
        sample.speed_rpm = static_cast<int16_t>(speed) / 10.0;
        sample.output_speed_rpm = motor_to_output_rpm(sample.speed_rpm);
        if (std::abs(sample.speed_rpm) > kMaxMotorRpm)
            return failed_sample(error(Error::unsafe, "actual speed exceeds manual maximum"));
        r = read(0x6064, 4, position, sample.position_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.position_sample);
        }
        sample.position_pulses = static_cast<int32_t>(position);
        sample.output_position_degrees = pulses_to_output_degrees(sample.position_pulses);
        if (sample.position_pulses < kMinPositionPulses || sample.position_pulses > kMaxPositionPulses)
            return failed_sample(error(Error::limit, "actual position outside manual motor-position range"));
        sample.result = {};
        if (!(r = publish_progress(progress, sample))) return r;
        if ((status & (1U << 10)) && std::abs(sample.speed_rpm) <= 0.5 &&
            std::abs(static_cast<int64_t>(sample.position_pulses) - expected_position) <= 16) {
            if (cancelled()) {
                r = cancel_motion(deadline);
            } else {
                r = stop(deadline);
            }
            sample.result = r;
            (void)publish_progress(progress, sample);
            return r;
        }
        std::this_thread::sleep_until(std::min(deadline,
            Clock::now() + std::chrono::milliseconds(70)));
    }
    return failed_sample(error(Error::timeout, "motion deadline expired"));
}
Result Motor::move_relative(int32_t pulses, Deadline deadline, uint32_t profile_rpm,
                            const ProgressCallback& progress) {
    const auto started = Clock::now();
    if (!pulses || profile_rpm < 1 || profile_rpm > kMaxMotorRpm || deadline <= started)
        return error(Error::invalid_argument, "relative move requires signed32 nonzero delta, 1..3000 RPM and future deadline");
    deadline = std::min(deadline, started + position_action_budget(pulses, profile_rpm));
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    if (last_.position_pulses < kMinPositionPulses || last_.position_pulses > kMaxPositionPulses)
        return error(Error::invalid_argument, "actual position outside manual motor-position range");
    const int64_t target = static_cast<int64_t>(last_.position_pulses) + pulses;
    if (target < kMinPositionPulses || target > kMaxPositionPulses)
        return error(Error::invalid_argument, "relative target exceeds manual motor-position range");
    if (Clock::now() >= deadline) return error(Error::timeout, "position action deadline expired during safety checks");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 1, deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6081, 0, 4, profile_rpm, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    if (!(r = write(0x607a, 0, 4, static_cast<uint32_t>(pulses), deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x004f, deadline))) return r;
    if (!(r = heartbeat(deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x005f, deadline))) return r;
    last_.commanded_speed_rpm = profile_rpm;
    return watch_motion(deadline, target, progress);
}
Result Motor::move_absolute(int32_t target_pulses, Deadline deadline, uint32_t profile_rpm,
                            const ProgressCallback& progress) {
    const auto started = Clock::now();
    if (deadline <= started || profile_rpm < 1 || profile_rpm > kMaxMotorRpm ||
        target_pulses < kMinPositionPulses || target_pulses > kMaxPositionPulses)
        return error(Error::invalid_argument, "absolute move requires manual-range target, future deadline and 1..3000 RPM");
    deadline = std::min(deadline, started + position_action_budget(
        static_cast<int64_t>(kMaxPositionPulses) * 2, profile_rpm));
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    if (last_.position_pulses < kMinPositionPulses || last_.position_pulses > kMaxPositionPulses)
        return error(Error::invalid_argument, "actual position outside manual motor-position range");
    const int64_t delta = static_cast<int64_t>(target_pulses) - last_.position_pulses;
    if (delta == 0)
        return error(Error::invalid_argument, "absolute target equals fresh actual position");
    deadline = std::min(deadline, started + position_action_budget(delta, profile_rpm));
    if (Clock::now() >= deadline) return error(Error::timeout, "position action deadline expired during safety checks");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 1, deadline))) return r;
    // On this drive 0x002F makes a subsequent 0x607A write execute immediately.
    // Clear any old relative-position cache to current actual while held, so
    // changing mode or setting the controlword cannot replay an old target.
    if (!(r = write(0x607a, 0, 4, static_cast<uint32_t>(last_.position_pulses), deadline))) return r;
    if (!(r = write(0x6081, 0, 4, profile_rpm, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    if (!(r = heartbeat(deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x002f, deadline))) return r;
    if (!(r = write(0x607a, 0, 4, static_cast<uint32_t>(target_pulses), deadline))) return r;
    last_.commanded_speed_rpm = profile_rpm;
    return watch_motion(deadline, target_pulses, progress);
}
// jog 与 spin 共用同一套速度模式时序与看护逻辑，只有时长上限和提示语不同。
Result Motor::run_velocity(int32_t rpm, std::chrono::milliseconds duration, bool continuous,
                           Deadline deadline, const ProgressCallback& progress) {
    const auto started = Clock::now();
    const auto duration_cap = continuous ? std::chrono::seconds(600) : std::chrono::seconds(5);
    auto label = [continuous](Error code, const char* jog_text, const char* spin_text) {
        return error(code, continuous ? spin_text : jog_text);
    };
    if (!rpm || rpm < -kMaxMotorRpm || rpm > kMaxMotorRpm ||
        duration.count() <= 0 || duration > duration_cap || deadline <= started)
        return label(Error::invalid_argument,
                     "jog: abs speed 1..3000 RPM, 1..5000ms, future deadline",
                     "spin: abs speed 1..3000 RPM, 1..600000ms, future deadline");
    deadline = std::min(deadline, started + jog_action_budget(duration));
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    // Velocity mode has no target coordinate. Conservatively reserve full
    // powered travel and the complete 500 RPM/s braking distance before TX.
    const int64_t magnitude = rpm < 0 ? -static_cast<int64_t>(rpm) : rpm;
    const int64_t stopping_ms = (magnitude * 1000 + 499) / 500;
    const int64_t maximum_travel = (magnitude * kMotorPulsesPerRevolution *
        (duration.count() + stopping_ms) + 59999) / 60000;
    const int64_t endpoint = static_cast<int64_t>(last_.position_pulses) +
        (rpm < 0 ? -maximum_travel : maximum_travel);
    if (last_.position_pulses < kMinPositionPulses || last_.position_pulses > kMaxPositionPulses ||
        endpoint < kMinPositionPulses || endpoint > kMaxPositionPulses)
        return label(Error::invalid_argument,
                     "jog travel and stopping distance exceed manual motor-position range",
                     "spin travel and stopping distance exceed manual motor-position range");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 3, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, static_cast<uint32_t>(rpm), deadline))) return r;
    if (!(r = heartbeat(deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x000f, deadline))) return r;
    const Deadline finish = std::min(deadline, Clock::now() + duration);
    Snapshot sample = last_;
    sample.target_position_valid = false;
    sample.commanded_speed_rpm = static_cast<uint32_t>(std::abs(rpm));
    if (!(r = publish_progress(progress, sample))) return r;
    auto failed_sample = [&](Result cause, Reading* reading = nullptr) -> Result {
        if (reading) *reading = {};
        Result first = fail(cause, deadline);
        sample.result = first;
        (void)publish_progress(progress, sample);
        return first;
    };
    auto read = [&](uint16_t object, uint8_t width, uint32_t& value, Reading& reading) -> Result {
        if (cancel_requested_.load()) return cancel_motion(deadline);
        Result serviced = service_watchdog(deadline);
        if (!serviced) return serviced;
        Result observed = sdo_.read(object, 0, width, value,
                                    bounded(deadline, std::chrono::milliseconds(100)));
        if (observed) reading = {true, Clock::now()};
        return observed;
    };
    while (Clock::now() < finish) {
        if (cancel_requested_.load()) {
            r = cancel_motion(deadline);
            sample.result = r;
            (void)publish_progress(progress, sample);
            return r;
        }
        uint32_t status = 0, alarm = 0, speed = 0, position = 0;
        r = read(0x6041, 2, status, sample.status_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.status_sample);
        }
        sample.statusword = load16(status);
        if ((status & (1U << 3)) || (status & 0xc000))
            return failed_sample(label((status & 0xc000) ? Error::limit : Error::alarm,
                                       "jog status fault/limit", "spin status fault/limit"));
        r = read(0x260e, 2, alarm, sample.alarm_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.alarm_sample);
        }
        sample.alarm_code = load16(alarm);
        if (alarm) return failed_sample(label(Error::alarm, "jog alarm", "spin alarm"));
        r = read(0x606c, 2, speed, sample.speed_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.speed_sample);
        }
        sample.speed_rpm = static_cast<int16_t>(speed) / 10.0;
        sample.output_speed_rpm = motor_to_output_rpm(sample.speed_rpm);
        if (std::abs(sample.speed_rpm) > kMaxMotorRpm)
            return failed_sample(label(Error::unsafe, "jog speed exceeds manual maximum",
                                       "spin speed exceeds manual maximum"));
        r = read(0x6064, 4, position, sample.position_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.position_sample);
        }
        sample.position_pulses = static_cast<int32_t>(position);
        sample.output_position_degrees = pulses_to_output_degrees(sample.position_pulses);
        if (sample.position_pulses < kMinPositionPulses || sample.position_pulses > kMaxPositionPulses)
            return failed_sample(label(Error::limit,
                                       "jog actual position exceeds manual motor-position range",
                                       "spin actual position exceeds manual motor-position range"));
        sample.result = {};
        if (!(r = publish_progress(progress, sample))) return r;
        std::this_thread::sleep_until(std::min(finish,
            Clock::now() + std::chrono::milliseconds(70)));
    }
    r = cancel_requested_.load() ? cancel_motion(deadline) : stop(deadline);
    sample.result = r;
    (void)publish_progress(progress, sample);
    return r;
}

// 常转启动：下发速度模式设定点后立即返回，不进入等待循环。
Result Motor::start_velocity(int32_t rpm, Deadline deadline, const ProgressCallback& progress) {
    if (!rpm || rpm < -kMaxMotorRpm || rpm > kMaxMotorRpm || deadline <= Clock::now())
        return error(Error::invalid_argument, "run: abs speed 1..3000 RPM, future deadline");
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    if (sustaining_) return error(Error::unsafe, "a sustained run is already active; stop it first");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    // 常转没有终点，无法预估全程行程；改为要求运行方向上至少留出一分钟行程，
    // 这样即使现场无人看管，也来得及执行 stop。一分钟行程 = 转速(rpm) × 每圈脉冲。
    const int64_t magnitude = rpm < 0 ? -static_cast<int64_t>(rpm) : rpm;
    const int64_t one_minute = magnitude * kMotorPulsesPerRevolution;
    const int64_t remaining = rpm < 0
        ? static_cast<int64_t>(last_.position_pulses) - kMinPositionPulses
        : static_cast<int64_t>(kMaxPositionPulses) - last_.position_pulses;
    if (remaining < one_minute)
        return error(Error::invalid_argument, "run: less than one minute of travel left in that direction");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 3, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, static_cast<uint32_t>(rpm), deadline))) return r;
    if (!(r = heartbeat(deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x000f, deadline))) return r;
    sustaining_ = true;
    sustain_rpm_ = rpm;
    Snapshot sample = last_;
    sample.target_position_valid = false;
    sample.commanded_speed_rpm = static_cast<uint32_t>(magnitude);
    sample.result = {};
    return publish_progress(progress, sample);
}
// 常转看护：与 jog/spin 循环内完全相同的四条检查，读失败即视为失去安全观察。
Result Motor::sustain_service(Deadline deadline, const ProgressCallback& progress) {
    if (!sustaining_) return {};
    Snapshot sample = last_;
    sample.target_position_valid = false;
    sample.commanded_speed_rpm = static_cast<uint32_t>(sustain_rpm_ < 0 ? -sustain_rpm_ : sustain_rpm_);
    auto failed_sample = [&](Result cause, Reading* reading = nullptr) -> Result {
        if (reading) *reading = {};
        Result first = fail(cause, deadline);
        sample.result = first;
        (void)publish_progress(progress, sample);
        return first;
    };
    if (cancel_requested_.load()) {
        Result cancelled = cancel_motion(deadline);
        sample.result = cancelled;
        (void)publish_progress(progress, sample);
        return cancelled;
    }
    uint32_t status = 0, alarm = 0, speed = 0, position = 0;
    Result r = {};
    auto read = [&](uint16_t object, uint8_t width, uint32_t& value, Reading& reading) -> Result {
        Result observed = sdo_.read(object, 0, width, value, bounded(deadline, std::chrono::milliseconds(100)));
        if (observed) reading = {true, sdo_.last_sample_at(), sdo_.last_read_retried()};
        return observed;
    };
    if (!(r = read(0x6041, 2, status, sample.status_sample))) return failed_sample(r, &sample.status_sample);
    sample.statusword = load16(status);
    if ((status & (1U << 3)) || (status & 0xc000))
        return failed_sample(error((status & 0xc000) ? Error::limit : Error::alarm, "run status fault/limit"));
    if (!(r = read(0x260e, 2, alarm, sample.alarm_sample))) return failed_sample(r, &sample.alarm_sample);
    sample.alarm_code = load16(alarm);
    if (alarm) return failed_sample(error(Error::alarm, "run alarm"));
    if (!(r = read(0x606c, 2, speed, sample.speed_sample))) return failed_sample(r, &sample.speed_sample);
    sample.speed_rpm = static_cast<int16_t>(speed) / 10.0;
    sample.output_speed_rpm = motor_to_output_rpm(sample.speed_rpm);
    if (sample.speed_rpm < -kMaxMotorRpm || sample.speed_rpm > kMaxMotorRpm)
        return failed_sample(error(Error::unsafe, "run speed exceeds manual maximum"));
    if (!(r = read(0x6064, 4, position, sample.position_sample))) return failed_sample(r, &sample.position_sample);
    sample.position_pulses = static_cast<int32_t>(position);
    sample.output_position_degrees = pulses_to_output_degrees(sample.position_pulses);
    if (sample.position_pulses < kMinPositionPulses || sample.position_pulses > kMaxPositionPulses)
        return failed_sample(error(Error::limit, "run actual position exceeds manual motor-position range"));
    sample.result = {};
    return publish_progress(progress, sample);
}

Result Motor::jog(int32_t rpm, std::chrono::milliseconds duration, Deadline deadline,
                  const ProgressCallback& progress) {
    return run_velocity(rpm, duration, false, deadline, progress);
}

Result Motor::spin(int32_t rpm, std::chrono::milliseconds duration, Deadline deadline,
                   const ProgressCallback& progress) {
    return run_velocity(rpm, duration, true, deadline, progress);
}
// 双轴同步：预置一次相对位置运动（完成安全检查与全部设定点写入），停在 0x6040=0x004F 不触发。
Result Motor::stage_relative(int32_t pulses, Deadline deadline, uint32_t profile_rpm) {
    const auto started = Clock::now();
    if (!pulses || profile_rpm < 1 || profile_rpm > static_cast<uint32_t>(kMaxMotorRpm) || deadline <= started)
        return error(Error::invalid_argument, "staged move requires signed32 nonzero delta, 1..3000 RPM and future deadline");
    deadline = std::min(deadline, started + position_action_budget(pulses, profile_rpm));
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    if (staged_) return error(Error::unsafe, "a staged move is already pending");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    if (last_.position_pulses < kMinPositionPulses || last_.position_pulses > kMaxPositionPulses)
        return error(Error::invalid_argument, "actual position outside manual motor-position range");
    const int64_t target = static_cast<int64_t>(last_.position_pulses) + pulses;
    if (target < kMinPositionPulses || target > kMaxPositionPulses)
        return error(Error::invalid_argument, "staged target exceeds manual motor-position range");
    if (Clock::now() >= deadline) return error(Error::timeout, "position action deadline expired during safety checks");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 1, deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6081, 0, 4, profile_rpm, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    if (!(r = write(0x607a, 0, 4, static_cast<uint32_t>(pulses), deadline))) return r;
    if (!(r = write(0x6040, 0, 2, 0x004f, deadline))) return r;
    staged_ = true;
    staged_velocity_ = false;
    staged_target_ = target;
    staged_rpm_ = static_cast<int32_t>(profile_rpm);
    last_.commanded_speed_rpm = profile_rpm;
    return {};
}
// 双轴同步：触发已预置的位置运动（0x6040←0x005F）。两轴连续调用即为同步点。
Result Motor::trigger(Deadline deadline) {
    if (!staged_ || staged_velocity_) return error(Error::unsafe, "no staged position move to trigger");
    staged_ = false;
    return write(0x6040, 0, 2, 0x005f, deadline);
}
// 双轴同步：预置速度模式（模式3 + 加速度 + 速度为0），停在启动之前。
Result Motor::stage_velocity(int32_t rpm, std::chrono::milliseconds travel, Deadline deadline) {
    const auto started = Clock::now();
    if (!rpm || rpm < -kMaxMotorRpm || rpm > kMaxMotorRpm || deadline <= started)
        return error(Error::invalid_argument, "staged velocity requires abs speed 1..3000 RPM and future deadline");
    if (node_ == 127) return error(Error::invalid_argument, "node 127 collides with own watchdog producer");
    if (!armed_ || !output_enabled_ || unsafe_) return error(Error::unsafe, "output and explicit arm required");
    if (staged_) return error(Error::unsafe, "a staged move is already pending");
    Result r = require_stationary(deadline, false);
    if (!r) return r;
    // 与 jog/spin 相同的行程保护：预留整段动力行程再加 500 RPM/s 的刹车距离。
    const int64_t magnitude = rpm < 0 ? -static_cast<int64_t>(rpm) : rpm;
    const int64_t stopping_ms = (magnitude * 1000 + 499) / 500;
    const int64_t reserve_ms = static_cast<int64_t>(travel.count()) + stopping_ms;
    const int64_t reserve = (magnitude * kMotorPulsesPerRevolution * reserve_ms + 59999) / 60000;
    const int64_t remaining = rpm < 0
        ? static_cast<int64_t>(last_.position_pulses) - kMinPositionPulses
        : static_cast<int64_t>(kMaxPositionPulses) - last_.position_pulses;
    if (remaining < reserve)
        return error(Error::invalid_argument, "staged velocity travel and stopping distance exceed remaining range");
    if (Clock::now() >= deadline) return error(Error::timeout, "velocity stage deadline expired during safety checks");
    control_attempted_ = true;
    if (!(r = write(0x6040, 0, 2, 0x010f, deadline))) return r;
    if (!(r = start_watchdog(deadline))) return r;
    if (!(r = write(0x60ff, 0, 4, 0, deadline))) return r;
    if (!(r = write(0x6060, 0, 1, 3, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, 500, deadline))) return r;
    staged_ = true;
    staged_velocity_ = true;
    velocity_running_ = false;
    staged_rpm_ = rpm;
    staged_target_ = 0;
    last_.commanded_speed_rpm = static_cast<uint32_t>(magnitude);
    return {};
}
// 双轴同步：触发已预置的速度运动（0x60FF←rpm 后 0x6040←0x000F）。
Result Motor::trigger_velocity(Deadline deadline) {
    if (!staged_ || !staged_velocity_) return error(Error::unsafe, "no staged velocity move to trigger");
    const int32_t rpm = staged_rpm_;
    staged_ = false;
    Result r = write(0x60ff, 0, 4, static_cast<uint32_t>(rpm), deadline);
    if (!r) return r;
    velocity_triggered_at_ = sdo_.last_transmitted_at();
    if (!(r = heartbeat(deadline))) return r;
    r = write(0x6040, 0, 2, 0x000f, deadline);
    velocity_running_ = static_cast<bool>(r);
    return r;
}
Result Motor::trigger_velocity_pair(Motor& other, Deadline deadline) {
    if (&other==this || heartbeat_!=other.heartbeat_ || !staged_ || !staged_velocity_ ||
        !other.staged_ || !other.staged_velocity_ || unsafe_ || other.unsafe_)
        return error(Error::unsafe,"both axes must stage velocity on one shared bus before paired trigger");
    Result r=service_watchdog(deadline);
    if(!r) return r;
    staged_=other.staged_=false;
    const auto before_a=sdo_.last_transmitted_at(), before_b=other.sdo_.last_transmitted_at();
    r=sdo_.write_pair(other.sdo_,0x60ff,4,static_cast<uint32_t>(staged_rpm_),
                      static_cast<uint32_t>(other.staged_rpm_),deadline);
    if(!r) {
        if(r.error==Error::cancelled && before_a==sdo_.last_transmitted_at() &&
            before_b==other.sdo_.last_transmitted_at()) return r;
        (void)other.fail(r,deadline);
        return fail(r,deadline);
    }
    velocity_triggered_at_=sdo_.last_transmitted_at();
    other.velocity_triggered_at_=other.sdo_.last_transmitted_at();
    r=sdo_.write_pair(other.sdo_,0x6040,2,0x000f,0x000f,deadline);
    if(!r) { (void)other.fail(r,deadline); return fail(r,deadline); }
    velocity_running_=other.velocity_running_=true;
    return {};
}
Result Motor::update_velocity(int32_t rpm, Deadline deadline) {
    if (!velocity_running_ || !output_enabled_ || unsafe_ || !rpm ||
        (rpm > 0) != (staged_rpm_ > 0) || std::abs(rpm) > std::abs(staged_rpm_))
        return error(Error::unsafe, "velocity trim must keep owned direction and nominal speed bound");
    Result r = write(0x60ff, 0, 4, static_cast<uint32_t>(rpm), deadline);
    if (r) last_.commanded_speed_rpm = static_cast<uint32_t>(std::abs(rpm));
    return r;
}
// 双轴同步：放弃已预置但未触发的运动，回到停止并清掉缓存。
Result Motor::abort_staged(Deadline deadline) {
    if (!staged_) return {};
    const bool was_velocity = staged_velocity_;
    staged_ = false;
    // 先回到 0x010F（去掉 0x004F 的执行位），缓存就失效了；再清缓存属于二次保险。
    Result r = write(0x6040, 0, 2, 0x010f, deadline);
    if (!r) return r;
    if (was_velocity) return write(0x60ff, 0, 4, 0, deadline);
    return write(0x607a, 0, 4, static_cast<uint32_t>(last_.position_pulses), deadline);
}
// 双轴同步：只发一帧 0x6040←0x0100 急停，不做确认；调用方须随后各自 stop() 确认。
Result Motor::halt_unconfirmed(Deadline deadline) {
    clear_motion_deadline();
    velocity_running_ = false;
    (void)deadline;
    if (!control_attempted_ && !output_enabled_) return {};
    control_attempted_ = true;
    operation_clear_attempted_ = true;
    staged_ = false;
    Frame emergency{static_cast<uint16_t>(0x600 + node_), 8, {0x2b, 0x40, 0x60, 0, 0x00, 0x01}};
    if (!transport_.send(emergency)) return error(Error::transport, "emergency hold send failed");
    return {};
}
// 双轴监视：只读状态字/报警/速度/位置四项，并做与运动循环相同的安全检查。
Result Motor::poll_safety(Snapshot& sample, Deadline deadline) {
    sample = last_;
    auto failed_sample = [&](Result cause, Reading* reading = nullptr) -> Result {
        if (reading) *reading = {};
        if (cause.error == Error::cancelled && coordinated_motion_) {
            sample.result = cause;
            return cause; // reaching the requested end time is not an unsafe latch
        }
        Result first = fail(cause, deadline);
        sample.result = first;
        return first;
    };
    uint32_t status = 0, alarm = 0, speed = 0, position = 0;
    Result r = {};
    auto read = [&](uint16_t object, uint8_t width, uint32_t& value, Reading& reading) -> Result {
        Result serviced = service_watchdog(deadline);
        if (!serviced) return serviced;
        Result observed = sdo_.read(object, 0, width, value, bounded(deadline, std::chrono::milliseconds(100)));
        if (observed) reading = {true, sdo_.last_sample_at(), sdo_.last_read_retried()};
        return observed;
    };
    if (!(r = read(0x6041, 2, status, sample.status_sample))) return failed_sample(r, &sample.status_sample);
    sample.statusword = load16(status);
    if ((status & (1U << 3)) || (status & 0xc000))
        return failed_sample(error((status & 0xc000) ? Error::limit : Error::alarm, "sync status fault/limit"));
    if (!(r = read(0x260e, 2, alarm, sample.alarm_sample))) return failed_sample(r, &sample.alarm_sample);
    sample.alarm_code = load16(alarm);
    if (alarm) return failed_sample(error(Error::alarm, "sync alarm"));
    if (!(r = read(0x606c, 2, speed, sample.speed_sample))) return failed_sample(r, &sample.speed_sample);
    sample.speed_rpm = static_cast<int16_t>(speed) / 10.0;
    sample.output_speed_rpm = motor_to_output_rpm(sample.speed_rpm);
    if (sample.speed_rpm < -kMaxMotorRpm || sample.speed_rpm > kMaxMotorRpm)
        return failed_sample(error(Error::unsafe, "sync speed exceeds manual maximum"));
    if (!(r = read(0x6064, 4, position, sample.position_sample))) return failed_sample(r, &sample.position_sample);
    sample.position_pulses = static_cast<int32_t>(position);
    sample.output_position_degrees = pulses_to_output_degrees(sample.position_pulses);
    if (sample.position_pulses < kMinPositionPulses || sample.position_pulses > kMaxPositionPulses)
        return failed_sample(error(Error::limit, "sync actual position exceeds manual motor-position range"));
    sample.result = {};
    last_ = sample;
    return {};
}
Result Motor::stage_node_id(uint8_t new_node, Deadline deadline) {
    if (new_node < 1 || new_node > 126 || new_node == node_)
        return error(Error::invalid_argument, "new node must change to 1..126; 127 owns watchdog");
    Result r = require_stationary(deadline, true, true);
    if (!r) return r;
    if (!transport_.accept_sdo_replies(node_, new_node))
        return error(Error::transport, "cannot install old/new SDO reply filters; node was not written");
    sdo_.accept_reply_from(new_node);
    struct Restore {
        Transport& transport; SdoClient& sdo; uint8_t node;
        ~Restore() { sdo.accept_reply_from(0); (void)transport.accept_sdo_replies(node, 0); }
    } restore{transport_, sdo_, node_};
    if (!(r = write(0x2615, 0, 2, new_node, deadline))) return r;
    if (!(r = verify_value(0x2615, 0, 2, new_node, deadline))) return fail(r, deadline);
    if (!(r = write(0x2614, 0, 2, 1, deadline))) return r;
    while (Clock::now() < deadline) {
        uint32_t saved = 0;
        r = sdo_.read(0x2614, 0, 2, saved, bounded(deadline));
        if (r && saved == 2)
            return {Error::none, 0, "node staged and saved; power-cycle/readback pending"};
        if (!r && r.error != Error::timeout) return fail(r, deadline);
        std::this_thread::sleep_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(100)));
    }
    return fail(error(Error::timeout, "node save did not report completion"), deadline);
}
Result Motor::stage_bitrate(uint32_t bitrate, Deadline deadline) {
    uint32_t code = 0;
    switch (bitrate) {
        case 125000: code = 800; break;
        case 250000: code = 801; break;
        case 500000: code = 802; break;
        case 1000000: code = 803; break;
        default: return error(Error::invalid_argument, "bitrate must be 125/250/500/1000 kbit/s");
    }
    Result r = require_stationary(deadline, true);
    if (!r) return r;
    if (!(r = write(0x2600, 0, 2, 1, deadline))) return r;
    if (!(r = write(0x6083, 0, 4, code, deadline))) return r;
    if (!(r = write(0x2604, 0, 2, 129, deadline))) return r;
    r = write(0x2600, 0, 2, 506, deadline);
    return r ? Result{Error::none, 0, "bitrate staged by ACK only; power-cycle/readback pending"} : r;
}
} // namespace yz_aim
