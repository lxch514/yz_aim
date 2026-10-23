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

SdoClient::SdoClient(Transport& transport, uint8_t node, SdoReplyTimeouts timeouts)
    : transport_(transport), node_(node), timeouts_(timeouts) {
    if (timeouts.read.count() < 0 || timeouts.write.count() < 0)
        throw std::invalid_argument("SDO reply timeouts must be nonnegative");
}

Result SdoClient::read(uint16_t index, uint8_t subindex, uint8_t width,
                       uint32_t& value, Deadline deadline) {
    if (width != 1 && width != 2 && width != 4)
        return sdo_failure(error(Error::invalid_argument, "SDO width must be 1, 2 or 4"),
                           true, index, subindex, "prequeue", Clock::now(), deadline);
    Frame frame{static_cast<uint16_t>(0x600 + node_), 8, {0x40,
        static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), subindex}};
    return exchange(frame, index, subindex, width, &value, deadline);
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
Result SdoClient::exchange(const Frame& request, uint16_t index, uint8_t subindex,
                           uint8_t width, uint32_t* value, Deadline deadline) {
    const auto started = Clock::now();
    const bool is_read = value != nullptr;
    const auto reply_limit = is_read ? timeouts_.read : timeouts_.write;
    auto failed = [&](Result r, const char* phase) {
        r = sdo_failure(std::move(r), is_read, index, subindex, phase, started, deadline);
        if (reply_limit.count() > 0)
            r.context += " reply_timeout_ms=" + std::to_string(reply_limit.count());
        return r;
    };
    if (node_ < 1 || node_ > 127) return failed(error(Error::invalid_argument, "invalid CANopen node"), "prequeue");
    if (poisoned_) return failed(error(Error::unsafe, "ambiguous SDO session; reconnect transport"), "prequeue");
    // Drain all queued frames, including any stale ACK buried behind periodic PDOs.
    // At the observed ~20 PDO/s a 30-second confirmation can queue hundreds of
    // frames. Both time and frame bounds prevent a continuously flooded bus from
    // delaying STOP indefinitely; never transmit if the queue is not empty.
    // A delayed ACK arriving after an empty queue remains intrinsically ambiguous.
    constexpr unsigned kMaximumPrequeueFrames = 4096;
    const Deadline drain_until = std::min(deadline, Clock::now() + std::chrono::milliseconds(200));
    bool drained_to_empty = false;
    for (unsigned drained = 0; drained < kMaximumPrequeueFrames && Clock::now() < drain_until; ++drained) {
        Frame queued;
        if (!transport_.receive(queued, std::min(drain_until, Clock::now() + std::chrono::milliseconds(1)))) {
            if (transport_.failed()) { poisoned_ = true; return failed(error(Error::transport, "receive failed during prequeue drain"), "prequeue"); }
            drained_to_empty = Clock::now() < drain_until;
            break;
        }
    }
    if (!drained_to_empty) {
        poisoned_ = true;
        return failed(error(Error::unsafe, "pre-transaction receive queue did not drain within bounded budget"), "prequeue");
    }
    if (Clock::now() >= deadline) return failed(error(Error::timeout, "SDO deadline expired before send"), "send");
    if (!transport_.send(request)) { poisoned_ = true; return failed(error(Error::transport, "CAN send failed"), "send"); }
    // Keep prequeue/SDK-send time separate from the configured reply window.
    // This extra cap never extends the enclosing action/transaction deadline.
    if (reply_limit.count() > 0)
        deadline = std::min(deadline, Clock::now() + reply_limit);
    // Ignore unrelated traffic, including other SDO objects, with both time and frame bounds.
    for (unsigned seen = 0; seen < 128 && Clock::now() < deadline; ++seen) {
        Frame reply;
        if (!transport_.receive(reply, bounded(deadline))) {
            if (transport_.failed()) { poisoned_ = true; return failed(error(Error::transport, "CAN receive failed"), "reply"); }
            if (Clock::now() < deadline) continue;
            break;
        }
        if (reply.id != 0x580 + node_) continue;
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
    poisoned_ = true;
    return failed(error(Error::timeout, "SDO reply timed out or receive limit exhausted; reconnect"), "reply");
}

Motor::Motor(Transport& transport, uint8_t node, SdoReplyTimeouts timeouts)
    : transport_(transport), sdo_(transport, node, timeouts), node_(node) {
    if (node < 1 || node > 127) throw std::invalid_argument("YZ-AIM node must be 1..127");
}
Motor::~Motor() = default;
void Motor::request_cancel() noexcept { cancel_requested_.store(true); }

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
    return result;
}
Result Motor::write(uint16_t index, uint8_t subindex, uint8_t width,
                    uint32_t value, Deadline deadline) {
    if (cancel_requested_.load() && control_attempted_) return cancel_motion(deadline);
    if (watchdog_started_ && Clock::now() - last_heartbeat_ >= std::chrono::milliseconds(200)) {
        Result heart = heartbeat(deadline);
        if (!heart) return heart;
    }
    Result r = sdo_.write(index, subindex, width, value, bounded(deadline));
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
        unsafe_ = true; watchdog_started_ = false;
        const uint8_t low = operation_clear_attempted_ ? 0x00 : 0x0f;
        Frame emergency{static_cast<uint16_t>(0x600 + node_), 8, {0x2b, 0x40, 0x60, 0, low, 0x01}};
        (void)transport_.send(emergency); // unconfirmed; SDO session may be poisoned
        return r;
    }
    Result observed = verify_stopped(deadline, true);
    if (!observed) return fail(observed, deadline);
    cancel_requested_.store(false); // verified hold fulfils any cancellation request
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
    if (!watchdog_started_ || unsafe_) return {};
    const auto now = Clock::now();
    if (now >= deadline) return fail(error(Error::timeout, "watchdog heartbeat deadline expired"), deadline);
    // A late heartbeat may resume motion after the motor has already timed out.
    // Refuse to resume and latch unsafe well before the temporary 1000ms consumer.
    if (last_heartbeat_ != Clock::time_point{} &&
        now - last_heartbeat_ > std::chrono::milliseconds(700))
        return fail(error(Error::unsafe, "watchdog service overdue; no heartbeat sent"), deadline);
    Frame frame{0x77f, 1, {0x05}};
    if (!transport_.send(frame)) return fail(error(Error::transport, "watchdog heartbeat send failed"), deadline);
    last_heartbeat_ = Clock::now();
    return {};
}
Result Motor::service_watchdog(Deadline deadline) {
    if (!watchdog_active()) return {};
    if (Clock::now() - last_heartbeat_ < std::chrono::milliseconds(200)) return {};
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
    last_heartbeat_ = {};
    watchdog_started_ = true;
    return heartbeat(deadline);
}
Result Motor::finish_motion(Deadline deadline) {
    // Called only after confirmed hold AND bit0-confirmed output disable.
    // Stop owned heartbeat before restoring the exact old consumer; never
    // return to normal until the previous value has been read back.
    watchdog_started_ = false;
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
Result Motor::jog(int32_t rpm, std::chrono::milliseconds duration, Deadline deadline,
                  const ProgressCallback& progress) {
    const auto started = Clock::now();
    if (!rpm || rpm < -kMaxMotorRpm || rpm > kMaxMotorRpm ||
        duration.count() <= 0 || duration > std::chrono::seconds(5) || deadline <= started)
        return error(Error::invalid_argument, "jog: abs speed 1..3000 RPM, 1..5000ms, future deadline");
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
        return error(Error::invalid_argument, "jog travel and stopping distance exceed manual motor-position range");
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
            return failed_sample(error((status & 0xc000) ? Error::limit : Error::alarm,
                                       "jog status fault/limit"));
        r = read(0x260e, 2, alarm, sample.alarm_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.alarm_sample);
        }
        sample.alarm_code = load16(alarm);
        if (alarm) return failed_sample(error(Error::alarm, "jog alarm"));
        r = read(0x606c, 2, speed, sample.speed_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.speed_sample);
        }
        sample.speed_rpm = static_cast<int16_t>(speed) / 10.0;
        sample.output_speed_rpm = motor_to_output_rpm(sample.speed_rpm);
        if (std::abs(sample.speed_rpm) > kMaxMotorRpm)
            return failed_sample(error(Error::unsafe, "jog speed exceeds manual maximum"));
        r = read(0x6064, 4, position, sample.position_sample);
        if (!r) {
            if (r.error == Error::cancelled) { sample.result = r; (void)publish_progress(progress, sample); return r; }
            return failed_sample(r, &sample.position_sample);
        }
        sample.position_pulses = static_cast<int32_t>(position);
        sample.output_position_degrees = pulses_to_output_degrees(sample.position_pulses);
        if (sample.position_pulses < kMinPositionPulses || sample.position_pulses > kMaxPositionPulses)
            return failed_sample(error(Error::limit, "jog actual position exceeds manual motor-position range"));
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
Result Motor::stage_node_id(uint8_t new_node, Deadline deadline) {
    if (new_node < 1 || new_node > 127 || new_node == node_) return error(Error::invalid_argument, "node must change to 1..127");
    Result r = require_stationary(deadline, true);
    if (!r) return r;
    if (!(r = write(0x2615, 0, 2, new_node, deadline))) return r;
    if (!(r = verify_value(0x2615, 0, 2, new_node, deadline))) return fail(r, deadline);
    if (!(r = write(0x2614, 0, 2, 1, deadline))) return r;
    for (unsigned i = 0; i < 5 && Clock::now() < deadline; ++i) {
        uint32_t saved = 0;
        r = sdo_.read(0x2614, 0, 2, saved, bounded(deadline));
        if (!r) return fail(r, deadline);
        if (saved == 2) return {Error::none, 0, "node staged and saved; power-cycle/readback pending"};
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
