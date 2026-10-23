#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <string>

namespace yz_aim {

using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;

struct Frame {
    uint16_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

// All calls are synchronous; receive returns false at deadline and must never exceed it.
// Transport never emits CAN on construction/destruction. Frames are classic 11-bit CAN.
class Transport {
public:
    virtual ~Transport() = default;
    virtual bool send(const Frame& frame) = 0;
    virtual bool receive(Frame& frame, Deadline deadline) = 0;
    virtual bool failed() const { return false; } // transport fault, distinct from receive timeout
};

enum class Error {
    none, invalid_argument, transport, timeout, malformed_reply, abort_reply,
    unsafe, stale, alarm, limit, moving, cancelled
};

struct Result {
    Error error = Error::none;
    uint32_t abort_code = 0;
    std::string message;
    std::string context{}; // optional diagnostic detail; message remains stable for translations
    explicit operator bool() const { return error == Error::none; }
};

// User-specified 51 motor revolutions per output revolution; OD values remain motor-side.
constexpr int32_t kMotorPulsesPerRevolution = 32768;
constexpr int32_t kMotorRevolutionsPerOutputRevolution = 51;
constexpr int64_t kOutputPulsesPerRevolution =
    static_cast<int64_t>(kMotorPulsesPerRevolution) * kMotorRevolutionsPerOutputRevolution;
// Manual: retained motor position is -60000..60000 motor turns. The SDO
// relative target is signed32, while an absolute traverse may cross both ends.
constexpr int32_t kMaxMotorRpm = 3000;
constexpr int32_t kMaxPositionPulses = 60000 * kMotorPulsesPerRevolution;
constexpr int32_t kMinPositionPulses = -kMaxPositionPulses;
constexpr int32_t kMaxRelativePositionPulses = std::numeric_limits<int32_t>::max();
constexpr int32_t kPositionProfileRpm = 30;
constexpr std::chrono::milliseconds kStopActionBudget{10000};
// Five seconds for setup and settling, plus ramp-up and ramp-down at the
// commanded 500 RPM/s. Clamp before negating so INT64_MIN remains harmless.
constexpr std::chrono::milliseconds position_action_budget(
    int64_t delta_pulses, uint32_t rpm = kPositionProfileRpm) noexcept {
    constexpr int64_t max_distance = static_cast<int64_t>(kMaxPositionPulses) * 2;
    const int64_t distance = delta_pulses < -max_distance || delta_pulses > max_distance
        ? max_distance : delta_pulses < 0 ? -delta_pulses : delta_pulses;
    const int64_t selected_rpm = rpm < 1 || rpm > kMaxMotorRpm ? 1 : rpm;
    const int64_t pulses_per_minute = selected_rpm * kMotorPulsesPerRevolution;
    const int64_t ramp_ms = (selected_rpm * 1000 + 499) / 500;
    return std::chrono::milliseconds(5000 + 2 * ramp_ms +
        (distance * 60000 + pulses_per_minute - 1) / pulses_per_minute);
}
// Jog is limited to five seconds of powered movement; reserve independent
// setup and confirmed hold time even at maximum speed.
constexpr std::chrono::milliseconds jog_action_budget(std::chrono::milliseconds duration) noexcept {
    return std::chrono::milliseconds(5000) + duration + kStopActionBudget;
}
constexpr double motor_to_output_rpm(double motor_rpm) noexcept {
    return motor_rpm / kMotorRevolutionsPerOutputRevolution;
}
constexpr double pulses_to_output_degrees(int32_t pulses) noexcept {
    return static_cast<double>(pulses) * 360.0 / kOutputPulsesPerRevolution;
}
Result output_revolutions_to_pulses(double revolutions, int32_t& raw_pulses);

// Vendor: 0x2601 readback bit 0 is output enable. Keep all other bits raw;
// their meanings are unspecified and must never be written back as commands.
constexpr bool output_is_enabled(uint16_t raw) noexcept { return (raw & 1U) != 0; }

struct Reading {
    bool valid = false;
    Clock::time_point sampled_at{};
};

struct Snapshot {
    Result result;
    Reading identity_sample, status_sample, position_sample, speed_sample, current_sample;
    Reading voltage_sample, temperature_sample, battery_sample, alarm_sample, output_sample;
    Result current_result, voltage_result, temperature_result, battery_result;
    uint32_t device_type = 0, vendor_id = 0, product_code = 0;
    uint16_t statusword = 0;
    uint16_t alarm_code = 0;
    uint16_t output_enabled = 0; // full 0x2601 readback; interpret bit 0 via output_is_enabled()
    int32_t position_pulses = 0;
    double speed_rpm = 0, current_a = 0, voltage_v = 0;
    double output_speed_rpm = 0, output_position_degrees = 0;
    uint16_t temperature_c = 0, battery_tenths = 0;
    bool unsafe_latched = false;
    // Command metadata only: neither field is a measured OD readback.
    bool target_position_valid = false;
    int32_t target_position_pulses = 0;
    uint32_t commanded_speed_rpm = 0;
};
// Called on the Motor worker, without an additional SDO read or thread.
// Consumers copy the data; they must not call Motor or Transport from inside it.
using ProgressCallback = std::function<void(const Snapshot&)>;

// Additional per-reply limits. Zero preserves the caller's legacy budget.
// Positive limits start after send() returns and never extend the caller deadline.
struct SdoReplyTimeouts {
    std::chrono::milliseconds read{0};
    std::chrono::milliseconds write{0};
};

class SdoClient {
public:
    SdoClient(Transport& transport, uint8_t node, SdoReplyTimeouts timeouts = {});
    Result read(uint16_t index, uint8_t subindex, uint8_t width, uint32_t& value, Deadline deadline);
    // One frame only, never automatically retries; return codes distinguish timeout and abort.
    Result write(uint16_t index, uint8_t subindex, uint8_t width, uint32_t value, Deadline deadline);
private:
    Result exchange(const Frame& request, uint16_t index, uint8_t subindex,
                    uint8_t width, uint32_t* value, Deadline deadline);
    Transport& transport_;
    uint8_t node_;
    bool poisoned_ = false;
    SdoReplyTimeouts timeouts_;
};

// A Motor owns no transport. Caller keeps Transport alive until Motor is destroyed.
// Construction and destruction send no writes/heartbeat/NMT. An unowned snapshot()
// stays read-only; once control is owned it may service the watchdog, and a failed
// safety read may send a single unconfirmed emergency hold.
class Motor {
public:
    Motor(Transport& transport, uint8_t node = 1, SdoReplyTimeouts timeouts = {});
    ~Motor();
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    Snapshot snapshot(Deadline deadline);
    Result arm(Deadline deadline); // fresh alarm-free, zero-speed, no-limit observation required
    void request_cancel() noexcept; // thread-safe request; active motion performs hold on its own thread
    // Call on the same Motor worker every ~200 ms while true, including modal prompts.
    // Opening/reading/arming without owned output sends no heartbeat.
    bool watchdog_active() const noexcept { return watchdog_started_ && !unsafe_; }
    Result service_watchdog(Deadline deadline);
    Result disarm(Deadline deadline); // armed-only exits silently; controlled motor holds
    Result stop(Deadline deadline); // explicit hold; always available even after a fault
    Result enable_output(Deadline deadline); // explicit operator intent, clears stale setpoints first
    Result disable_output(Deadline deadline);
    Result reset_alarm(Deadline deadline); // gated; disabled-output reset semantics unverified
    Result set_profile_speed(uint32_t rpm, Deadline deadline); // 0..3000 RPM, volatile
    Result set_acceleration(uint32_t rpm_per_second, Deadline deadline); // 10..1000; see docs
    Result set_current_limit(uint16_t milliamps, Deadline deadline); // 10..10000, multiple of 10
    Result move_relative(int32_t pulses, Deadline deadline,
                         uint32_t profile_rpm = kPositionProfileRpm,
                         const ProgressCallback& progress = {}); // signed32 delta; resulting position within manual limits
    Result move_absolute(int32_t target_pulses, Deadline deadline,
                         uint32_t profile_rpm = kPositionProfileRpm,
                         const ProgressCallback& progress = {}); // target within manual limits, fresh actual
    Result jog(int32_t rpm, std::chrono::milliseconds duration, Deadline deadline,
               const ProgressCallback& progress = {}); // abs(rpm)<=3000, <=5s
    // No implicit save/reset/reconnect; intended as setup operations only while disabled/stationary.
    Result stage_node_id(uint8_t new_node, Deadline deadline); // 0x2615 then 0x2614=1, poll ==2
    Result stage_bitrate(uint32_t bitrate, Deadline deadline); // 125/250/500/1000 kbit/s; reboot
    bool unsafe_latched() const { return unsafe_; }
    bool armed() const { return armed_; }
    bool cleanup_required() const { return output_enabled_ || control_attempted_ || consumer_overridden_; }
    uint8_t node() const { return node_; }
private:
    Result require_stationary(Deadline deadline, bool require_disabled, bool allow_communication_alarm = false);
    Result write(uint16_t index, uint8_t subindex, uint8_t width, uint32_t value, Deadline deadline);
    Result hold(Deadline deadline);
    Result verify_stopped(Deadline deadline, bool allow_communication_alarm = false);
    Result verify_value(uint16_t index, uint8_t subindex, uint8_t width, uint32_t expected, Deadline deadline);
    Result verify_output(bool expected_enabled, Deadline deadline);
    Result fail(Result result, Deadline deadline);
    Result start_watchdog(Deadline deadline);
    Result finish_motion(Deadline deadline);
    Result heartbeat(Deadline deadline);
    Result watch_motion(Deadline deadline, int64_t expected_position, const ProgressCallback& progress);
    Result cancel_motion(Deadline deadline);
    Result publish_progress(const ProgressCallback& progress, Snapshot sample);
    Transport& transport_;
    SdoClient sdo_;
    uint8_t node_;
    bool armed_ = false, unsafe_ = false, watchdog_started_ = false, output_enabled_ = false;
    bool control_attempted_ = false, consumer_overridden_ = false;
    bool operation_clear_attempted_ = false; // 0x0100 phase: emergency must not reassert enable bits
    uint32_t previous_consumer_ = 0;
    std::atomic<bool> cancel_requested_{false};
    Clock::time_point last_heartbeat_{};
    Snapshot last_{};
};

// LYS USBCANFD-Mini adapter; runtime-load liblyscan.so from explicit library_path.
// No vendor library is linked into Controller. Opening does not transmit any frame.
class FrameTrace;
std::unique_ptr<Transport> open_lys_can(const std::string& library_path,
                                        uint32_t bitrate, std::string& error,
                                        uint32_t device = 0, uint32_t channel = 0,
                                        uint32_t nominal_timing = 0, FrameTrace* trace = nullptr);
} // namespace yz_aim
