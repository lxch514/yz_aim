#pragma once
// YZ-AIM CANopen 驱动接口：传输层抽象、SDO 客户端、电机安全门禁与运动控制。

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
// 双轴同步：两轴位置偏差的默认阈值（输出轴角度），以及同步运动的速度硬上限（电机 RPM）。
constexpr double kSyncDefaultToleranceDeg = 0.5;
constexpr int32_t kSyncMaxRpm = 300;
// 双轴同步：速度模式每拍允许的两轴稳态速度差（电机 RPM）。两轴各走自己的速度环，稳态下
// 必然存在微小速度差，位置之和会以恒定速率漂移；门限由"固定阈值 + 这个速度差 × 自上次
// 重取基准以来的时长"给出，0 表示不放宽（门限退化为固定的 sync-tol）。
// 缓慢漂移每拍都会被重取基准吸收，所以门限不随总运行时间变松，跑多久都锁在 sync-tol 附近。
constexpr double kSyncDefaultDriftRpm = 10.0;
// 双轴同步：丝杆导程（输出轴转一整圈的位移，mm/圈），供 sync-mm 把毫米换算成行程。
// 0 = 未设置 —— 导程错了会走错距离，所以不留隐式默认值，未设置时 sync-mm 直接拒绝执行。
constexpr double kSyncDefaultLeadMm = 0.0;
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

class SdoClient {
public:
    SdoClient(Transport& transport, uint8_t node);
    Result read(uint16_t index, uint8_t subindex, uint8_t width, uint32_t& value, Deadline deadline);
    // One frame only, never automatically retries; return codes distinguish timeout and abort.
    Result write(uint16_t index, uint8_t subindex, uint8_t width, uint32_t value, Deadline deadline);
private:
    Result exchange(const Frame& request, uint16_t index, uint8_t subindex,
                    uint8_t width, uint32_t* value, Deadline deadline);
    Transport& transport_;
    uint8_t node_;
    bool poisoned_ = false;
};

// A Motor owns no transport. Caller keeps Transport alive until Motor is destroyed.
// Construction and destruction send no writes/heartbeat/NMT. An unowned snapshot()
// stays read-only; once control is owned it may service the watchdog, and a failed
// safety read may send a single unconfirmed emergency hold.
class Motor {
public:
    Motor(Transport& transport, uint8_t node = 1);
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
    // 速度模式连续运转：与 jog 共用同一套时序与看护，只把时长上限放到 10 分钟。
    Result spin(int32_t rpm, std::chrono::milliseconds duration, Deadline deadline,
                const ProgressCallback& progress = {}); // abs(rpm)<=3000, 1..600s
    // 常转：下发速度模式后立即返回，由调用方周期调用 sustain_service 维持看护，直到 stop。
    Result start_velocity(int32_t rpm, Deadline deadline,
                          const ProgressCallback& progress = {}); // abs(rpm)<=3000, 运行方向上须留足 1 分钟行程
    // 常转看护：维持心跳并做一次状态/报警/速度/位置读检查；返回非 none 表示已判不安全并停发心跳。
    Result sustain_service(Deadline deadline, const ProgressCallback& progress = {});
    // 双轴同步：预置一次相对位置运动（完成安全检查与全部设定点写入），停在 0x6040=0x004F 不触发。
    // 两轴都预置成功后连续调用 trigger 即为同步点；放弃时调用 abort_staged。
    Result stage_relative(int32_t pulses, Deadline deadline,
                          uint32_t profile_rpm = kPositionProfileRpm);
    // 双轴同步：触发已预置的位置运动（0x6040←0x005F）。
    Result trigger(Deadline deadline);
    // 双轴同步：预置速度模式（模式3 + 加速度 + 速度为0），停在启动之前。
    // travel 为本次要预留的动力行程时长（run 传 60 s，spin 传设定时长），另加 500 RPM/s 的刹车距离。
    Result stage_velocity(int32_t rpm, std::chrono::milliseconds travel, Deadline deadline);
    // 双轴同步：触发已预置的速度运动（0x60FF←rpm 后 0x6040←0x000F）。
    Result trigger_velocity(Deadline deadline);
    // 双轴同步：放弃已预置但未触发的运动，回到停止并清掉位置缓存。
    Result abort_staged(Deadline deadline);
    // 双轴同步：只发一帧 0x6040←0x0100 急停，不做确认；调用方须随后各自 stop() 确认。
    Result halt_unconfirmed(Deadline deadline);
    // 双轴监视：只读状态字/报警/速度/位置四项，并做与运动循环相同的安全检查。
    Result poll_safety(Snapshot& sample, Deadline deadline);
    bool staged() const { return staged_; }
    bool staged_stationary() const { return staged_ && !staged_velocity_; }
    int64_t staged_target_pulses() const { return staged_target_; }
    // No implicit save/reset/reconnect; intended as setup operations only while disabled/stationary.
    Result stage_node_id(uint8_t new_node, Deadline deadline); // 0x2615 then 0x2614=1, poll ==2
    Result stage_bitrate(uint32_t bitrate, Deadline deadline); // 125/250/500/1000 kbit/s; reboot
    bool unsafe_latched() const { return unsafe_; }
    bool armed() const { return armed_; }
    bool sustaining() const { return sustaining_; }
    int32_t sustained_rpm() const { return sustain_rpm_; }
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
    Result run_velocity(int32_t rpm, std::chrono::milliseconds duration, bool continuous,
                        Deadline deadline, const ProgressCallback& progress);
    Result cancel_motion(Deadline deadline);
    Result publish_progress(const ProgressCallback& progress, Snapshot sample);
    Transport& transport_;
    SdoClient sdo_;
    uint8_t node_;
    bool armed_ = false, unsafe_ = false, watchdog_started_ = false, output_enabled_ = false;
    bool control_attempted_ = false, consumer_overridden_ = false;
    bool sustaining_ = false; int32_t sustain_rpm_ = 0;
    // 双轴同步：已预置但尚未触发的运动。staged_velocity_ 区分位置/速度两种预置。
    bool staged_ = false, staged_velocity_ = false;
    int64_t staged_target_ = 0; int32_t staged_rpm_ = 0;
    bool operation_clear_attempted_ = false; // 0x0100 phase: emergency must not reassert enable bits
    uint32_t previous_consumer_ = 0;
    std::atomic<bool> cancel_requested_{false};
    Clock::time_point last_heartbeat_{};
    Snapshot last_{};
};

} // namespace yz_aim
