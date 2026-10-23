#include "yz_aim_tui.h"
#include "maintenance_tui_session.h"
#include "maintenance_tui_input.h"
#include "yz_aim_cli.h"
#include "yz_aim_mock.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <clocale>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <functional>
#include <mutex>
#include <map>
#include <optional>
#include <thread>
#include <utility>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace yz_aim::tool {
namespace {
std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running.store(false);
}

void install_signals()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}

std::string hex32(uint32_t value)
{
    char text[12];
    std::snprintf(text, sizeof(text), "0x%08X", value);
    return text;
}

std::string hex16(uint16_t value)
{
    char text[8];
    std::snprintf(text, sizeof(text), "0x%04X", value);
    return text;
}

constexpr std::array<int, 7> kStepDegrees{{5, 15, 30, 90, 360, 3600, 36000}};

int32_t step_pulses(std::size_t index)
{
    return static_cast<int32_t>((kOutputPulsesPerRevolution * kStepDegrees.at(index) + 180) / 360);
}

std::string step_text(std::size_t index)
{
    return std::to_string(kStepDegrees.at(index)) + "° / " +
        std::to_string(step_pulses(index)) + " 电机脉冲";
}

std::string number(double value, int precision = 1)
{
    std::ostringstream output;
    output.setf(std::ios::fixed);
    output.precision(precision);
    output << value;
    return output.str();
}

std::optional<std::string> prompt_text(maintenance_tui::Session& session,
                        const std::string& title,
                        const std::string& summary,
                        const std::string& prompt,
                        bool mouse_enabled,
                        const std::function<void()>& stop_action,
                        bool* stop_requested,
                        const std::vector<std::string>& hints = {})
{
    using namespace maintenance_tui;
    std::string text;
    while (g_running.load()) {
        Dashboard dialog{};
        dialog.title = title;
        dialog.status = "请确认操作";
        dialog.status_tone = Tone::warning;
        dialog.subtitle = summary;
        Panel panel{};
        panel.title = prompt;
        panel.lines = hints;
        panel.lines.insert(panel.lines.end(), {"输入: " + text, "回车确认；Esc/q 取消。",
                                                "空格/x 执行停止并取消当前输入。"});
        dialog.panels.push_back(std::move(panel));
        dialog.shortcuts = "回车确认 | Esc/q 取消 | 空格/x 停止";
        dialog.actions = {{"空格/x", "停止", KeyAction(' ')},
                          {"Esc", "取消", KeyAction(27)}};
        const auto size = terminal_size();
        session.present(dialog, size.columns, size.rows);
        const int key = session.read_key(mouse_enabled, 50);
        if (key < 0) continue;
        if (key == ' ' || key == 'x' || key == 'X') {
            if (stop_action) stop_action();
            if (stop_requested) *stop_requested = true;
            return {};
        }
        if (key == 27 || key == 'q' || key == 'Q') return {};
        if (key == '\n' || key == '\r') return text;
        if (key == 8 || key == 127) {
            if (!text.empty()) text.pop_back();
        } else if (key >= 32 && key <= 126 && text.size() < 32) {
            text.push_back(static_cast<char>(key));
        }
    }
    return {};
}

bool confirm(maintenance_tui::Session& session,
             const std::string& command,
             const std::string& summary,
             bool mouse_enabled,
             const std::function<void()>& stop_action,
             std::string* message)
{
    bool stopped = false;
    const std::optional<std::string> answer = prompt_text(
        session, "YZ-AIM / 操作确认", summary,
        "直接按回车确认；Esc/q 取消", mouse_enabled, stop_action, &stopped);
    (void)command;
    const bool accepted = !stopped && answer && answer->empty();
    if (!accepted && message)
        *message = stopped ? "已取消确认并请求停止。" : "已取消：" + summary;
    return accepted;
}

bool prompt_number(maintenance_tui::Session& session,
                   const std::string& title,
                   const std::string& summary,
                   uint32_t minimum,
                   uint32_t maximum,
                   bool mouse_enabled,
                   const std::function<void()>& stop_action,
                   uint32_t* value,
                   std::string* message)
{
    bool stopped = false;
    const std::optional<std::string> text = prompt_text(session, title, summary,
        "请输入十进制数值", mouse_enabled, stop_action, &stopped);
    uint32_t parsed = 0;
    if (stopped || !text || !parse_uint(text->c_str(), maximum, &parsed) || parsed < minimum) {
        if (message) *message = stopped ? "已取消输入并请求停止。" :
            "输入已取消或超出范围，未发送命令。";
        return false;
    }
    *value = parsed;
    return true;
}

bool prompt_absolute_position(maintenance_tui::Session& session, bool mouse_enabled,
                              const std::function<void()>& stop_action,
                              int32_t observed, std::size_t step_index,
                              int32_t* target, std::string* message)
{
    bool stopped = false;
    const int64_t current = observed;
    const int64_t low = kMinPositionPulses;
    const int64_t high = kMaxPositionPulses;
    const int64_t step = step_pulses(step_index);
    const int64_t example = current + (current <= high - step ? step : -step);
    const auto text = prompt_text(session, "YZ-AIM / 绝对目标",
        "当前 " + std::to_string(observed) + " 电机脉冲；手册总范围 ±" +
            std::to_string(kMaxPositionPulses) + " 脉冲",
        "输入带符号十进制绝对目标（电机脉冲）", mouse_enabled, stop_action, &stopped,
        {"p 是绝对目标；m/M 按所选步长增量移动。",
         "示例：输入 " + std::to_string(example) + " = 当前" +
             (example > current ? "+" : "-") + std::to_string(step) + " 脉冲。",
         "允许 [" + std::to_string(low) + ", " + std::to_string(high) +
             "]，不能等于当前；运动前会重新读取位置。"});
    if (stopped || !text || text->empty()) {
        if (message) *message = stopped ? "已取消输入并请求停止。" : "已取消绝对目标输入，未发送命令。";
        return false;
    }
    int32_t parsed = 0;
    const char* const begin = text->data() + (text->front() == '+' ? 1 : 0);
    const auto parsed_result = std::from_chars(begin, text->data() + text->size(), parsed);
    if (parsed_result.ec != std::errc{} || parsed_result.ptr != text->data() + text->size()) {
        if (message) *message = "请输入有符号32位十进制位置，未发送命令。";
        return false;
    }
    if (parsed < low || parsed > high || parsed == observed) {
        if (message) *message = "绝对目标须在 [" + std::to_string(low) + ", " +
            std::to_string(high) + "] 电机脉冲内且不等于当前；未发送命令。";
        return false;
    }
    *target = parsed;
    return true;
}

bool prompt_output_degrees(maintenance_tui::Session& session, bool mouse_enabled,
                           const std::function<void()>& stop_action, bool absolute,
                           int32_t observed, int32_t* pulses, std::string* message)
{
    bool stopped = false;
    const int64_t current = observed;
    const int64_t relative_low = std::max<int64_t>(std::numeric_limits<int32_t>::min(),
        static_cast<int64_t>(kMinPositionPulses) - current);
    const int64_t relative_high = std::min<int64_t>(kMaxRelativePositionPulses,
        static_cast<int64_t>(kMaxPositionPulses) - current);
    const auto text = prompt_text(session, absolute ? "YZ-AIM / 绝对输出角度" : "YZ-AIM / 相对输出角度",
        "当前输出 " + number(yz_aim::pulses_to_output_degrees(observed), 3) + "° / " +
            std::to_string(observed) + " 脉冲；手册总范围 ±" +
            number(yz_aim::pulses_to_output_degrees(kMaxPositionPulses), 3) + "°",
        absolute ? "输入绝对输出角度（度，可带小数/负号）" :
                   "输入相对输出角度（度，可带小数/负号）",
        mouse_enabled, stop_action, &stopped,
        {"轴位对应电机脉冲 [" + std::to_string(kMinPositionPulses) + ", " +
             std::to_string(kMaxPositionPulses) + "]；p 可直接输入脉冲。",
         absolute ? "目标必须不同于当前位置，运动前重新读取并校验。" :
             "当前允许增量脉冲 [" + std::to_string(relative_low) + ", " +
                 std::to_string(relative_high) + "]；运动前重新校验。"});
    if (stopped || !text || text->empty()) {
        if (message) *message = stopped ? "已请求停止并取消输入。" : "输入已取消，未发送命令。";
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const double degrees = std::strtod(text->c_str(), &end);
    if (errno || end != text->c_str() + text->size() || !std::isfinite(degrees) ||
        (!absolute && degrees == 0)) {
        if (message) *message = "请输入有效非零输出角度（绝对目标可为零），未发送命令。";
        return false;
    }
    int32_t converted = 0;
    const Result conversion = yz_aim::output_revolutions_to_pulses(degrees / 360.0, converted);
    if (!conversion || (!absolute && converted == 0)) {
        if (message) *message = "输出角度超出有符号32位脉冲范围或过小，未发送命令。";
        return false;
    }
    const int64_t resulting_position = absolute ? static_cast<int64_t>(converted) :
        current + converted;
    if (resulting_position < kMinPositionPulses || resulting_position > kMaxPositionPulses ||
        (absolute ? converted == observed :
            static_cast<int64_t>(converted) < relative_low ||
            static_cast<int64_t>(converted) > relative_high)) {
        if (message) *message = "目标超出手册总轴位 [" + std::to_string(kMinPositionPulses) +
            ", " + std::to_string(kMaxPositionPulses) +
            "] 脉冲、相对32位增量范围，或等于当前；未发送命令。";
        return false;
    }
    *pulses = converted;
    return true;
}

std::string output_text(const Snapshot& snapshot, const UiState& state)
{
    if (!snapshot.output_sample.valid) {
        return state.output_raw_valid ?
            "未知（旧值 " + hex16(state.output_raw) + "）" :
            "未知（无有效采样）";
    }
    const bool enabled = yz_aim::output_is_enabled(snapshot.output_enabled);
    return std::string(enabled ? "已使能（原始值 " : "已失能（原始值 ") +
        hex16(snapshot.output_enabled) + "）";
}

maintenance_tui::Tone output_tone(const Snapshot& snapshot)
{
    using maintenance_tui::Tone;
    if (!snapshot.output_sample.valid) return Tone::error;
    return yz_aim::output_is_enabled(snapshot.output_enabled) ? Tone::warning : Tone::normal;
}

void add_status(maintenance_tui::Panel& panel, const Snapshot& snapshot,
                const UiState& state, bool compact)
{
    using namespace maintenance_tui;
    panel.fields = {
        {"状态字", hex16(snapshot.statusword),
         snapshot.status_sample.valid ? Tone::success : Tone::warning},
        {"位置", snapshot.position_sample.valid ?
            std::to_string(snapshot.position_pulses) + " 电机脉冲 / " +
            number(snapshot.output_position_degrees, 3) + " 输出度" : "--"},
        {"转速", snapshot.speed_sample.valid ?
            number(snapshot.speed_rpm, 1) + " 电机 RPM / " +
            number(snapshot.output_speed_rpm, 3) + " 输出 RPM" : "--"},
        {"电流", snapshot.current_sample.valid ?
            number(snapshot.current_a, 3) + " A" : "--"},
        {"电压", snapshot.voltage_sample.valid ?
            number(snapshot.voltage_v, 2) + " V" : "--"},
        {"温度", snapshot.temperature_sample.valid ?
            std::to_string(snapshot.temperature_c) + " ℃" : "--"},
        {"报警", !snapshot.alarm_sample.valid ? "--" :
            snapshot.alarm_code == 0x0020 ?
                "0x0020 通信心跳超时（使能时恢复）" : hex16(snapshot.alarm_code),
            snapshot.alarm_code ? Tone::error : Tone::success},
        {"输出", output_text(snapshot, state), output_tone(snapshot)},
        {"电量", snapshot.battery_sample.valid ?
            std::to_string(snapshot.battery_tenths * 10) + "%" : "--"},
    };
    if (compact) {
        std::vector<KeyValue> essentials;
        for (const char* key : {"报警", "输出", "转速", "位置", "状态字"}) {
            const auto field = std::find_if(panel.fields.begin(), panel.fields.end(),
                [key](const KeyValue& entry) { return entry.key == key; });
            if (field != panel.fields.end()) essentials.push_back(*field);
        }
        panel.fields = std::move(essentials);
        return;
    }
    panel.lines = {
        "转速来自对象 0x606C，输出转速按 51:1 换算。",
        "位置为电机脉冲；输出角度按 32768 x 51 脉冲/圈换算。",
        "停止与输出失能是不同操作。",
        "停止应答不代表机械制动器已锁止。"};
}

enum class Command { refresh, arm, disarm, enable, disable, stop, move, absolute, jog,
                      set_speed, set_accel, set_current, stage_node, stage_bitrate,
                      watchdog_service };

const char* command_name(Command command);

// Operator input errors and cancelled actions are not evidence of a device fault.
bool is_critical_failure(const Result& result)
{
    if (result || result.error == Error::invalid_argument || result.error == Error::cancelled)
        return false;
    return result.message != "output and explicit arm required";
}

class MotorWorker {
public:
    explicit MotorWorker(Motor& motor) : motor_(motor), thread_([this] { run(); })
    {
        (void)schedule(Command::refresh);
    }
    ~MotorWorker()
    {
        request_cancel_if_moving();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
    bool schedule(Command command, int64_t argument = 0, uint32_t profile_rpm = 30)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || pending_ || stopping_) return false;
        if (watchdog_failed_.load() && command != Command::refresh &&
            command != Command::stop && command != Command::disable &&
            command != Command::disarm) return false;
        pending_ = command;
        argument_ = argument;
        profile_rpm_ = profile_rpm;
        busy_ = true;
        active_command_ = command;
        wake_.notify_one();
        return true;
    }
    bool busy() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return busy_;
    }
    bool active_motion() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return busy_ && (active_command_ == Command::move || active_command_ == Command::absolute ||
                         active_command_ == Command::jog);
    }
    void request_cancel_if_moving()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ && (active_command_ == Command::move || active_command_ == Command::absolute ||
                      active_command_ == Command::jog))
            motor_.request_cancel();
    }
    bool take_progress(Snapshot* snapshot)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!progress_updated_) return false;
        if (snapshot) *snapshot = progress_snapshot_;
        progress_updated_ = false;
        return true;
    }
    bool take_update(Snapshot* snapshot, Command* command, Result* result, int64_t* argument,
                     bool* snapshot_updated, bool* cleanup_required = nullptr,
                     Result* snapshot_result = nullptr, std::string* first_fault = nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updated_) return false;
        if (snapshot) *snapshot = snapshot_;
        if (snapshot_updated) *snapshot_updated = snapshot_updated_;
        if (cleanup_required) *cleanup_required = cleanup_required_;
        if (snapshot_result) *snapshot_result = snapshot_result_;
        if (first_fault) *first_fault = first_fault_;
        if (command) *command = completed_command_;
        if (result) *result = result_;
        if (argument) *argument = completed_argument_;
        updated_ = false;
        return true;
    }
private:
    void run()
    {
        auto next_watchdog_service = Clock::now() + std::chrono::milliseconds(200);
        for (;;) {
            Command command{};
            int64_t argument = 0;
            uint32_t profile_rpm = 30;
            bool service_due = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait_until(lock, next_watchdog_service,
                    [&] { return pending_.has_value() || stopping_; });
                if (stopping_ && !pending_) break;
                if (!pending_) {
                    service_due = true;
                    next_watchdog_service = Clock::now() + std::chrono::milliseconds(200);
                } else {
                    command = *pending_;
                    pending_.reset();
                    argument = argument_;
                    profile_rpm = profile_rpm_;
                }
            }
            if (service_due) {
                if (motor_.watchdog_active()) {
                    Result service = motor_.service_watchdog(Clock::now() + std::chrono::milliseconds(100));
                    if (!service) {
                        watchdog_failed_.store(true);
                        std::lock_guard<std::mutex> lock(mutex_);
                        completed_command_ = Command::watchdog_service;
                        completed_argument_ = 0;
                        if (first_fault_.empty() && is_critical_failure(service))
                            first_fault_ = std::string(command_name(Command::watchdog_service)) + "：" + result_text(service);
                        result_ = std::move(service);
                        snapshot_updated_ = false;
                        snapshot_result_ = {};
                        cleanup_required_ = motor_.cleanup_required();
                        updated_ = true;
                    }
                }
                continue;
            }
            Result result{};
            const ProgressCallback progress = [this](const Snapshot& reading) {
                std::lock_guard<std::mutex> lock(mutex_);
                progress_snapshot_ = reading;
                progress_updated_ = true;
            };
            switch (command) {
            case Command::refresh: break;
            case Command::arm: result = motor_.arm(Clock::now() + kStopActionBudget); break;
            case Command::disarm: result = motor_.disarm(Clock::now() + kStopActionBudget); break;
            case Command::enable:
                result = motor_.arm(Clock::now() + kStopActionBudget);
                if (result) result = motor_.enable_output(Clock::now() + kStopActionBudget);
                break;
            case Command::disable: result = motor_.disable_output(Clock::now() + kStopActionBudget); break;
            case Command::stop: result = motor_.stop(Clock::now() + kStopActionBudget); break;
            case Command::move:
                result = motor_.arm(Clock::now() + kStopActionBudget);
                if (result) result = motor_.move_relative(argument,
                    Clock::now() + position_action_budget(argument, profile_rpm), profile_rpm, progress);
                break;
            case Command::absolute:
                result = motor_.arm(Clock::now() + kStopActionBudget);
                if (result) result = motor_.move_absolute(static_cast<int32_t>(argument),
                    Clock::now() + position_action_budget(static_cast<int64_t>(kMaxPositionPulses) * 2, profile_rpm), profile_rpm, progress);
                break;
            case Command::jog:
                result = motor_.arm(Clock::now() + kStopActionBudget);
                if (result) result = motor_.jog(static_cast<int32_t>(argument), std::chrono::milliseconds(250),
                    Clock::now() + jog_action_budget(std::chrono::milliseconds(250)), progress);
                break;
            case Command::set_speed: result = motor_.set_profile_speed(static_cast<uint32_t>(argument), action_deadline()); break;
            case Command::set_accel: result = motor_.set_acceleration(static_cast<uint32_t>(argument), action_deadline()); break;
            case Command::set_current: result = motor_.set_current_limit(static_cast<uint16_t>(argument), action_deadline()); break;
            case Command::stage_node: result = motor_.stage_node_id(static_cast<uint8_t>(argument), action_deadline()); break;
            case Command::stage_bitrate: result = motor_.stage_bitrate(static_cast<uint32_t>(argument), action_deadline()); break;
            case Command::watchdog_service: result = motor_.service_watchdog(action_deadline()); break;
            }
            Snapshot snapshot{};
            const bool has_snapshot = command == Command::refresh ||
                command == Command::enable || command == Command::disable ||
                command == Command::move || command == Command::absolute || command == Command::jog;
            if (has_snapshot) snapshot = motor_.snapshot(poll_deadline());
            if (motor_.unsafe_latched()) watchdog_failed_.store(true);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // Preserve the action result even when its follow-up snapshot fails.
                if (command != Command::refresh && first_fault_.empty() && is_critical_failure(result))
                    first_fault_ = std::string(command_name(command)) + "：" + result_text(result);
                if (has_snapshot && first_fault_.empty() && is_critical_failure(snapshot.result))
                    first_fault_ = command == Command::refresh ?
                        "读取：" + result_text(snapshot.result) :
                        std::string(command_name(command)) + "后采样：" + result_text(snapshot.result);
                result_ = command == Command::refresh ? snapshot.result : result;
                snapshot_result_ = has_snapshot ? snapshot.result : Result{};
                if (has_snapshot) snapshot_ = snapshot;
                snapshot_updated_ = has_snapshot;
                cleanup_required_ = motor_.cleanup_required();
                completed_command_ = command;
                completed_argument_ = argument;
                busy_ = false;
                updated_ = true;
            }
        }
    }
    Motor& motor_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<Command> pending_;
    Command active_command_ = Command::refresh;
    Command completed_command_ = Command::refresh;
    int64_t argument_ = 0;
    uint32_t profile_rpm_ = 30;
    Snapshot progress_snapshot_{};
    bool progress_updated_ = false;
    int64_t completed_argument_ = 0;
    bool busy_ = false;
    bool stopping_ = false;
    bool updated_ = false;
    bool snapshot_updated_ = false;
    bool cleanup_required_ = false;
    std::atomic<bool> watchdog_failed_{false};
    Snapshot snapshot_{};
    Result result_{};
    Result snapshot_result_{};
    std::string first_fault_;
    std::thread thread_;
};

const char* command_name(Command command)
{
    switch (command) {
    case Command::refresh: return "读取";
    case Command::arm: return "安全检查";
    case Command::disarm: return "解除检查";
    case Command::enable: return "安全检查并使能";
    case Command::disable: return "输出失能";
    case Command::stop: return "停止请求";
    case Command::move: return "相对运动";
    case Command::absolute: return "绝对位置";
    case Command::jog: return "点动";
    case Command::set_speed: return "运行速度";
    case Command::set_accel: return "加速度";
    case Command::set_current: return "电流限制";
    case Command::stage_node: return "节点配置";
    case Command::stage_bitrate: return "波特率配置";
    case Command::watchdog_service: return "心跳服务";
    }
    return "操作";
}

int run_loop(Motor& motor, const Options& options, bool setup_mode)
{
    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
        std::cerr << "交互模式需要终端；运行 --help 查看参数\n";
        return 2;
    }
    g_running.store(true);
    install_signals();
    std::setlocale(LC_ALL, "");
    maintenance_tui::TerminalMode terminal;
    if (!terminal.enter(!options.no_mouse)) {
        std::cerr << "无法进入终端模式\n";
        return 2;
    }
    maintenance_tui::Session session;
    MotorWorker worker(motor);
    UiState state{};
    state.mock = options.mock;
    state.setup_mode = setup_mode;
    state.message = setup_mode ? "启动只读；设置操作需手动确认。" : "启动只读；先 e 使能，v 选电机 RPM，a/r 输入输出轴角度。";
    Snapshot latest{};
    bool have_snapshot = false;
    bool running = true;
    bool stop_when_idle = false;
    bool exit_when_idle = false;
    bool preserve_action_result = false;
    auto last_poll = Clock::time_point::min();

    while (running) {
        Snapshot progress{};
        if (worker.take_progress(&progress)) {
            latest = progress;
            have_snapshot = true;
            last_poll = Clock::now();
            if (progress.output_sample.valid) {
                state.output_raw = progress.output_enabled;
                state.output_raw_valid = true;
            }
            if (progress.unsafe_latched) state.watchdog_failed = true;
        }
        Command completed{};
        Result result{};
        int64_t completed_argument = 0;
        Snapshot updated{};
        bool updated_snapshot = false;
        bool cleanup_required = false;
        Result updated_snapshot_result{};
        std::string first_fault;
        if (worker.take_update(&updated, &completed, &result, &completed_argument,
                               &updated_snapshot, &cleanup_required,
                               &updated_snapshot_result, &first_fault)) {
            state.first_fault = std::move(first_fault);
            state.output_owned = cleanup_required;
            if (updated_snapshot) {
                latest = updated;
                have_snapshot = true;
                last_poll = Clock::now();
            }
            if (completed == Command::refresh) {
                if (!preserve_action_result)
                    state.message = std::string(command_name(completed)) + "：" + result_text(result);
            } else {
                state.message = result.error == Error::cancelled ?
                    std::string(command_name(completed)) + "：已确认停止，位置动作已取消。" :
                    std::string(command_name(completed)) + "：" + result_text(result);
                preserve_action_result = true;
            }
            if (completed != Command::refresh && completed != Command::watchdog_service &&
                updated_snapshot && !updated_snapshot_result) {
                const std::string diagnostic = std::string(command_name(completed)) +
                    "后采样：" + result_text(updated_snapshot_result);
                state.message += "；" + diagnostic;
                state.history.push_back(diagnostic);
                if (state.history.size() > 3) state.history.erase(state.history.begin());
            }
            if (completed == Command::watchdog_service) state.watchdog_failed = true;
            if (completed == Command::arm) state.armed = static_cast<bool>(result);
            if (completed == Command::enable) state.armed = static_cast<bool>(result);
            if (completed == Command::disarm) state.armed = false;
            if (updated_snapshot && updated.output_sample.valid) {
                state.output_raw = updated.output_enabled;
                state.output_raw_valid = true;
            }
            if (updated_snapshot && updated.unsafe_latched) state.watchdog_failed = true;
            if (completed == Command::disable || completed == Command::stop) {
                state.armed = false;
                state.motion_active = false;
            }
            if (completed == Command::set_speed && result)
                state.profile_speed = std::to_string(completed_argument) + " 电机 RPM / " +
                    number(yz_aim::motor_to_output_rpm(completed_argument), 3) + " 输出 RPM（已读回确认）";
            if (completed == Command::set_accel && result)
                state.acceleration = std::to_string(completed_argument) + " 电机 RPM/s / " +
                    number(yz_aim::motor_to_output_rpm(completed_argument), 3) + " 输出 RPM/s（已读回确认）";
            if (completed == Command::set_current && result)
                state.current_limit = std::to_string(completed_argument) + " mA（本次已确认）";
            if (completed == Command::stage_node && result) {
                state.staged_node = std::to_string(options.node) + " -> " +
                    std::to_string(completed_argument) + "（断电重启后再读回）";
                state.message += "；不会在线切换，请断电重启后按新节点重新连接";
            }
            if (completed == Command::stage_bitrate && result) {
                state.staged_bitrate = std::to_string(options.bitrate) + " -> " +
                    std::to_string(completed_argument) + " bit/s（断电重启后再读回）";
                state.message += "；仅收到确认，不能在线切换，请整条总线断电重启";
            }
            if (completed == Command::move || completed == Command::absolute || completed == Command::jog) {
                state.armed = false; // the motor's completed motion always ends with STOP
                state.motion_active = false;
            }
        }
        if (!g_running.load() || exit_when_idle) {
            if (worker.active_motion()) {
                worker.request_cancel_if_moving();
                state.message = "收到退出信号，正在请求取消运动。";
            }
            if (!worker.busy() && !stop_when_idle) running = false;
        }
        if (stop_when_idle && !worker.busy()) {
            if (worker.schedule(Command::stop)) {
                stop_when_idle = false;
                state.message = "当前操作完成后执行停止。";
            }
        }
        if (running && !worker.busy() &&
            (!have_snapshot || Clock::now() - last_poll >= std::chrono::seconds(2)))
            (void)worker.schedule(Command::refresh);

        const auto size = terminal_size();
        session.present(make_dashboard(latest, state, options.node,
            options.bitrate, options.mock ? "模拟设备 / 无硬件" : "真实 LYS",
            size.rows), size.columns, size.rows);
        const int key = session.read_key(!options.no_mouse, 100);
        if (key < 0) continue;
        preserve_action_result = false;
        if (state.watchdog_failed && key != ' ' && key != 'x' && key != 'X' && key != 'd' && key != 'D' &&
            key != 'R' && key != 'q' && key != 'Q' && key != 27 &&
            key != '?' && key != 'h' && key != 'H') {
                state.message = "安全状态已锁定；仅允许刷新、停止、失能或退出。";
            continue;
        }
        auto request_stop = [&] {
            if (options.node == 127) { state.message = "节点 127 为只读，未发送停止命令。"; return; }
            if (worker.active_motion()) {
                worker.request_cancel_if_moving();
                state.message = "已请求停止，正在取消运动并保持。";
            } else if (worker.busy()) {
                stop_when_idle = true;
                state.message = "停止已排队，将在当前限时操作后执行。";
            } else if (worker.schedule(Command::stop)) {
                state.armed = false;
                state.motion_active = false;
                state.message = "已请求停止 0x010F；机械制动状态未确认。";
            }
        };
        if (key == 'q' || key == 'Q' || key == 27) {
            if (worker.busy()) {
                exit_when_idle = true;
                if (worker.active_motion()) worker.request_cancel_if_moving();
                state.message = worker.active_motion() ?
                    "正在取消运动，完成后退出。" :
                    "正在等待当前操作完成后退出。";
                continue;
            }
            running = false;
        } else if (key == '?' || key == 'h' || key == 'H') {
            state.help = !state.help;
        } else if (key == ' ' || key == 'x' || key == 'X') {
            request_stop();
        } else if (!setup_mode && (key == '[' || key == ']')) {
            if (key == '[' && state.position_step_index > 0) --state.position_step_index;
            if (key == ']' && state.position_step_index + 1 < kStepDegrees.size())
                ++state.position_step_index;
            state.message = "所选输出轴步长 " + step_text(state.position_step_index) +
                "；仅改变下次 m/M 位移，未发送运动命令。";
        } else if ((setup_mode && (key == 'r' || key == 'R')) || (!setup_mode && key == 'R')) {
            if (!worker.busy()) (void)worker.schedule(Command::refresh);
            else state.message = "当前操作完成后才能刷新。";
        } else if (setup_mode && (key == 's' || key == 'S' || key == 'a' || key == 'A' || key == 'c' || key == 'C')) {
            if (options.node == 127) { state.message = "节点 127 为只读，设置写入已禁用。"; continue; }
            if (worker.busy()) { state.message = "当前有操作进行中，请稍候。"; continue; }
            const uint32_t selection = (key == 's' || key == 'S') ? 1 :
                (key == 'a' || key == 'A') ? 2 : 3;
            std::string message;
            const char* label = selection == 1 ? "运行速度（电机 RPM）" :
                selection == 2 ? "加速度（电机 RPM/s）" : "电流限制（mA）";
            const uint32_t min = selection == 1 ? 0 : selection == 2 ? 10 : 10;
            const uint32_t max = selection == 1 ? kMaxMotorRpm : selection == 2 ? 1000 : 10000;
            uint32_t value = 0;
            if (!prompt_number(session, "YZ-AIM / 设置参数",
                    std::string("请输入 ") + label + "，回车发送；当前值为本次会话最近确认值。",
                    min, max, !options.no_mouse, request_stop, &value, &message)) {
                state.message = message;
                continue;
            }
            if (selection == 3 && value % 10 != 0) {
                state.message = "电流限制必须是 10 mA 的整数倍，未发送命令。";
                continue;
            }
            const Command command = selection == 1 ? Command::set_speed :
                selection == 2 ? Command::set_accel : Command::set_current;
            if (!worker.schedule(command, value)) state.message = "当前有操作进行中，请稍候。";
            else state.message = std::string("正在设置") + label + " = " + std::to_string(value) + "；等待读回确认。";
        } else if (setup_mode && (key == 'n' || key == 'N')) {
            if (worker.busy() || options.node == 127) {
                state.message = options.node == 127 ?
                    "节点 127 为只读，节点配置已禁用。" :
                    "当前有操作进行中，请稍候。";
                continue;
            }
            uint32_t target = 0;
            std::string message;
            if (!prompt_number(session, "YZ-AIM / 配置节点",
                    "输入 1~126；只暂存配置，不在线切换，完成后需整机断电重启。",
                    1, 126, !options.no_mouse, request_stop, &target, &message)) {
                state.message = message;
                continue;
            }
            if (target == options.node) {
                state.message = "新节点与当前节点相同，未发送命令。";
                continue;
            }
            const std::string summary = "节点 " + std::to_string(options.node) + " → " +
                std::to_string(target) + "；确认后写入，断电重启后才生效。";
            if (confirm(session, "节点", summary, !options.no_mouse, request_stop, &message)) {
                if (!worker.schedule(Command::stage_node, target)) state.message = "当前有操作进行中，请稍候。";
            } else state.message = message;
        } else if (setup_mode && (key == 'b' || key == 'B')) {
            if (worker.busy() || options.node == 127) {
                state.message = options.node == 127 ?
                    "节点 127 为只读，波特率配置已禁用。" :
                    "当前有操作进行中，请稍候。";
                continue;
            }
            uint32_t target = 0;
            std::string message;
            if (!prompt_number(session, "YZ-AIM / 配置波特率",
                    "输入 125000、250000、500000 或 1000000；断电重启后生效。",
                    125000, 1000000, !options.no_mouse, request_stop, &target, &message)) {
                state.message = message;
                continue;
            }
            if (target != 125000 && target != 250000 && target != 500000 && target != 1000000) {
                state.message = "不支持的波特率，未发送命令。";
                continue;
            }
            const std::string summary = "波特率 " +
                (options.mock ? std::string("模拟") : std::to_string(options.bitrate)) + " → " +
                std::to_string(target) + " bit/s；确认后暂存，整条总线断电重启后生效。";
            if (confirm(session, "波特率", summary, !options.no_mouse, request_stop, &message)) {
                if (!worker.schedule(Command::stage_bitrate, target)) state.message = "当前有操作进行中，请稍候。";
            } else state.message = message;
        } else if (!setup_mode && (key == 'v' || key == 'V' || key == 'a' || key == 'A' || key == 'r')) {
            if (options.node == 127) { state.message = "节点 127 为只读，未发送运动命令。"; continue; }
            if (worker.busy()) {
                state.message = "运动/操作中，请先按空格停止，等待停稳后再输入速度或角度。";
                continue;
            }
            if (key != 'v' && key != 'V' && (!have_snapshot || !latest.position_sample.valid)) {
                state.message = "缺少有效位置读数；请先按 R 刷新。";
                continue;
            }
            std::string message;
            if (key == 'v' || key == 'V') {
                uint32_t speed = 0;
                if (!prompt_number(session, "YZ-AIM / 选择下次运动速度",
                    "输入 1~3000 电机 RPM；仅改变本次 TUI 参数，不写设备，不保存。",
                    1, kMaxMotorRpm, !options.no_mouse, request_stop, &speed, &message)) {
                    state.message = message;
                    continue;
                }
                state.selected_speed_rpm = speed;
                state.message = "下次运动 " + std::to_string(speed) + " 电机 RPM / " +
                    number(yz_aim::motor_to_output_rpm(speed), 3) + " 输出 RPM；未写设备。";
                continue;
            }
            const bool absolute = key == 'a' || key == 'A';
            int32_t pulses = 0;
            if (!prompt_output_degrees(session, !options.no_mouse, request_stop,
                absolute, latest.position_pulses, &pulses, &message)) {
                state.message = message;
                continue;
            }
            state.motion_active = worker.schedule(absolute ? Command::absolute : Command::move,
                                                  pulses, state.selected_speed_rpm);
            state.message = state.motion_active ?
                std::string(absolute ? "已请求绝对输出目标 " : "已请求相对输出位移 ") +
                number(yz_aim::pulses_to_output_degrees(pulses), 3) + "° / " +
                std::to_string(pulses) + " 电机脉冲；等待实时采样。" :
                "当前操作尚未完成，请稍候。";
        } else if (!setup_mode && (key == 'e' || key == 'E')) {
            if (options.node == 127 || worker.busy()) {
                state.message = options.node == 127 ? "节点 127 为只读，不能使能输出。" : "当前有操作进行中，请稍候。";
                continue;
            }
            if (!worker.schedule(Command::enable)) state.message = "当前有操作进行中，请稍候。";
            else state.message = "正在检查设备并安全使能，不会启动运动。";
        } else if (!setup_mode && (key == 'd' || key == 'D')) {
            if (options.node == 127) { state.message = "节点 127 为只读，未发送失能命令。"; continue; }
            if (worker.busy()) { state.message = "当前有操作进行中，请稍候。"; continue; }
            if (!worker.schedule(Command::disable)) state.message = "当前有操作进行中，请稍候。";
            else state.message = "正在停止并确认静止，然后关闭输出。";
        } else if (!setup_mode && (key == 'm' || key == 'M' ||
                                   key == maintenance_tui::kKeyUp ||
                                   key == maintenance_tui::kKeyDown)) {
            if (options.node == 127 || worker.busy()) {
                state.message = options.node == 127 ? "节点 127 为只读，不能运动。" : "当前有操作进行中，请稍候。";
                continue;
            }
            const int32_t pulses = (key == 'm' || key == maintenance_tui::kKeyUp) ?
                step_pulses(state.position_step_index) : -step_pulses(state.position_step_index);
            const double angle = yz_aim::pulses_to_output_degrees(pulses);
            state.motion_active = worker.schedule(Command::move, pulses, state.selected_speed_rpm);
            if (!state.motion_active) state.message = "当前操作尚未完成，请稍候。";
            else state.message = "已请求输出轴位移 " + std::to_string(kStepDegrees.at(state.position_step_index)) +
                "°（" + std::to_string(pulses) + " 电机脉冲，约 " + number(angle, 3) +
                "°）；实时检查和限幅由驱动执行。";
        } else if (!setup_mode && (key == 'p' || key == 'P')) {
            if (options.node == 127 || worker.busy()) {
                state.message = options.node == 127 ? "节点 127 为只读，不能运动。" : "当前有操作进行中，请稍候。";
                continue;
            }
            if (!have_snapshot || !latest.position_sample.valid) {
                state.message = "缺少有效位置读数；请先按 R 刷新。";
                continue;
            }
            int32_t target = 0;
            std::string message;
            if (!prompt_absolute_position(session, !options.no_mouse, request_stop,
                                          latest.position_pulses, state.position_step_index,
                                          &target, &message)) {
                state.message = message;
                continue;
            }
            state.motion_active = worker.schedule(Command::absolute, target, state.selected_speed_rpm);
            state.message = state.motion_active ?
                "绝对目标 " + std::to_string(target) + " 电机脉冲；驱动将重读当前位置并核对手册轴位范围。" :
                "当前操作尚未完成，请稍候。";
        } else if (!setup_mode && (key == 'j' || key == 'J' ||
                                   key == maintenance_tui::kKeyRight ||
                                   key == maintenance_tui::kKeyLeft)) {
            if (options.node == 127 || worker.busy()) {
                state.message = options.node == 127 ? "节点 127 为只读，不能运动。" : "当前有操作进行中，请稍候。";
                continue;
            }
            const int32_t rpm = (key == 'j' || key == maintenance_tui::kKeyRight) ?
                static_cast<int32_t>(state.selected_speed_rpm) : -static_cast<int32_t>(state.selected_speed_rpm);
            state.motion_active = worker.schedule(Command::jog, rpm);
            if (!state.motion_active) state.message = "当前操作尚未完成，请稍候。";
            else state.message = "已请求点动 " + std::to_string(rpm) + " RPM，持续 250 ms。";
        }

    }
    int exit_code = 0;
    std::string cleanup_diagnostic;
    if (state.output_owned || state.armed || state.motion_active) {
        const Command cleanup = state.output_owned ? Command::disable : Command::disarm;
        bool cleanup_ok = worker.schedule(cleanup);
        if (cleanup_ok) {
            while (worker.busy()) (void)session.read_key(false, 20);
            Command completed{};
            Result result{};
            std::string first_fault;
            cleanup_ok = worker.take_update(nullptr, &completed, &result, nullptr, nullptr,
                                            nullptr, nullptr, &first_fault) && completed == cleanup;
            if (!first_fault.empty()) state.first_fault = std::move(first_fault);
            if (cleanup_ok) {
                cleanup_ok = static_cast<bool>(result);
                state.message = std::string(command_name(completed)) +
                    (cleanup_ok ? "：退出清理已确认。" : "：退出清理失败：" + result_text(result));
            }
        }
        if (!cleanup_ok) {
            if (state.message.find("退出清理失败") == std::string::npos)
                state.message = "无法确认输出已清理；请检查现场状态。";
            cleanup_diagnostic = "YZ-AIM 退出清理结果：" + state.message;
            if (state.first_fault.empty()) state.first_fault = state.message;
            exit_code = 6;
        }
    }
    terminal.restore();
    if (!cleanup_diagnostic.empty()) std::cerr << cleanup_diagnostic << "\n";
    if (!state.first_fault.empty())
        std::cerr << "YZ-AIM 首个故障：" << state.first_fault << "\n";
    return exit_code;
}

class UiMockTransport final : public MockTransport {
public:
    explicit UiMockTransport(uint8_t node)
    {
        abort_enable_once_ = std::getenv("YZ_AIM_MOCK_ABORT_ENABLE") != nullptr;
        put(0x6041, 0, 0x0427, 2);
        put(0x260e, 0, 0, 2);
        put(0x2601, 0, 0x0006, 2); // opaque high bits stay visible while bit 0 is toggled
        int32_t initial_position = 1200;
        if (const char* initial = std::getenv("YZ_AIM_MOCK_POSITION")) {
            const char* end = initial + std::char_traits<char>::length(initial);
            const auto parsed = std::from_chars(initial, end, initial_position);
            if (parsed.ec != std::errc{} || parsed.ptr != end ||
                initial_position < kMinPositionPulses || initial_position > kMaxPositionPulses)
                initial_position = 1200;
        }
        put(0x6064, 0, static_cast<uint32_t>(initial_position), 4);
        put(0x606c, 0, 0, 2);
        put(0x6078, 0, 0, 2);
        put(0x6079, 0, 24000, 2);
        put(0x2612, 0, 27, 2);
        put(0x261e, 0, 8, 2);
        put(0x6081, 0, 10, 4);
        put(0x6083, 0, 500, 4);
        put(0x1016, 1, 0, 4); // synthetic consumer: no watchdog before operator motion
        put(0x6060, 0, 0, 1);
        put(0x261d, 0, 3000, 2);
        put(0x1000, 0, 0x00020192, 4);
        put(0x1018, 0, 4, 1);
        put(0x1018, 1, 0x00000331, 4);
        put(0x1018, 2, 0x00000001, 4);
        put(0x1018, 3, 0x00010000, 4);
        put(0x1018, 4, 0x00000001, 4);
        on_send = [this, node](const Frame& request, MockTransport& transport) {
            if (request.id == 0x77f) return;
            if (request.id != static_cast<uint16_t>(0x600 + node) || request.dlc != 8)
                return;
            const uint16_t index = static_cast<uint16_t>(request.data[1]) |
                (static_cast<uint16_t>(request.data[2]) << 8);
            const uint8_t subindex = request.data[3];
            const ObjectKey key{index, subindex};
            Frame response{};
            response.id = static_cast<uint16_t>(0x580 + node);
            response.dlc = 8;
            response.data[1] = request.data[1];
            response.data[2] = request.data[2];
            response.data[3] = subindex;
            if (request.data[0] == 0x40) {
                if (index == 0x6064 && pending_position_) {
                    const int64_t actual = static_cast<int32_t>(values_.at({0x6064, 0}).first);
                    const int64_t remaining = pending_position_target_ - actual;
                    const int64_t step = remaining / 3;
                    const int64_t next = step == 0 ? pending_position_target_ : actual + step;
                    put(0x6064, 0, static_cast<uint32_t>(static_cast<int32_t>(next)), 4);
                    if (next == pending_position_target_) {
                        pending_position_ = false;
                        put(0x606c, 0, 0, 2);
                        put(0x6041, 0, 0x0427, 2);
                    } else {
                        put(0x6041, 0, 0x0027, 2);
                    }
                }
                const auto found = values_.find(key);
                if (found == values_.end()) {
                    response.data[0] = 0x80;
                    response.data[4] = 0x00;
                    response.data[5] = 0x00;
                    response.data[6] = 0x02;
                    response.data[7] = 0x06;
                } else {
                    const uint32_t value = found->second.first;
                    const uint8_t width = found->second.second;
                    response.data[0] = width == 1 ? 0x4f : width == 2 ? 0x4b : 0x43;
                    for (unsigned i = 0; i < width; ++i)
                        response.data[4 + i] = static_cast<uint8_t>(value >> (8 * i));
                }
            } else {
                if (abort_enable_once_ && index == 0x2601 && request.data[4] == 1) {
                    abort_enable_once_ = false;
                    response.data[0] = 0x80;
                    response.data[4] = 0x00;
                    response.data[5] = 0x00;
                    response.data[6] = 0x02;
                    response.data[7] = 0x06;
                    transport.replies.push_back(response);
                    return;
                }
                const uint8_t command = request.data[0];
                const uint8_t width = command == 0x2f ? 1 : command == 0x2b ? 2 : 4;
                uint32_t value = 0;
                for (unsigned i = 0; i < width; ++i)
                    value |= static_cast<uint32_t>(request.data[4 + i]) << (8 * i);
                if (index == 0x2601) {
                    const uint32_t previous = values_.at(key).first;
                    put(index, subindex, (previous & ~1U) | (value & 1U), width);
                } else {
                    put(index, subindex, value, width);
                }
                if (index == 0x2614 && value == 1) put(index, subindex, 2, 2);
                if (index == 0x6040 && value == 0x010f) {
                    pending_position_ = false;
                    put(0x6041, 0, 0x0427, 2);
                    put(0x606c, 0, 0, 2);
                }
                if (index == 0x6040 && value == 0x004f &&
                    values_.at({0x6060, 0}).first == 1 &&
                    yz_aim::output_is_enabled(static_cast<uint16_t>(values_.at({0x2601, 0}).first))) {
                    put(0x6041, 0, 0x0027, 2); // target pending; position has not moved
                }
                if (index == 0x6040 && value == 0x005f &&
                    values_.at({0x6060, 0}).first == 1 &&
                    yz_aim::output_is_enabled(static_cast<uint16_t>(values_.at({0x2601, 0}).first)) &&
                    values_.at({0x1016, 1}).first == 0x007f03e8) {
                    const int64_t actual = static_cast<int32_t>(values_.at({0x6064, 0}).first);
                    const int64_t delta = static_cast<int32_t>(values_.at({0x607a, 0}).first);
                    const int64_t next = actual + delta;
                    if (next >= kMinPositionPulses && next <= kMaxPositionPulses) {
                        pending_position_ = true;
                        pending_position_target_ = next;
                        put(0x606c, 0, static_cast<uint16_t>(values_.at({0x6081, 0}).first * 10), 2);
                        put(0x6041, 0, 0x0027, 2); // synthetic target-pending state
                    } else {
                        put(0x6041, 0, 0x8027, 2); // synthetic positive-limit fault
                    }
                }
                if (index == 0x607a && values_.at({0x6040, 0}).first == 0x002f &&
                    values_.at({0x6060, 0}).first == 1 &&
                    yz_aim::output_is_enabled(static_cast<uint16_t>(values_.at({0x2601, 0}).first)) &&
                    values_.at({0x1016, 1}).first == 0x007f03e8) {
                    pending_position_ = true;
                    pending_position_target_ = static_cast<int32_t>(value);
                    put(0x606c, 0, static_cast<uint16_t>(values_.at({0x6081, 0}).first * 10), 2);
                    put(0x6041, 0, 0x0027, 2); // synthetic target-pending state
                }
                if (index == 0x6040 && value == 0x000f &&
                    values_.at({0x6060, 0}).first == 3 &&
                    yz_aim::output_is_enabled(static_cast<uint16_t>(values_.at({0x2601, 0}).first)) &&
                    values_.at({0x1016, 1}).first == 0x007f03e8) {
                    const int32_t rpm = static_cast<int32_t>(values_.at({0x60ff, 0}).first);
                    put(0x606c, 0, static_cast<uint16_t>(rpm * 10), 2);
                    put(0x6041, 0, 0x0027, 2);
                }
                response.data[0] = 0x60;
            }
            transport.replies.push_back(response);
        };
    }
private:
    using ObjectKey = std::pair<uint16_t, uint8_t>;
    void put(uint16_t index, uint8_t subindex, uint32_t value, uint8_t width)
    {
        values_[{index, subindex}] = {value, width};
    }
    std::map<ObjectKey, std::pair<uint32_t, uint8_t>> values_;
    bool pending_position_ = false;
    int64_t pending_position_target_ = 0;
    bool abort_enable_once_ = false;
};

} // namespace

bool parse_terminal_size(const std::string& text, TerminalSize* size)
{
    if (!size) return false;
    int columns = 0, rows = 0;
    char trailing = 0;
    if (std::sscanf(text.c_str(), "%dx%d%c", &columns, &rows, &trailing) != 2 ||
        columns <= 0 || rows <= 0) return false;
    size->columns = columns;
    size->rows = rows;
    return true;
}

TerminalSize terminal_size()
{
    TerminalSize size{};
    winsize value{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &value) == 0) {
        if (value.ws_col) size.columns = value.ws_col;
        if (value.ws_row) size.rows = value.ws_row;
    }
    return size;
}

maintenance_tui::Dashboard make_dashboard(const Snapshot& snapshot,
                                           const UiState& state,
                                           uint8_t node,
                                           uint32_t bitrate,
                                           const std::string& mode,
                                           int terminal_rows)
{
    using namespace maintenance_tui;
    const bool compact = terminal_rows < 32;
    Dashboard view{};
    view.title = compact ? "YZ-AIM CANopen | 减速比 51:1" :
        "YZ-AIM CANopen 维护工具 | 减速比 51:1";
    const bool snapshot_complete = snapshot.result && snapshot.identity_sample.valid && snapshot.status_sample.valid &&
        snapshot.speed_sample.valid && snapshot.alarm_sample.valid &&
        snapshot.output_sample.valid && snapshot.position_sample.valid;
    view.status = state.mock ? "模拟设备 / 无硬件" :
        (snapshot_complete ? "在线 / 采样完整" : "离线 / 采样不完整");
    view.status_tone = state.mock ? Tone::warning :
        (snapshot_complete ? Tone::success : Tone::error);
    view.subtitle = std::string(state.setup_mode ? "" : "v 选速度 | r 相对角度 | a 绝对角度 | ") +
        "节点=" + std::to_string(node) + " | " + mode +
        " | " + (state.mock ? std::string("模拟") : std::to_string(bitrate) + " bit/s");
    view.message = state.first_fault.empty() ? state.message :
        "首个故障：" + state.first_fault;
    view.message_tone = state.first_fault.empty() && snapshot.result ? Tone::normal : Tone::warning;
    view.shortcuts = state.help ? "空格 停止 | q 退出 | ? 返回" :
        state.setup_mode ? (node == 127 ? "只读节点 | r 刷新 | ? 帮助 | q 退出" :
            "空格 停止 | s 速度 | a 加速度 | c 电流 | n 节点 | b 波特率 | ? 帮助 | q 退出") :
        node == 127 ? "只读节点 | R 刷新 | ? 帮助 | q 退出" :
            "e 使能 | v 速度 | r 相对角度 | a 绝对角度 | d 失能 | 空格 停止 | q 退出";
    view.actions = {{"q", "退出", KeyAction('q')}};
    if (node != 127) {
        if (state.setup_mode) {
            view.actions.push_back({"空格", "停止", KeyAction(' ')});
            if (!state.help) {
                view.actions.push_back({"s", "速度", KeyAction('s')});
                view.actions.push_back({"a", "加速", KeyAction('a')});
                view.actions.push_back({"c", "电流", KeyAction('c')});
                view.actions.push_back({"n", "节点", KeyAction('n')});
                view.actions.push_back({"b", "波特率", KeyAction('b')});
            }
        } else if (!state.help) {
            view.actions.push_back({"e", "使能", KeyAction('e')});
            view.actions.push_back({"v", "速度", KeyAction('v')});
            view.actions.push_back({"r", "相对", KeyAction('r')});
            view.actions.push_back({"a", "绝对", KeyAction('a')});
            view.actions.push_back({"d", "失能", KeyAction('d')});
            view.actions.push_back({"空格", "停", KeyAction(' ')});
        } else {
            view.actions.push_back({"空格", "停止", KeyAction(' ')});
        }
    }
    if (state.setup_mode || state.help) view.actions.push_back({"?", "帮助", KeyAction('?')});
    if (!state.setup_mode && !state.help) view.actions.push_back({"?", "帮助", KeyAction('?')});
    if (node == 127) view.actions.push_back({state.setup_mode ? "r" : "R", "刷新",
                                               KeyAction(state.setup_mode ? 'r' : 'R')});
    if (state.help) {
        Panel help{};
        help.title = "帮助 / 安全";
        if (state.setup_mode) {
            help.lines = {"启动和刷新只读取状态；节点 127 只允许读取。",
                          "s 运行速度 0~3000 电机 RPM；a 加速度 10~1000 电机 RPM/s。",
                          "c 电流限制 10~10000 mA（以 10 mA 为单位）。",
                          "s/a/c 输入数值后按回车设置，需设备读回一致；不会自动保存。",
                          "n/b 配置节点或波特率须再次回车确认，断电重启后生效。",
                          "空格或 x 请求停止；q/Esc 退出并恢复终端。"};
        } else {
            help.lines = {"启动只读；e 检查设备并使能，d 确认停止后失能。",
                          "v 选择 1~3000 电机 RPM，只影响后续运动，不立即写设备。",
                          "r 相对 / a 绝对输出轴度；总轴位 ±60000 电机圈（手册范围）。",
                          "运动中单线程维持 CAN 心跳并刷新实测 0x6064，位置与目标分别显示。",
                          "空格/x 停止并等待静止确认；停稳后可直接继续运动。",
                          "R 手动刷新；q 退出；节点 127 只允许读取。",
                          "高级：m/M 按步长运动，[/] 选 5/15/30/90/360/3600/36000°。",
                          "高级：p 输入绝对电机脉冲；j/J 以所选 RPM 点动 250 ms。",
                          "断电前 d 失能、记下 0x6064；上电后只读比较，不预设记忆可用。",
                          "停止确认不等于机械制动器锁止。"};
        }
        view.panels.push_back(std::move(help));
        return view;
    }
    Panel live{};
    live.title = state.setup_mode ? "设备实时数据" : "运动与实测数据";
    add_status(live, snapshot, state, compact);
    if (!state.setup_mode) {
        const std::string selected = std::to_string(state.selected_speed_rpm) + " 电机 RPM / " +
            number(yz_aim::motor_to_output_rpm(state.selected_speed_rpm), 3) + " 输出 RPM";
        live.fields.insert(live.fields.begin(), {"选定速度", selected, Tone::warning});
        live.fields.insert(live.fields.begin(), {"运动", state.motion_active ? "进行中" : "空闲",
            state.motion_active ? Tone::warning : Tone::normal});
        if (snapshot.target_position_valid)
            live.fields.insert(live.fields.begin() + 2, {"本次目标",
                std::to_string(snapshot.target_position_pulses) + " 电机脉冲 / " +
                number(yz_aim::pulses_to_output_degrees(snapshot.target_position_pulses), 3) + " 输出度",
                Tone::warning});
        if (snapshot.position_sample.valid) {
            const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - snapshot.position_sample.sampled_at).count();
            live.fields.push_back({"位置采样", std::to_string(age) + " ms 前",
                age > 1000 ? Tone::warning : Tone::normal});
        }
        if (!compact) {
            live.lines = {"实测 0x6064 与本次目标分开显示；停止后再选择下次速度/角度。",
                          "位置单位：电机脉冲 / 输出度；51 电机圈 = 1 输出圈。"};
            if (snapshot.identity_sample.valid) {
                live.fields.insert(live.fields.begin(), {"设备类型", hex32(snapshot.device_type)});
                live.fields.insert(live.fields.begin() + 1, {"厂商 / 产品",
                    hex32(snapshot.vendor_id) + " / " + hex32(snapshot.product_code)});
            }
        }
        if (!state.first_fault.empty()) {
            live.fields.push_back({"首个故障", state.first_fault, Tone::error});
            live.lines.insert(live.lines.begin(), "最新操作：" + state.message);
        }
        view.panels.push_back(std::move(live));
        return view;
    }
    if (!compact && snapshot.identity_sample.valid) {
        live.fields.insert(live.fields.begin(), {"设备类型", hex32(snapshot.device_type)});
        live.fields.insert(live.fields.begin() + 1, {"厂商 / 产品",
            hex32(snapshot.vendor_id) + " / " + hex32(snapshot.product_code)});
    }
    view.panels.push_back(std::move(live));
    Panel control{};
    control.title = "配置状态";
    control.fields = {{"运行速度", state.profile_speed},
                      {"加速度", state.acceleration},
                      {"电流限制", state.current_limit},
                      {"待配置节点", state.staged_node},
                      {"待配置波特率", state.staged_bitrate}};
    if (!compact) control.lines = {
        "参数显示本次会话最近确认值；设置不会自动保存。",
        "节点或波特率配置需整条总线断电重启后重新连接。"};
    if (!state.first_fault.empty()) {
        control.fields.push_back({"首个故障", state.first_fault, Tone::error});
        control.lines.insert(control.lines.begin(), "最新操作：" + state.message);
    }
    view.panels.push_back(std::move(control));
    if (compact) return view;
    Panel notes{};
    notes.title = "读数说明";
    notes.lines = {"本设备减速比按用户提供的 51:1 显示。",
                   "一圈输出对应 1,671,168 个电机侧 CAN 脉冲，实际精度需单机核验。",
                   "停止和输出失能含义不同，也都不确认机械制动器状态。",
                   "其他波特率及负载运动尚未完成现场验收。"};
    view.panels.push_back(std::move(notes));
    return view;
}

std::string result_text(const Result& result)
{
    if (result) return "成功";
    if (result.error == Error::abort_reply)
        return "失败：从站拒绝 SDO，终止码 " + hex32(result.abort_code) +
            (result.context.empty() ? "" : "；" + result.context);
    static const std::map<std::string, std::string> messages = {
        {"ambiguous SDO session; reconnect transport", "SDO 应答不确定，请重新连接设备"},
        {"receive failed during prequeue drain", "清理接收队列时通信失败"},
        {"pre-transaction receive queue did not drain within bounded budget", "接收队列未及时排空"},
        {"SDO deadline expired before send", "SDO 发送前超时"},
        {"CAN send failed", "CAN 发送失败"},
        {"CAN receive failed", "CAN 接收失败"},
        {"truncated SDO reply", "SDO 应答长度不足"},
        {"SDO DLC must be 8", "SDO 应答帧长度不是 8"},
        {"invalid SDO download acknowledgement", "SDO 写入确认无效"},
        {"SDO upload width mismatch", "SDO 读取宽度不匹配"},
        {"SDO reply timed out or receive limit exhausted; reconnect", "SDO 应答超时，请重新连接设备"},
        {"snapshot deadline expired", "状态采样超时"},
        {"identity deadline expired", "设备身份读取超时"},
        {"optional telemetry deadline expired", "可选遥测读取超时"},
        {"post-write readback mismatches requested value", "写入后读回与目标不一致"},
        {"output-enable bit in 0x2601 readback mismatches requested value", "0x2601 使能位读回与目标不一致"},
        {"limit observed after hold", "停止后仍检测到限位"},
        {"hold observed a non-communication alarm", "停止后仍有非通信报警"},
        {"alarm observed after hold", "停止后仍有报警"},
        {"hold ACK received but stationary readback unconfirmed", "停止已应答，但静止读回未确认"},
        {"unsafe state latched; operator intervention required", "安全故障已锁定，需现场检查"},
        {"unrecognized YZ-AIM vendor/product/device type", "设备身份与 YZ-AIM 不符"},
        {"stale motor evidence", "设备状态采样过期"},
        {"drive reports alarm", "驱动器报告报警"},
        {"drive reports position limit", "驱动器报告位置限位"},
        {"motor is moving", "电机仍在运动"},
        {"disable drive output first", "请先关闭驱动输出"},
        {"operator cancelled motion", "已请求取消运动"},
        {"operator cancelled jog", "已请求取消点动"},
        {"fresh identity/output/speed evidence required before explicit stop", "停止前缺少有效的身份、输出或转速采样"},
        {"output disabled but motor still moving; use physical stop", "输出已关闭但电机仍运动，请采用现场停机措施"},
        {"arm with fresh stationary evidence first", "缺少新鲜静止状态，不能使能"},
        {"unexpected alarm during heartbeat recovery", "心跳恢复时出现意外报警"},
        {"communication alarm did not clear after guarded heartbeat", "受控心跳后通信报警未解除"},
        {"profile speed must be 0..3000 RPM", "运行速度需在 0~3000 电机 RPM 之间"},
        {"acceleration must be 10..1000 RPM/s; low values overlap limit setup", "加速度需在 10~1000 电机 RPM/s 之间"},
        {"current limit must be 10..10000mA, multiples of 10", "电流需在 10~10000 mA 之间，且是 10 的整数倍"},
        {"watchdog heartbeat deadline expired", "心跳发送超时"},
        {"watchdog service overdue; no heartbeat sent", "心跳服务已逾期，未继续发送心跳"},
        {"watchdog heartbeat send failed", "心跳发送失败"},
        {"owned watchdog stopped; reconnect and inspect motor", "会话心跳已停止，请重新连接并检查电机"},
        {"existing watchdog producer is not node 127", "现有心跳生产者不是节点 127"},
        {"motor status fault/limit", "运动中发生设备故障或限位"},
        {"motor alarm during motion", "运动中驱动器报警"},
        {"actual speed exceeds manual maximum", "实际速度超过手册上限"},
        {"motion deadline expired", "运动等待超时"},
        {"output and explicit arm required", "请先按 e 使能；运动需要已确认的输出和新鲜安全检查"},
        {"relative move requires signed32 nonzero delta, 1..3000 RPM and future deadline", "相对位移须为非零有符号32位脉冲，运动速度须为 1~3000 电机 RPM 且截止时间有效"},
        {"relative target exceeds manual motor-position range", "相对目标超出手册电机轴位范围"},
        {"actual position outside manual motor-position range", "实际位置已超出手册电机轴位范围"},
        {"absolute move requires manual-range target, future deadline and 1..3000 RPM", "绝对目标、速度或截止时间不符合手册范围"},
        {"absolute target equals fresh actual position", "目标与最新实测位置相同"},
        {"position action deadline expired during safety checks", "位置动作安全检查耗尽时间预算"},
        {"jog: abs speed 1..3000 RPM, 1..5000ms, future deadline", "点动速度绝对值须为 1~3000 电机 RPM、持续 1~5000 ms 且截止时间有效"},
        {"jog travel and stopping distance exceed manual motor-position range", "点动行程及预计停机距离将超出手册电机轴位范围"},
        {"jog actual position exceeds manual motor-position range", "点动实测位置已超出手册电机轴位范围"},
        {"jog status fault/limit", "点动中发生故障或限位"},
        {"jog alarm", "点动中驱动器报警"},
        {"jog speed exceeds manual maximum", "点动时速度超过手册上限"},
        {"node save did not report completion", "节点保存未报告完成"},
    };
    const auto found = messages.find(result.message);
    if (found != messages.end()) return "失败：" + found->second +
        (result.context.empty() ? "" : "；" + result.context);
    const char* category = "设备操作";
    switch (result.error) {
    case Error::invalid_argument: category = "参数无效"; break;
    case Error::transport: category = "CAN 通信失败"; break;
    case Error::timeout: category = "通信或动作超时"; break;
    case Error::malformed_reply: category = "设备应答格式错误"; break;
    case Error::unsafe: category = "安全条件未满足"; break;
    case Error::stale: category = "采样或读回不一致"; break;
    case Error::alarm: category = "驱动器报警"; break;
    case Error::limit: category = "位置限位"; break;
    case Error::moving: category = "电机未静止"; break;
    case Error::cancelled: category = "操作已取消"; break;
    case Error::none: case Error::abort_reply: break;
    }
    return std::string("失败：") + category +
        (result.message.empty() ? "" : "；原始诊断：" + result.message) +
        (result.context.empty() ? "" : "；" + result.context);
}

Deadline poll_deadline() { return Clock::now() + kPollBudget; }
Deadline action_deadline() { return Clock::now() + kActionBudget; }

std::unique_ptr<Transport> make_mock_transport(uint8_t node)
{
    return std::make_unique<UiMockTransport>(node);
}

void print_mock_audit(const Transport& transport, uint8_t node)
{
    const auto* mock = dynamic_cast<const MockTransport*>(&transport);
    if (!mock) return;
    std::size_t writes = 0, stop_writes = 0, output_disable_writes = 0;
    std::size_t heartbeat_frames = 0;
    std::vector<uint32_t> motion_speeds;
    std::vector<int32_t> position_targets;
    for (const Frame& frame : mock->sent) {
        if (frame.id == 0x77f && frame.dlc == 1 && frame.data[0] == 0x05)
            ++heartbeat_frames;
        if (frame.id != static_cast<uint16_t>(0x600 + node) || frame.dlc != 8 ||
            frame.data[0] == 0x40) continue;
        ++writes;
        const uint16_t index = static_cast<uint16_t>(frame.data[1]) |
            (static_cast<uint16_t>(frame.data[2]) << 8);
        const unsigned width = frame.data[0] == 0x2f ? 1 :
            frame.data[0] == 0x2b ? 2 : 4;
        uint32_t value = 0;
        for (unsigned i = 0; i < width; ++i)
            value |= static_cast<uint32_t>(frame.data[4 + i]) << (8 * i);
        if (index == 0x6040 && value == 0x010f) ++stop_writes;
        if (index == 0x2601 && value == 0) ++output_disable_writes;
        if (index == 0x6081) motion_speeds.push_back(value);
        if (index == 0x607a) position_targets.push_back(static_cast<int32_t>(value));
    }
    std::cerr << "MOCK_WRITE_COUNT=" << writes
              << " MOCK_STOP_010F_WRITES=" << stop_writes
              << " MOCK_OUTPUT_DISABLE_WRITES=" << output_disable_writes
              << " MOCK_HEARTBEAT_COUNT=" << heartbeat_frames;
    std::cerr << " MOCK_PROFILE_RPM_WRITES=";
    for (std::size_t i = 0; i < motion_speeds.size(); ++i)
        std::cerr << (i ? "," : "") << motion_speeds[i];
    std::cerr << " MOCK_POSITION_TARGET_WRITES=";
    for (std::size_t i = 0; i < position_targets.size(); ++i)
        std::cerr << (i ? "," : "") << position_targets[i];
    std::cerr << "\n";
}

int run_setup(Motor& motor, const Options& options)
{
    return run_loop(motor, options, true);
}

int run_test(Motor& motor, const Options& options)
{
    return run_loop(motor, options, false);
}

} // namespace yz_aim::tool
