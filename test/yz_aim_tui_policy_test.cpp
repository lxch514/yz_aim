#include "yz_aim_cli.h"
#include "yz_aim_mock.h"
#include "yz_aim_tui.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
bool check(bool ok, const char* expression, int line)
{
    if (!ok) std::cerr << "CHECK failed at line " << line << ": " << expression << "\n";
    return ok;
}
#define CHECK(expression) do { if (!check(static_cast<bool>(expression), #expression, __LINE__)) return 1; } while (false)

std::string hex16(uint16_t value)
{
    std::ostringstream text;
    text << "0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << value;
    return text.str();
}

const maintenance_tui::KeyValue* output_field(const maintenance_tui::Panel& panel)
{
    const auto found = std::find_if(panel.fields.begin(), panel.fields.end(),
        [](const auto& entry) { return entry.key == "输出"; });
    return found == panel.fields.end() ? nullptr : &*found;
}

bool parse(std::vector<std::string> args, yz_aim::tool::Options* out)
{
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (auto& item : args) argv.push_back(item.data());
    std::string error;
    return yz_aim::tool::parse_options(static_cast<int>(argv.size()), argv.data(), out, &error);
}
}

int main()
{
    using namespace yz_aim;
    using namespace yz_aim::tool;

    Options options{};
    CHECK(parse({"yz_aim_test", "--mock", "--node", "127", "--no-mouse"}, &options));
    CHECK(options.mock && options.node == 127 && options.no_mouse);
    CHECK(!parse({"yz_aim_test", "--bitrate", "300000"}, &options));
    CHECK(!parse({"yz_aim_test", "--bitrate", "500000", "--node", "128"}, &options));
    CHECK(!parse({"yz_aim_test", "--bitrate", "500000", "--channel", "-1"}, &options));
    CHECK(!parse({"yz_aim_test"}, &options));
    CHECK(parse({"yz_aim_test", "--bitrate", "1000000", "--abit-timing", "0x01400106"}, &options));
    CHECK(options.bitrate == 1000000 && options.nominal_timing == 0x01400106);
    CHECK(!parse({"yz_aim_test", "--mock", "--abit-timing", "0xNOTHEX"}, &options));

    TerminalSize size{};
    CHECK(parse_terminal_size("80x24", &size) && size.columns == 80 && size.rows == 24);
    CHECK(parse_terminal_size("160x50", &size));
    CHECK(!parse_terminal_size("0x24", &size));
    CHECK(!parse_terminal_size("80x24junk", &size));

    auto transport = make_mock_transport(1);
    Motor motor(*transport, 1);
    const Snapshot snapshot = motor.snapshot(Clock::now() + std::chrono::seconds(2));
    CHECK(snapshot.result);
    CHECK(snapshot.identity_sample.valid);
    CHECK(snapshot.status_sample.valid && snapshot.position_sample.valid && snapshot.speed_sample.valid);
    CHECK(snapshot.device_type == 0x00020192 && snapshot.vendor_id == 0x331 && snapshot.product_code == 1);
    CHECK(snapshot.speed_rpm == 0.0 && snapshot.output_speed_rpm == 0.0);
    CHECK(snapshot.output_position_degrees == pulses_to_output_degrees(snapshot.position_pulses));
    CHECK(snapshot.battery_sample.valid && snapshot.battery_tenths == 8);
    const auto* mock = dynamic_cast<const MockTransport*>(transport.get());
    CHECK(mock != nullptr);
    CHECK(mock->sent.size() > 0);
    CHECK(motor.arm(Clock::now() + std::chrono::seconds(2)));
    CHECK(motor.disarm(Clock::now() + std::chrono::seconds(2)));
    for (const Frame& frame : mock->sent) {
        if (frame.id == 0x601) CHECK(frame.data[0] == 0x40);
    }

    UiState state{};
    state.mock = true;
    const auto dashboard = make_dashboard(snapshot, state, 1, 0, "模拟设备 / 无硬件");
    CHECK(dashboard.status == "模拟设备 / 无硬件");
    CHECK(dashboard.title.find("YZ-AIM") != std::string::npos);
    CHECK(dashboard.shortcuts.find("e 使能") != std::string::npos);
    CHECK(dashboard.shortcuts.find("v 速度") != std::string::npos);
    CHECK(dashboard.shortcuts.find("r 相对角度") != std::string::npos);
    CHECK(dashboard.shortcuts.find("a 绝对角度") != std::string::npos);
    CHECK(dashboard.subtitle.find("v 选速度") != std::string::npos);
    CHECK(dashboard.panels.size() == 1);
    const auto has_key = [](const maintenance_tui::Dashboard& view, const std::string& key) {
        return std::any_of(view.actions.begin(), view.actions.end(),
            [&](const auto& action) { return action.key == key; });
    };
    for (const char* key : {"空格", "e", "d", "v", "r", "a", "?"})
        CHECK(has_key(dashboard, key));
    UiState expanded = state;
    expanded.position_step_index = 3;
    const auto expanded_view = make_dashboard(snapshot, expanded, 1, 0, "模拟设备 / 无硬件", 24);
    CHECK(expanded_view.panels.size() == 1);
    CHECK(std::any_of(expanded_view.panels[0].fields.begin(), expanded_view.panels[0].fields.end(),
        [](const auto& field) { return field.key == "选定速度" &&
            field.value.find("30 电机 RPM") != std::string::npos; }));
    expanded.selected_speed_rpm = kMaxMotorRpm;
    expanded.position_step_index = 6;
    const auto maximum_view = make_dashboard(snapshot, expanded, 1, 0, "模拟设备 / 无硬件", 24);
    CHECK(std::any_of(maximum_view.panels[0].fields.begin(), maximum_view.panels[0].fields.end(),
        [](const auto& field) { return field.key == "选定速度" &&
            field.value.find("3000 电机 RPM") != std::string::npos; }));
    CHECK(result_text({Error::invalid_argument, 0, "profile speed must be 0..3000 RPM"}) ==
          "失败：运行速度需在 0~3000 电机 RPM 之间");
    CHECK(result_text({Error::invalid_argument, 0, "relative target exceeds manual motor-position range"}).find(
          "手册电机轴位范围") != std::string::npos);
    CHECK(result_text({Error::invalid_argument, 0, "absolute target equals fresh actual position"}).find(
          "最新实测位置相同") != std::string::npos);
    expanded.selected_speed_rpm = 6;
    const auto slower_view = make_dashboard(snapshot, expanded, 1, 0, "模拟设备 / 无硬件", 24);
    CHECK(std::any_of(slower_view.panels[0].fields.begin(), slower_view.panels[0].fields.end(),
        [](const auto& field) { return field.key == "选定速度" &&
            field.value.find("6 电机 RPM") != std::string::npos; }));
    Snapshot underway = snapshot;
    underway.target_position_valid = true;
    underway.target_position_pulses = snapshot.position_pulses + 69632;
    underway.position_pulses += 4096;
    underway.output_position_degrees = pulses_to_output_degrees(underway.position_pulses);
    expanded.motion_active = true;
    const auto underway_view = make_dashboard(underway, expanded, 1, 0, "模拟设备 / 无硬件", 24);
    CHECK(underway_view.panels.size() == 1);
    CHECK(std::any_of(underway_view.panels[0].fields.begin(), underway_view.panels[0].fields.end(),
        [](const auto& field) { return field.key == "本次目标" &&
            field.value.find("70832") != std::string::npos; }));
    CHECK(std::any_of(underway_view.panels[0].fields.begin(), underway_view.panels[0].fields.end(),
        [](const auto& field) { return field.key == "位置" &&
            field.value.find("5296") != std::string::npos; }));
    UiState setup = state;
    setup.setup_mode = true;
    const auto setup_view = make_dashboard(snapshot, setup, 1, 0, "模拟设备 / 无硬件", 24);
    for (const char* key : {"s", "a", "c", "n", "b"})
        CHECK(has_key(setup_view, key));
    const auto readonly = make_dashboard(snapshot, state, 127, 0, "模拟设备 / 无硬件", 24);
    CHECK(!has_key(readonly, "e") && !has_key(readonly, "m") && !has_key(readonly, "p") &&
          !has_key(readonly, "空格"));
    CHECK(result_text({Error::none, 0, {}}) == "成功");
    CHECK(result_text({Error::abort_reply, 0x06090011, "SDO abort"}).find("0x06090011") != std::string::npos);
    const auto timeout_text = result_text({Error::timeout, 0, "SDO reply timed out or receive limit exhausted; reconnect",
                                           "read 0x6064:0 elapsed=80ms budget=80ms"});
    CHECK(timeout_text.find("SDO 应答超时") != std::string::npos);
    CHECK(timeout_text.find("read 0x6064:0 elapsed=80ms budget=80ms") != std::string::npos);
    const auto abort_text = result_text({Error::abort_reply, 0x06090011, "SDO abort",
                                         "write 0x2601:0 phase=reply"});
    CHECK(abort_text.find("write 0x2601:0 phase=reply") != std::string::npos);
    UiState diagnosed = state;
    diagnosed.message = "安全检查并使能：成功；安全检查并使能后采样：失败：状态采样超时";
    diagnosed.first_fault = "安全检查并使能后采样：失败：状态采样超时";
    const auto diagnosed_view = make_dashboard(snapshot, diagnosed, 1, 0,
                                                "模拟设备 / 无硬件", 24);
    CHECK(diagnosed_view.message.find("首个故障：") == 0);
    CHECK(diagnosed_view.message.find("后采样：失败") != std::string::npos);
    CHECK(std::any_of(diagnosed_view.panels[0].lines.begin(), diagnosed_view.panels[0].lines.end(),
        [](const std::string& line) { return line.find("动作") != std::string::npos ||
            line.find("最新操作：安全检查并使能：成功") != std::string::npos; }));
    for (const uint16_t raw : {uint16_t{0}, uint16_t{1}, uint16_t{5}, uint16_t{6},
                               uint16_t{7}, uint16_t{0xffff}}) {
        Snapshot sampled = snapshot;
        sampled.output_sample.valid = true;
        sampled.output_enabled = raw;
        UiState cached = state;
        cached.output_raw_valid = true;
        cached.output_raw = 0x0007; // a valid snapshot must override a stale cached raw value
        const auto view = make_dashboard(sampled, cached, 1, 0,
                                         "模拟设备 / 无硬件", 24);
        const bool enabled = output_is_enabled(raw);
        const std::string expected = std::string(enabled ? "已使能（原始值 " : "已失能（原始值 ") +
            hex16(raw) + "）";
        const auto* live_output = output_field(view.panels[0]);
        CHECK(live_output != nullptr && live_output->value == expected);
    }

    Snapshot invalid = snapshot;
    invalid.output_enabled = 7;
    invalid.output_sample.valid = false;
    UiState stale = state;
    stale.output_raw_valid = true;
    stale.output_raw = 7;
    const auto invalid_view = make_dashboard(invalid, stale, 1, 0,
                                             "模拟设备 / 无硬件", 24);
    const auto* live_output = output_field(invalid_view.panels[0]);
    CHECK(live_output != nullptr && live_output->value == "未知（旧值 0x0007）");
    CHECK(live_output->value.find("已使能") == std::string::npos);

    Snapshot comm_timeout = snapshot;
    comm_timeout.alarm_sample.valid = true;
    comm_timeout.alarm_code = 0x0020;
    const auto alarm_view = make_dashboard(comm_timeout, state, 1, 0,
                                           "模拟设备 / 无硬件", 24);
    const auto alarm = std::find_if(alarm_view.panels[0].fields.begin(),
        alarm_view.panels[0].fields.end(),
        [](const auto& entry) { return entry.key == "报警"; });
    CHECK(alarm != alarm_view.panels[0].fields.end());
    CHECK(alarm->value.find("通信心跳超时") != std::string::npos);
    CHECK(alarm->value.find("使能时恢复") != std::string::npos);

    std::cout << "yz_aim_tui_policy_test: PASS\n";
    return 0;
}
