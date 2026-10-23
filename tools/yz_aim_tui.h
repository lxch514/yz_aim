#pragma once

#include "maintenance_tui.h"
#include "yz_aim.h"

#include <chrono>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace yz_aim::tool {

struct UiState {
    bool help = false;
    bool armed = false;
    bool output_raw_valid = false;
    uint16_t output_raw = 0;
    bool output_owned = false;
    bool watchdog_failed = false;
    bool motion_active = false;
    bool mock = false;
    bool setup_mode = false;
    uint32_t selected_speed_rpm = 30; // local next-motion selection only
    std::size_t position_step_index = 0; // 5, 15, 30, 90, 360, 3600, 36000 output degrees
    std::string profile_speed = "未知";
    std::string acceleration = "未知";
    std::string current_limit = "未知";
    std::string staged_node = "无";
    std::string staged_bitrate = "无";
    std::string message = "启动时只读；按 ? 查看操作帮助。";
    std::string first_fault;
    std::vector<std::string> history;
};

struct TerminalSize { int columns = 80; int rows = 24; };

bool parse_terminal_size(const std::string& text, TerminalSize* size);
TerminalSize terminal_size();
maintenance_tui::Dashboard make_dashboard(const Snapshot& snapshot,
                                           const UiState& state,
                                           uint8_t node,
                                           uint32_t bitrate,
                                           const std::string& mode,
                                           int terminal_rows = 36);
std::string result_text(const Result& result);

constexpr std::chrono::milliseconds kPollBudget{1100};
constexpr std::chrono::milliseconds kActionBudget{5000};
Deadline poll_deadline();
Deadline action_deadline();

struct Options;
int run_setup(Motor& motor, const Options& options);
int run_test(Motor& motor, const Options& options);
std::unique_ptr<Transport> make_mock_transport(uint8_t node);
void print_mock_audit(const Transport& transport, uint8_t node);
} // namespace yz_aim::tool
