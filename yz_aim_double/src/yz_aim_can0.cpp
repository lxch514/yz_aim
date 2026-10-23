// YZ-AIM 双轴命令与控制循环；USB构建通过 YZ_AIM_USB_DOUBLE 选择共享USB传输。
#include "yz_aim.h"
#include "yz_aim_sync.h"
#include "motion_log.h"
#include <filesystem>
#include "socketcan_transport.h"
#include "can_link_query.h"
#include "frame_trace.h"
#ifdef YZ_AIM_USB_DOUBLE
#include "lys_transport.h"
#include "shared_can_bus.h"
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>

namespace {

using yz_aim::Clock;
using yz_aim::Deadline;
using yz_aim::Error;
using yz_aim::Motor;
using yz_aim::Result;
using yz_aim::Snapshot;
using yz_aim::SocketCanTransport;

#ifdef YZ_AIM_USB_DOUBLE
constexpr const char* kProgram = "yz_aim_double";
#else
constexpr const char* kProgram = "yz_aim_can0";
#endif
constexpr const char* kVersion = "1.2.0-trace";
constexpr uint32_t kDefaultNode = 1;
constexpr uint32_t kDefaultBitrate = 1000000;
constexpr uint64_t kDefaultTimeoutMs = 20000;
constexpr int kInputPollMs = 100;
// 双轴同步的三个固定参数：连续超阈值拍数、到位判据、一轴先到位后的宽限。
constexpr int kSyncTripSamples = 3;
constexpr int kSyncArrivePulses = 16;
constexpr int64_t kSyncFinalGraceMs = 5000;

Motor* g_motor = nullptr;
volatile std::sig_atomic_t g_interrupted = 0;

void handle_signal(int) {
    g_interrupted = 1;
    if (g_motor) g_motor->request_cancel();
}

Deadline after_ms(uint64_t milliseconds) {
    return Clock::now() + std::chrono::milliseconds(milliseconds);
}

std::string fixed(double value, int digits) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(digits) << value;
    return out.str();
}

std::string hex_text(uint32_t value, int width) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << std::setw(width) << std::setfill('0') << value;
    return out.str();
}

std::string timestamp_text() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm local{};
    if (localtime_r(&seconds, &local) == nullptr) return "--:--:--";
    char clock_text[16] = {};
    std::strftime(clock_text, sizeof(clock_text), "%H:%M:%S", &local);
    std::ostringstream out;
    out << clock_text << "." << std::setw(3) << std::setfill('0') << milliseconds;
    return out.str();
}

bool parse_u32(const std::string& text, uint32_t minimum, uint32_t maximum, uint32_t& out) {
    if (text.empty() || text[0] == '-') return false;
    errno = 0;
    char* end = nullptr;
    const unsigned long value = std::strtoul(text.c_str(), &end, 10);
    if (errno || end == text.c_str() || *end != '\0') return false;
    if (value < minimum || value > maximum) return false;
    out = static_cast<uint32_t>(value);
    return true;
}

bool parse_i32(const std::string& text, int32_t minimum, int32_t maximum, int32_t& out) {
    if (text.empty()) return false;
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (errno || end == text.c_str() || *end != '\0') return false;
    if (value < minimum || value > maximum) return false;
    out = static_cast<int32_t>(value);
    return true;
}

bool parse_double(const std::string& text, double& out) {
    if (text.empty()) return false;
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (errno || end == text.c_str() || *end != '\0' || !std::isfinite(value)) return false;
    out = value;
    return true;
}

const char* error_text(Error error) {
    switch (error) {
        case Error::none: return "成功";
        case Error::invalid_argument: return "参数无效";
        case Error::transport: return "通信故障";
        case Error::timeout: return "超时";
        case Error::malformed_reply: return "应答格式或宽度不符";
        case Error::abort_reply: return "设备拒绝（SDO abort）";
        case Error::unsafe: return "安全门禁未通过";
        case Error::stale: return "读回值与请求不一致";
        case Error::alarm: return "驱动报警";
        case Error::limit: return "限位或坐标越界";
        case Error::moving: return "电机未静止";
        case Error::cancelled: return "已取消";
    }
    return "未知";
}

const char* error_hint(Error error, const std::string& message) {
    if (message.find("ambiguous") != std::string::npos)
        return "本会话此前有一次 SDO 写操作的确认没有收到（或应答格式不符）：这条写到底执行没执行无法判定，\n"
               "        整个 SDO 会话因此被判为不可信，之后每条命令都会被立刻拒绝 —— 这与电机有没有接、接线对不对都无关。\n"
               "        请 quit 后重新启动本工具；重启后一条一条敲，前一条失败就先 quit 重开，不要连着敲。\n"
               "        只读命令（sync-status / status / read-od）超时不会锁死会话，可以直接重试。";
    switch (error) {
        case Error::timeout:
#ifdef YZ_AIM_USB_DOUBLE
            return "检查USB连接、电机供电、节点号、CANH/CANL与120Ω终端电阻、--bitrate及--abit-timing；结合TX/RX帧和SDO诊断定位。";
#else
            return "检查：can0 波特率与电机一致、CANH/CANL 未接反、总线末端 120Ω 终端电阻、节点号正确；\n"
                   "        另外先看 ip -details link show can0 的 can state —— 若是 ERROR-PASSIVE/BUS-OFF，\n"
                   "        控制器收发已不可信（常见于之前发过帧没被应答），跑 sh tools/can0_up.sh can0 1000000 复位后再试。";
#endif
        case Error::transport:
#ifdef YZ_AIM_USB_DOUBLE
            return "检查USB适配器/SDK、供电与接线；TX表示SDK接受，不保证电机应答，查看TX_ERR与RX。";
#else
            return "检查接线与 ip -details -statistics link show can0 的收发计数、bus-off 状态。";
#endif
        case Error::alarm:
            return "读 0x260E 与红灯闪烁次数对照手册：0x20 通信掉线、0x10 电池掉电、0x12 堵转、0x14 失速、0x15 过压。";
        case Error::unsafe:
            return "本会话已锁止，且锁止不会自行复位：stop／disable 都会被拒绝。\n"
                   "        请 quit 退出后重新启动本工具（重新启动不会改变设备状态；输出由驱动器自身的\n"
                   "        心跳看护 —— 约 1 s 收不到 0x77F 就报 0x20 并停机）。";
        case Error::moving:
            return "确认电机真正静止（读 0x606C）；软件静止判定不等于机械制动。";
        default:
            return nullptr;
    }
}

bool report(const Result& result, const std::string& action) {
    if (result) {
        std::cout << "[成功] " << action << "\n";
        return true;
    }
    std::cout << "[失败] " << action << "：" << error_text(result.error);
    if (!result.message.empty()) std::cout << "（" << result.message << "）";
    std::cout << "\n";
    if (!result.context.empty()) std::cout << "        诊断 " << result.context << "\n";
    if (result.error == Error::abort_reply)
        std::cout << "        SDO abort 码 " << hex_text(result.abort_code, 8) << "\n";
    const char* hint = error_hint(result.error, result.message);
    if (hint) std::cout << "        建议 " << hint << "\n";
    return false;
}

std::string output_state_text(const Snapshot& snapshot) {
    if (!snapshot.output_sample.valid) return "未知";
    return hex_text(snapshot.output_enabled, 4) +
           (yz_aim::output_is_enabled(snapshot.output_enabled) ? "（已使能）" : "（已禁止）");
}

std::string field_or_dash(bool valid, const std::string& text) {
    return valid ? text : std::string("--");
}

void print_snapshot(const Snapshot& snapshot, uint8_t node, const std::string& interface_name) {
    std::cout << "节点 " << static_cast<unsigned>(node) << " @ " << interface_name << "   ";
    if (snapshot.identity_sample.valid)
        std::cout << "设备类型 " << hex_text(snapshot.device_type, 6)
                  << "  厂商 " << hex_text(snapshot.vendor_id, 3)
                  << "  产品代码 " << snapshot.product_code;
    else
        std::cout << "身份未读到";
    std::cout << "\n";
    std::cout << "  状态字 " << field_or_dash(snapshot.status_sample.valid, hex_text(snapshot.statusword, 4))
              << "   报警 " << field_or_dash(snapshot.alarm_sample.valid, hex_text(snapshot.alarm_code, 4))
              << "   输出 " << output_state_text(snapshot) << "\n";
    std::cout << "  位置 " << field_or_dash(snapshot.position_sample.valid, std::to_string(snapshot.position_pulses))
              << " 电机脉冲（" << field_or_dash(snapshot.position_sample.valid,
                                                 fixed(snapshot.output_position_degrees, 3)) << " 输出度）\n";
    std::cout << "  转速 " << field_or_dash(snapshot.speed_sample.valid, fixed(snapshot.speed_rpm, 1))
              << " 电机RPM（" << field_or_dash(snapshot.speed_sample.valid,
                                               fixed(snapshot.output_speed_rpm, 3)) << " 输出RPM）\n";
    std::cout << "  电流 " << field_or_dash(snapshot.current_sample.valid, fixed(snapshot.current_a, 3))
              << " A   电压 " << field_or_dash(snapshot.voltage_sample.valid, fixed(snapshot.voltage_v, 2))
              << " V   温度 " << field_or_dash(snapshot.temperature_sample.valid,
                                               std::to_string(snapshot.temperature_c))
              << " ℃   电池 " << field_or_dash(snapshot.battery_sample.valid,
                                               std::to_string(snapshot.battery_tenths * 10)) << " %\n";
}

std::string one_line_snapshot(const Snapshot& snapshot) {
    std::ostringstream out;
    out << "状态 " << field_or_dash(snapshot.status_sample.valid, hex_text(snapshot.statusword, 4))
        << " 报警 " << field_or_dash(snapshot.alarm_sample.valid, hex_text(snapshot.alarm_code, 4))
        << " 输出 " << output_state_text(snapshot)
        << " 位置 " << field_or_dash(snapshot.position_sample.valid, std::to_string(snapshot.position_pulses))
        << " 转速 " << field_or_dash(snapshot.speed_sample.valid, fixed(snapshot.speed_rpm, 1)) << "RPM"
        << " 电流 " << field_or_dash(snapshot.current_sample.valid, fixed(snapshot.current_a, 3)) << "A"
        << " 电压 " << field_or_dash(snapshot.voltage_sample.valid, fixed(snapshot.voltage_v, 2)) << "V"
        << " " << field_or_dash(snapshot.temperature_sample.valid, std::to_string(snapshot.temperature_c)) << "℃";
    return out.str();
}

// 运动过程中的实时进度，由驱动在同一个线程里回调。
void show_progress(const Snapshot& snapshot) {
    std::cout << "\r  " << field_or_dash(snapshot.position_sample.valid, std::to_string(snapshot.position_pulses))
              << " 脉冲（" << field_or_dash(snapshot.position_sample.valid,
                                            fixed(snapshot.output_position_degrees, 2)) << "°）  转速 "
              << field_or_dash(snapshot.speed_sample.valid, fixed(snapshot.speed_rpm, 1)) << " RPM";
    if (snapshot.target_position_valid) std::cout << "   目标 " << snapshot.target_position_pulses;
    std::cout << "      " << std::flush;
}

struct Options {
    std::string interface_name = "can0";
    uint32_t node = kDefaultNode;
    uint32_t node2 = 2; // 第二轴节点号；0 表示只用单轴
    uint32_t bitrate = kDefaultBitrate;
    uint64_t timeout_ms = kDefaultTimeoutMs;
    bool trace = true;
#ifdef YZ_AIM_USB_DOUBLE
    std::string library = "liblyscan.so";
    uint32_t device = 0, channel = 0, nominal_timing = 0;
#endif
    bool assume_yes = false;
    bool help = false;
    bool version = false;
    std::vector<std::string> command;
};

void print_usage() {
    std::cout <<
        "YZ-AIM " << kProgram << " " << kVersion << " 构建 " __DATE__ " " __TIME__ "\n"
        "用法：" << kProgram << " [选项] [命令 参数...]\n"
#ifdef YZ_AIM_USB_DOUBLE
        "USB选项：--library /路径/liblyscan.so --lys 0 --channel 0 --abit-timing 0x01400106\n"
#else
        "接口选项：--if can0\n"
#endif
        "节点选项：--node 1 --node2 2；默认持续打印收发帧到stderr，--no-trace可关闭。\n"
        "选项：--bitrate 1000000  --timeout 毫秒  --node2 0（单轴） --version --help\n"
        "默认双轴：节点2取正、节点1取负；启动只读，不自动使能或运动。\n\n"
        "双轴：\n"
        "  sync-enable / sync-disable         两轴安全使能 / 同时停止再失能\n"
        "  sync-zero                          两轴静止时记录固定软件零点；运动命令不会重置\n"
        "  extend <rpm> <ms>                  伸出：默认节点2 +、节点1 -\n"
        "  retract <rpm> <ms>                 缩进：默认节点2 -、节点1 +\n"
        "  sync-spin <带符号rpm> [ms]         同一速度控制路径，默认60000ms\n"
        "  sync-run <带符号rpm>               常转直到sync-stop；也需要sync-zero\n"
        "  sync-stop                          同时停止两轴并确认，结束CSV日志\n"
        "  sync-status                        两轴状态、固定零点偏差及同步修正设置\n"
        "  sync-correct <0~10RPM>             最大减速修正，默认5，另受名义速度10%限制\n"
        "  sync-tol <输出角度>                真实累计偏差保护，默认0.5度；0为显式关闭\n"
        "  sync-drift <RPM>                   速度差告警参考，不会增加累计偏差门限\n"
        "  sync-dir <正转节点> <反转节点>     默认2 1；只能静止时设置\n"
        "  sync-rel <输出角度> [rpm]          镜像相对位置运动\n"
        "  sync-abs <输出角度> [rpm]          以A轴坐标为基准的位置运动\n"
        "  set-lead <mm/输出圈> / sync-mm <mm> [rpm]\n"
        "  jog <带符号rpm> [ms]               双轴点动，默认500ms，上限5000ms\n"
        "  one <节点> <带符号rpm> <ms> / one-stop  指定一轴单动/两轴停止\n"
        "速度命令上限300电机RPM，时长1~600000ms。速度是名义上限，领先轴可小幅减速。\n"
        "速度运动前必须sync-enable、静止时sync-zero。所有速度动作写本目录logs/*.csv。\n\n"
        "单轴（--node2 0）：\n"
        "  enable / disable / stop\n"
        "  rel <输出角度> / abs <输出角度> / pulses <原始目标脉冲>\n"
        "  speed <rpm> / jog <rpm> [ms] / spin <rpm> [ms] / run <rpm>\n"
        "  set-profile-speed <rpm> / set-accel <RPM/s> / set-current <mA>\n"
        "  set-node <1~126>                   需--yes、--node2 0；自动停稳失能后写入保存\n"
        "  set-zero / set-bitrate <125|250|500|1000>  需--yes\n"
        "通用：status / monitor [ms] [间隔ms] / read-od <index> <sub> <宽度> / help / quit\n";
}
void print_help() {
    print_usage();
    std::cout <<
        "\n建议流程：sync-enable → sync-zero → extend 300 50000 → retract 300 50000 → sync-disable。\n"
        "sync-zero只记软件基准，不移动或写硬件零点；退出后重新确认，禁止运动中重新置零。\n"
        "位置按采样时刻对齐；不再每拍清除累计错位。修正只减速，不反向、不超过命令速度。\n"
        "到时先给两轴停止，再确认停稳；停止请求时刻与物理停稳时刻分别记入CSV。\n"
        "真实报警、限位、持续无新鲜反馈、危险累计超差仍停止；默认0.5度未自行放宽。\n"
        "改号时只接目标电机（两台同ID无法单独寻址），保存完成后仍需重新上电。\n";
}

bool parse_options(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto next_value = [&](const char* name, std::string& out) {
            if (i + 1 >= argc) {
                std::cerr << "选项 " << name << " 缺少参数\n";
                return false;
            }
            out = argv[++i];
            return true;
        };
        if (argument == "--help" || argument == "-h") { options.help = true; return true; }
        if (argument == "--version") { options.version = true; return true; }
        if (argument == "--no-trace") { options.trace = false; continue; }
#ifdef YZ_AIM_USB_DOUBLE
        if (argument == "--library") {
            if (!next_value("--library", options.library) || options.library.empty()) return false;
            continue;
        }
        if (argument == "--lys" || argument == "--channel" || argument == "--abit-timing") {
            std::string value;
            if (!next_value(argument.c_str(), value) || value.empty() || value[0]=='-') return false;
            char* end = nullptr;
            errno = 0;
            const unsigned long number = std::strtoul(value.c_str(), &end, 0);
            if (errno || *end || number > UINT32_MAX ||
                (argument=="--channel" && number > 1) || (argument=="--lys" && number > 255)) {
                std::cerr << "USB选项数值无效：" << argument << "\n"; return false;
            }
            if (argument=="--lys") options.device=static_cast<uint32_t>(number);
            else if (argument=="--channel") options.channel=static_cast<uint32_t>(number);
            else options.nominal_timing=static_cast<uint32_t>(number);
            continue;
        }
#endif
        if (argument == "--yes") { options.assume_yes = true; continue; }
#ifndef YZ_AIM_USB_DOUBLE
        if (argument == "--if") {
            if (!next_value("--if", options.interface_name)) return false;
            continue;
        }
#endif
        if (argument == "--node") {
            std::string value;
            if (!next_value("--node", value) || !parse_u32(value, 1, 127, options.node)) {
                std::cerr << "--node 需为 1~127 的整数\n";
                return false;
            }
            continue;
        }
        if (argument == "--node2") {
            std::string value;
            if (!next_value("--node2", value) || !parse_u32(value, 0, 126, options.node2)) {
                std::cerr << "--node2 需为 0~126 的整数（0 = 只用单轴；127 被本工具占用于发心跳）\n";
                return false;
            }
            continue;
        }
        if (argument == "--bitrate") {
            std::string value;
            if (!next_value("--bitrate", value) || !parse_u32(value, 1000, 1000000, options.bitrate)) {
                std::cerr << "--bitrate 需为整数（125000/250000/500000/1000000）\n";
                return false;
            }
            continue;
        }
        if (argument == "--timeout") {
            std::string value;
            uint32_t milliseconds = 0;
            if (!next_value("--timeout", value) || !parse_u32(value, 100, 600000, milliseconds)) {
                std::cerr << "--timeout 需为 100~600000 的整数（毫秒）\n";
                return false;
            }
            options.timeout_ms = milliseconds;

            continue;
        }
        // 第一个非选项参数起，后面一律按命令及其参数处理：否则 sync-rel -720 / sync-mm -10
        // 这类带负数的命令在单命令模式下会被当成未知选项（选项必须写在命令之前）。
        if (!options.command.empty()) {
            options.command.push_back(argument);
            continue;
        }
        if (!argument.empty() && argument[0] == '-' && argument != "-") {
            std::cerr << "未知选项：" << argument << "\n";
            return false;
        }
        options.command.push_back(argument);
    }
    return true;
}

struct Session {
    Session(Motor& motor_ref, yz_aim::Transport& transport_ref, uint8_t node_id,
            uint64_t budget_ms, bool yes, std::string label = "USB-CAN")
        : motor(motor_ref), transport(transport_ref), interface_label(std::move(label)), node(node_id),
          timeout_ms(budget_ms), assume_yes(yes) {}
    Session(Motor& motor_ref, SocketCanTransport& transport_ref, uint8_t node_id,
            uint64_t budget_ms, bool yes)
        : Session(motor_ref, static_cast<yz_aim::Transport&>(transport_ref), node_id,
                  budget_ms, yes, transport_ref.interface_name()) {}
    Motor& motor;
    yz_aim::Transport& transport;
    std::string interface_label;
    // B 轴自己的传输层（单轴模式为空）。两轴各用一条套接字，接收过滤器只放行本节点报文。
    yz_aim::Transport* transport2 = nullptr;
    uint8_t node = kDefaultNode;
    bool enabled = false;
    uint32_t motion_rpm = static_cast<uint32_t>(yz_aim::kPositionProfileRpm);
    uint64_t timeout_ms = kDefaultTimeoutMs;
    bool assume_yes = false;
    int command_rc = 0;
    bool trace_idle = false;
    yz_aim::VelocitySynchronizer synchronizer;
    double correction_rpm = 5.0;
    double max_error_deg = 0;
    double acceleration_a = 0, acceleration_b = 0;
    Snapshot previous_feedback_a{}, previous_feedback_b{};
    int32_t nominal_rpm = 0, target_rpm_a = 0, target_rpm_b = 0;
    uint32_t requested_ms = 0;
    Clock::time_point feedback_at{}, last_usable_feedback{}, stop_a_at{}, stop_b_at{}, stopped_at{};
    std::unique_ptr<yz_aim::MotionLog> motion_log;
    std::filesystem::path log_directory;
    std::string end_reason;
    Snapshot sustain_last{};
    Clock::time_point sustain_started{};
    Clock::time_point sustain_next_sweep{};
    Clock::time_point sustain_next_line{};
    // 双轴同步：second 为空即单轴模式。两轴各用自己的一条 transport（socket），
    // 所有 SDO 仍然严格串行 —— 本文件全部双轴操作都是单线程顺序调用，不引入后台线程。
    std::unique_ptr<yz_aim::Motor> second;
    uint8_t node2 = 0;
    bool dual_enabled = false;            // 两轴输出都由本会话持有
    // 两轴是镜像安装（转向相反），所以双轴命令一律一轴取正、一轴取负 —— 不存在"同向"这种配置。
    // plus_is_a 记录 A 轴（--node）拿到的是 + 还是 −，由 sync-dir 设置。
    // 现场按"节点 2 取 +、节点 1 取 −"接线，所以默认 A 轴（--node）取 −。
    bool plus_is_a = false;
    double sync_tolerance_deg = yz_aim::kSyncDefaultToleranceDeg; // 位置偏差阈值（输出角度）
    // 两轴位置之和最大可达 ±2×kMaxPositionPulses，已超出 int32，基准必须用 64 位存。
    int64_t sync_base_pulses = 0;         // 位置模式的固定目标指标；速度模式不再重取累计基准
    int64_t sync_origin_pulses = 0;       // 运动起点的指标，只用于显示累计偏差
    Clock::time_point sync_base_at{};     // 上面那个基准的取样时刻，门限只按它算"已跑多久"
    // 速度模式：门限里"允许的两轴速度差"（电机 RPM），乘自上次重取基准以来的时长；0 = 不放宽。
    double sync_drift_rpm = yz_aim::kSyncDefaultDriftRpm;
    Clock::time_point sync_move_started{}; // 本次双轴动作的启动时刻（只用于打印已运行时长）
    // 丝杆导程（mm/输出圈），由 set-lead 设置；0 = 未设置，sync-mm 拒绝执行。
    double lead_mm = yz_aim::kSyncDefaultLeadMm;
    // sync-zero 固定行程基准（会话级）：用于速度同步修正、累计保护和日志，动作不会重置。
    bool sync_zero_valid = false;
    int32_t sync_zero_a = 0;
    int32_t sync_zero_b = 0;
    // 本次动作起点的两轴原始 0x6064 与位置模式的目标坐标：动作结束后用它算各轴行程、
    // 到位残差与两端错位量（只用于显示）。
    bool sync_start_valid = false;
    int32_t sync_start_a = 0;
    int32_t sync_start_b = 0;
    int32_t sync_target_a = 0;
    int32_t sync_target_b = 0;
    // sync-run 常转式的持续同步状态，由主循环每 200 ms 看护。
    bool sync_run_active = false;
    int32_t sync_run_rpm = 0;
    int sync_run_over = 0;
    Clock::time_point sync_run_started{};
    Clock::time_point sync_run_next_tick{};
    Clock::time_point sync_run_next_line{};
};

enum class Disposition { keep_going, quit };

// 双轴常转看护（定义在双轴命令族里，这里先声明供 monitor 的内层等待循环调用）。
void sync_run_tick(Session& session, bool& prompt_shown);

// 常转期间除停止类命令外的写操作一律拒绝，避免半途改变设定点或误触发安全锁止。
bool blocked_by_sustain(Session& session, const std::string& name) {
    if (!session.motor.sustaining()) return false;
    std::cout << "[失败] " << name << "：常转进行中，请先输入 stop。\n";
    return true;
}

Disposition command_status(Session& session) {
    const Snapshot snapshot = session.motor.snapshot(after_ms(session.timeout_ms));
    print_snapshot(snapshot, session.node, session.interface_label);
    if (!snapshot.result) report(snapshot.result, "读取状态");
    else if (snapshot.unsafe_latched) std::cout << "[!] 本会话已锁止，只能 stop / disable / 退出。\n";
    return Disposition::keep_going;
}

Disposition command_monitor(Session& session, const std::vector<std::string>& arguments) {
    uint32_t total_ms = 10000;
    uint32_t period_ms = 500;
    if (!arguments.empty() && !parse_u32(arguments[0], 1, 3600000, total_ms)) {
        std::cout << "用法：monitor [毫秒] [间隔ms]\n";
        return Disposition::keep_going;
    }
    if (arguments.size() >= 2 && !parse_u32(arguments[1], 100, 5000, period_ms)) {
        std::cout << "用法：monitor [毫秒] [间隔ms]\n";
        return Disposition::keep_going;
    }
    const Deadline until = after_ms(static_cast<uint64_t>(total_ms));
    std::cout << "只读监视 " << total_ms << " ms，每 " << period_ms << " ms 一次…\n";
    while (Clock::now() < until) {
        if (g_interrupted) { std::cout << "收到中断，结束监视。\n"; break; }
        const Snapshot snapshot = session.motor.snapshot(after_ms(2000));
        if (!snapshot.result) { report(snapshot.result, "读取状态"); break; }
        std::cout << timestamp_text() << "  " << one_line_snapshot(snapshot) << std::endl;
        // 采样间隔可能长于心跳周期，等待期间必须继续维持所属心跳。
        const Deadline next = std::min(until, Clock::now() + std::chrono::milliseconds(period_ms));
        while (Clock::now() < next) {
            if (g_interrupted) break;
            if (session.motor.watchdog_active()) {
                const Result serviced = session.motor.service_watchdog(after_ms(kInputPollMs));
                if (!serviced) { report(serviced, "维持所属心跳"); return Disposition::keep_going; }
            }
            if (session.motor.sustaining()) {
                const Result patrolled = session.motor.sustain_service(after_ms(2000));
                if (!patrolled) {
                    std::cout << "\n";
                    report(patrolled, "常转看护");
                    return Disposition::keep_going;
                }
            }
            if (session.sync_run_active) {
                bool prompt_shown = false;
                sync_run_tick(session, prompt_shown);
        if (session.trace_idle) {
            // No SDO transaction is outstanding here; drain unsolicited frames for live RX tracing.
            const auto until=Clock::now()+std::chrono::milliseconds(2);
            for (auto* port : {&session.transport,session.transport2}) {
                if (!port) continue;
                yz_aim::Frame unsolicited;
                for(unsigned n=0;n<128 && Clock::now()<until;++n)
                    if(!port->receive(unsolicited,std::min(until,Clock::now()+std::chrono::microseconds(100)))) break;
            }
        }
                if (!session.sync_run_active) { std::cout << "\n"; return Disposition::keep_going; }
            }
            std::this_thread::sleep_until(std::min(next, Clock::now() + std::chrono::milliseconds(kInputPollMs)));
        }
    }
    std::cout << "监视结束。\n";
    return Disposition::keep_going;
}

Disposition command_read_od(Session& session, const std::vector<std::string>& arguments) {
    if (arguments.empty()) {
        std::cout << "用法：read-od <索引(十六进制)>[:<子索引>] [宽度 1|2|4]\n";
        return Disposition::keep_going;
    }
    const std::string& object = arguments[0];
    const std::size_t colon = object.find(':');
    const std::string index_text = colon == std::string::npos ? object : object.substr(0, colon);
    const std::string subindex_text = colon == std::string::npos ? "00" : object.substr(colon + 1);
    unsigned long index = 0;
    unsigned long subindex = 0;
    char* end = nullptr;
    index = std::strtoul(index_text.c_str(), &end, 16);
    if (end == index_text.c_str() || *end != '\0' || index > 0xffffUL) {
        std::cout << "索引需为十六进制 0~0xFFFF\n";
        return Disposition::keep_going;
    }
    subindex = std::strtoul(subindex_text.c_str(), &end, 16);
    if (end == subindex_text.c_str() || *end != '\0' || subindex > 0xffUL) {
        std::cout << "子索引需为十六进制 0~0xFF\n";
        return Disposition::keep_going;
    }

    std::vector<uint8_t> widths{4, 2, 1};
    if (arguments.size() >= 2) {
        uint32_t width = 0;
        if (!parse_u32(arguments[1], 1, 4, width) || (width != 1 && width != 2 && width != 4)) {
            std::cout << "宽度只能是 1、2 或 4\n";
            return Disposition::keep_going;
        }
        widths.assign(1, static_cast<uint8_t>(width));
    }

    const auto name = static_cast<uint16_t>(index);
    const auto sub = static_cast<uint8_t>(subindex);
    for (const uint8_t width : widths) {
        // 每次试探都用新的 SDO 会话：宽度不符会污染该会话，但不影响工具体本身的其余操作。
        yz_aim::SdoClient sdo(session.transport, session.node);
        uint32_t value = 0;
        const Result result = sdo.read(name, sub, width, value, after_ms(1500));
        if (result) {
            std::cout << "0x" << hex_text(name, 4).substr(2) << ":" << hex_text(sub, 2).substr(2)
                      << " 宽度 " << static_cast<unsigned>(width)
                      << " = " << hex_text(value, width * 2)
                      << "（无符号 " << value
                      << "，有符号 " << static_cast<int32_t>(value) << "）\n";
            return Disposition::keep_going;
        }
        if (result.error == Error::malformed_reply) continue;
        report(result, "读取对象字典");
        return Disposition::keep_going;
    }
    std::cout << "[失败] 读取对象字典：1/2/4 字节宽度都与设备应答不符\n";
    return Disposition::keep_going;
}

// 运动前统一做只读 ARM 门禁；输出所有权由 enable/disable 决定，且不在这里改变。
Disposition run_motion(Session& session, const std::string& action, uint64_t budget_ms,
                       const std::function<Result(Deadline)>& operation) {
    if (blocked_by_sustain(session, action)) return Disposition::keep_going;
    if (!session.enabled) {
        std::cout << "[失败] " << action << "：本会话尚未使能输出，先执行 enable。\n";
        return Disposition::keep_going;
    }
    if (!session.motor.armed()) {
        const Result armed = session.motor.arm(after_ms(session.timeout_ms));
        if (!report(armed, "运动前安全检查（ARM，只读）")) return Disposition::keep_going;
    }
    const Result result = operation(after_ms(budget_ms));
    std::cout << "\n";
    if (result.error == Error::cancelled) {
        // Ctrl-C 只用于打断这一次运动：此时电机已被确认停止并保持，会话继续可用。
        g_interrupted = 0;
        std::cout << "[已取消] " << action << "（" << result.message << "）\n"
                     "        电机已确认停止并保持，输出仍使能；本会话继续可用（输入 quit 退出）。\n";
        return Disposition::keep_going;
    }
    report(result, action);
    return Disposition::keep_going;
}

uint64_t position_budget_ms(int64_t delta_pulses, uint32_t rpm) {
    const auto budget = yz_aim::position_action_budget(delta_pulses, rpm);
    return static_cast<uint64_t>(budget.count()) + 5000;
}

Disposition command_relative(Session& session, const std::string& text) {
    if (!session.enabled) {
        std::cout << "[失败] 相对位置运动：本会话尚未使能输出，先执行 enable。\n";
        return Disposition::keep_going;
    }
    double degrees = 0;
    if (!parse_double(text, degrees)) {
        std::cout << "用法：rel <输出角度，度，可小数可负>\n";
        return Disposition::keep_going;
    }
    int32_t pulses = 0;
    const Result converted = yz_aim::output_revolutions_to_pulses(degrees / 360.0, pulses);
    if (!report(converted, "把输出角度换算成电机脉冲")) return Disposition::keep_going;
    if (pulses == 0) {
        std::cout << "换算结果是 0 脉冲，未发送命令。\n";
        return Disposition::keep_going;
    }
    std::cout << "请求相对位移 " << degrees << "° = " << pulses << " 电机脉冲，速度 "
              << session.motion_rpm << " 电机RPM\n";
    return run_motion(session, "相对位置运动", position_budget_ms(pulses, session.motion_rpm),
                      [&session, pulses](Deadline deadline) {
                          return session.motor.move_relative(pulses, deadline, session.motion_rpm, show_progress);
                      });
}

Disposition command_absolute(Session& session, const std::string& text, bool raw_pulses) {
    if (!session.enabled) {
        std::cout << "[失败] 绝对位置运动：本会话尚未使能输出，先执行 enable。\n";
        return Disposition::keep_going;
    }
    int32_t target = 0;
    if (raw_pulses) {
        if (!parse_i32(text, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max(), target)) {
            std::cout << "用法：pulses <电机脉冲绝对坐标>\n";
            return Disposition::keep_going;
        }
    } else {
        double degrees = 0;
        if (!parse_double(text, degrees)) {
            std::cout << "用法：abs <输出角度，度，可小数可负>\n";
            return Disposition::keep_going;
        }
        const Result converted = yz_aim::output_revolutions_to_pulses(degrees / 360.0, target);
        if (!report(converted, "把输出角度换算成电机脉冲")) return Disposition::keep_going;
    }
    if (target < yz_aim::kMinPositionPulses || target > yz_aim::kMaxPositionPulses) {
        std::cout << "[失败] 目标超出手册坐标范围 ±" << yz_aim::kMaxPositionPulses << " 电机脉冲。\n";
        return Disposition::keep_going;
    }
    // 先只读确认当前坐标：既用于显示前后对比，也用于估算本次动作的时间预算。
    const Snapshot before = session.motor.snapshot(after_ms(session.timeout_ms));
    if (!before.result) { report(before.result, "动作前读取位置"); return Disposition::keep_going; }
    if (!before.position_sample.valid) {
        std::cout << "[失败] 动作前位置不可用，未发送命令。\n";
        return Disposition::keep_going;
    }
    const int64_t delta = static_cast<int64_t>(target) - before.position_pulses;
    if (delta == 0) {
        std::cout << "目标与当前实测位置相同，未发送命令。\n";
        return Disposition::keep_going;
    }
    std::cout << "当前 " << before.position_pulses << " 脉冲（"
              << fixed(before.output_position_degrees, 3) << "°）→ 目标 " << target << " 脉冲（"
              << fixed(yz_aim::pulses_to_output_degrees(target), 3) << "°），增量 " << delta
              << " 脉冲，速度 " << session.motion_rpm << " 电机RPM\n";
    const std::string action = raw_pulses ? "绝对位置运动（原始脉冲）" : "绝对位置运动（输出角度）";
    return run_motion(session, action, position_budget_ms(delta, session.motion_rpm),
                      [&session, target](Deadline deadline) {
                          return session.motor.move_absolute(target, deadline, session.motion_rpm, show_progress);
                      });
}

Disposition command_jog(Session& session, const std::vector<std::string>& arguments) {
    if (!session.enabled) {
        std::cout << "[失败] 点动：本会话尚未使能输出，先执行 enable。\n";
        return Disposition::keep_going;
    }
    if (arguments.empty()) {
        std::cout << "用法：jog <rpm> [毫秒]\n";
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -static_cast<int32_t>(yz_aim::kMaxMotorRpm),
                   static_cast<int32_t>(yz_aim::kMaxMotorRpm), rpm) || rpm == 0) {
        std::cout << "jog 转速需为 ±1~±" << yz_aim::kMaxMotorRpm << " 电机RPM（不能为 0）\n";
        return Disposition::keep_going;
    }
    uint32_t duration_ms = 500;
    if (arguments.size() >= 2 && !parse_u32(arguments[1], 1, 5000, duration_ms)) {
        std::cout << "jog 时长需为 1~5000 毫秒\n";
        return Disposition::keep_going;
    }
    const auto budget = yz_aim::jog_action_budget(std::chrono::milliseconds(duration_ms));
    const uint64_t budget_ms = static_cast<uint64_t>(budget.count()) + 5000;
    std::cout << "点动 " << rpm << " 电机RPM，持续 " << duration_ms << " ms\n";
    return run_motion(session, "点动", budget_ms, [&session, rpm, duration_ms](Deadline deadline) {
        return session.motor.jog(rpm, std::chrono::milliseconds(duration_ms), deadline, show_progress);
    });
}

// 连续运转：与 jog 同一套速度模式时序，只放开 5 秒上限。运行期间命令输入被阻塞，
// 想提前停就按 Ctrl-C（本进程会立刻做确认停止并保持，会话继续可用）。
Disposition command_spin(Session& session, const std::vector<std::string>& arguments) {
    if (arguments.empty()) {
        std::cout << "用法：spin <rpm> [毫秒]   （rpm 正=正转、负=反转；毫秒 默认 60000、最长 600000）\n";
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -static_cast<int32_t>(yz_aim::kMaxMotorRpm),
                   static_cast<int32_t>(yz_aim::kMaxMotorRpm), rpm) || rpm == 0) {
        std::cout << "spin 转速需为 ±1~±" << yz_aim::kMaxMotorRpm << " 电机RPM（不能为 0）\n";
        return Disposition::keep_going;
    }
    uint32_t duration_ms = 60000;
    if (arguments.size() >= 2 && !parse_u32(arguments[1], 1, 600000, duration_ms)) {
        std::cout << "spin 时长需为 1~600000 毫秒\n";
        return Disposition::keep_going;
    }
    const auto duration = std::chrono::milliseconds(duration_ms);
    const auto budget = yz_aim::jog_action_budget(duration);
    const uint64_t budget_ms = static_cast<uint64_t>(budget.count()) + 5000;
    std::cout << "连续运转 " << rpm << " 电机RPM（" << fixed(yz_aim::motor_to_output_rpm(rpm), 3)
              << " 输出RPM），最多 " << duration_ms << " ms；按 Ctrl-C 可随时停。\n"
                 "  运行中请留意 0x6064 的实时位置：越接近 ±" << yz_aim::kMaxPositionPulses
              << " 电机脉冲（±60000 电机圈），驱动会主动停机并报错。\n";
    return run_motion(session, "连续运转", budget_ms, [&session, rpm, duration](Deadline deadline) {
        return session.motor.spin(rpm, duration, deadline, show_progress);
    });
}

// 常转：启动后立即回到提示符，由 run_session 的循环维持心跳与看护，直到 stop。
Disposition command_run(Session& session, const std::vector<std::string>& arguments) {
    if (arguments.empty()) {
        std::cout << "用法：run <rpm>   （rpm 正=正转、负=反转；一直转到输入 stop 为止）\n";
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -static_cast<int32_t>(yz_aim::kMaxMotorRpm),
                   static_cast<int32_t>(yz_aim::kMaxMotorRpm), rpm) || rpm == 0) {
        std::cout << "run 转速需为 ±1~±" << yz_aim::kMaxMotorRpm << " 电机RPM（不能为 0）\n";
        return Disposition::keep_going;
    }
    if (!session.enabled) {
        std::cout << "[失败] 常转：本会话尚未使能输出，先执行 enable。\n";
        return Disposition::keep_going;
    }
    if (!session.motor.armed()) {
        const Result armed = session.motor.arm(after_ms(session.timeout_ms));
        if (!report(armed, "常转前安全检查（ARM，只读）")) return Disposition::keep_going;
    }
    const Result started = session.motor.start_velocity(rpm, after_ms(session.timeout_ms), show_progress);
    std::cout << "\n";
    if (!report(started, "启动常转")) return Disposition::keep_going;
    session.sustain_started = Clock::now();
    session.sustain_next_sweep = Clock::now();
    session.sustain_next_line = Clock::now();
    std::cout << "已启动常转 " << rpm << " 电机RPM（" << fixed(yz_aim::motor_to_output_rpm(rpm), 3)
              << " 输出RPM），一直转到输入 stop 为止。\n"
                 "  运行期间每约 200 ms 发送心跳 0x77F:05 并复查状态字/报警/转速/位置，每秒打印一行状态；\n"
                 "  stop = 停止并保持（输出仍使能）；disable = 停止并失能；Ctrl-C 等同于 stop；quit 退出。\n";
    return Disposition::keep_going;
}

// 常转看护节拍：每 200 ms 复查一次，每 1 秒打印一行状态；任何看护被触发即停机并锁止会话。
void sustain_tick(Session& session, bool& prompt_shown) {
    if (!session.motor.sustaining()) return;
    const auto now = Clock::now();
    if (now < session.sustain_next_sweep) return;
    session.sustain_next_sweep = now + std::chrono::milliseconds(200);
    auto capture = [&session](const Snapshot& sample) { session.sustain_last = sample; };
    const Result result = session.motor.sustain_service(after_ms(2000), capture);
    if (!result) {
        prompt_shown = false;
        report(result, "常转看护");
        std::cout << "        常转已停止并锁止本会话：心跳已停发，驱动约 1 s 后报 0x20 自行停机。\n"
                     "        确认现场电机已停后，执行 disable 或 quit 退出本工具。\n";
        return;
    }
    if (now < session.sustain_next_line) return;
    session.sustain_next_line = now + std::chrono::seconds(1);
    prompt_shown = false;
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - session.sustain_started).count();
    std::cout << "[常转 " << elapsed << "s] " << one_line_snapshot(session.sustain_last) << "\n";
}

// ===== 双轴同步 =====
// 两个 Motor 共用同一条 transport，SDO 请求/应答必须严格串行。下面的操作全部单线程顺序执行，
// 不会在等待一轴应答的同时给另一轴发请求；心跳是单向帧，可以穿插在任意两条 SDO 之间。
// 同步点就是"两条触发帧背靠背发出"，间隔只有一个 SDO 往返（约 1~5 ms）。

// 输出角度换算成电机脉冲；换算失败或结果为 0 时返回 0。
int32_t sync_degrees_to_pulses(double degrees) {
    int32_t pulses = 0;
    const Result converted = yz_aim::output_revolutions_to_pulses(degrees / 360.0, pulses);
    if (!converted) return 0;
    return pulses;
}

// 偏差阈值换算成电机脉冲；返回 0 表示关闭监控。
double sync_tolerance_pulses(const Session& session) {
    if (session.sync_tolerance_deg <= 0) return 0;
    const double raw = session.sync_tolerance_deg *
        static_cast<double>(yz_aim::kOutputPulsesPerRevolution) / 360.0;
    return std::max(1.0, raw);
}


// 两轴的一致性指标：两轴镜像安装、转向相反，刷架正常移动时一个位置增大、另一个减小，
// 所以用 posA + posB 之和判断同步，而不是差 —— 用差会把正常运动当成两倍行程的超差。
int64_t sync_pair_metric(int64_t a, int64_t b) {
    return a + b;
}

// 两轴位置是先后两次采样到的：采样时差在 300 电机RPM 下就有上万脉冲，直接相减会把
// "采样时差"当成"位置偏差"，几拍就误跳。这里按各自的采样时刻、用当拍转速把位置外推到
// 较晚的那个采样时刻再比较；外推本身有残差，但量级从"上万脉冲"降到"几百脉冲"。
void sync_align_positions(const Snapshot& a, const Snapshot& b, int64_t& pa, int64_t& pb) {
    pa = a.position_pulses;
    pb = b.position_pulses;
    if (!a.position_sample.valid || !b.position_sample.valid) return;
    const auto ta = a.position_sample.sampled_at;
    const auto tb = b.position_sample.sampled_at;
    const auto latest = ta > tb ? ta : tb;
    const double pulses_per_second_per_rpm =
        static_cast<double>(yz_aim::kMotorPulsesPerRevolution) / 60.0;
    pa += static_cast<int64_t>(a.speed_rpm * pulses_per_second_per_rpm *
                               std::chrono::duration<double>(latest - ta).count());
    pb += static_cast<int64_t>(b.speed_rpm * pulses_per_second_per_rpm *
                               std::chrono::duration<double>(latest - tb).count());
}

// A 轴在镜像对里的方向符号（+1 或 -1）。B 轴永远取反，一条命令必须一正一负。
int32_t sync_sign_a(const Session& session) {
    return session.plus_is_a ? 1 : -1;
}

// 镜像对的当前配置文字，用于打印。
std::string sync_dir_text(const Session& session) {
    std::ostringstream out;
    out << "节点 " << static_cast<unsigned>(session.node) << (session.plus_is_a ? "(+)" : "(-)")
        << " / 节点 " << static_cast<unsigned>(session.node2) << (session.plus_is_a ? "(-)" : "(+)");
    return out.str();
}

// 差值的绝对值。
int64_t sync_abs_diff(int64_t a, int64_t b) {
    const int64_t delta = a - b;
    return delta < 0 ? -delta : delta;
}

// 行程文字：<脉冲> 脉冲（<输出角度>°[ / <毫米> mm]）。毫米只在会话里设过导程时才给 ——
// 没有导程的毫米是假数，宁可不出。
std::string travel_text(double lead_mm, int64_t pulses) {
    std::ostringstream out;
    out << pulses << " 脉冲（"
        << fixed(static_cast<double>(pulses) * 360.0 /
                     static_cast<double>(yz_aim::kOutputPulsesPerRevolution), 4)
        << "°";
    if (lead_mm > 0) {
        out << " / "
            << fixed(static_cast<double>(pulses) /
                         static_cast<double>(yz_aim::kOutputPulsesPerRevolution) * lead_mm, 4)
            << " mm";
    }
    out << "）";
    return out.str();
}

std::filesystem::path motion_log_directory() {
    std::error_code ec;
    auto path = std::filesystem::canonical("/proc/self/exe", ec).parent_path();
    for (int i = 0; !ec && i < 5; ++i, path = path.parent_path())
        if (std::filesystem::exists(path / "src/yz_aim_can0.cpp") &&
            std::filesystem::exists(path / "build.sh")) return path / "logs";
    return {}; // never silently write logs to an unrelated working directory
}

bool sync_log_sample(Session& session, const Snapshot& a, const Snapshot& b, const char* event) {
    if (!session.motion_log) return true;
    const auto now = Clock::now();
    const auto f = yz_aim::aligned_feedback(a, b, now, session.acceleration_a,
                                           session.acceleration_b, std::max(1,std::abs(session.nominal_rpm)));
    const int sign = sync_sign_a(session);
    const int64_t xa = sign * (f.position_a - session.sync_zero_a);
    const int64_t xb = -sign * (f.position_b - session.sync_zero_b);
    if (f.usable) session.max_error_deg = std::max(session.max_error_deg,
        std::abs(double(xb-xa)*360.0/yz_aim::kOutputPulsesPerRevolution));
    auto age = [now](const yz_aim::Reading& r) -> double {
        return r.valid ? std::chrono::duration<double, std::milli>(now-r.sampled_at).count() : -1.0;
    };
    std::ostringstream row;
    row << std::fixed << std::setprecision(3)
        << std::chrono::duration<double, std::milli>(now-session.sync_move_started).count() << ',' << event
        << ',' << unsigned(session.node) << ',' << unsigned(session.node2)
        << ',' << session.target_rpm_a << ',' << session.target_rpm_b
        << ',' << a.speed_rpm << ',' << b.speed_rpm << ',' << a.position_pulses << ',' << b.position_pulses
        << ',' << f.position_a << ',' << f.position_b << ',' << xa << ',' << xb << ',' << xb-xa
        << ',' << double(xb-xa)*360.0/yz_aim::kOutputPulsesPerRevolution
        << ',' << f.sample_skew_ms << ',' << age(a.position_sample) << ',' << age(b.position_sample)
        << ',' << age(a.speed_sample) << ',' << age(b.speed_sample)
        << ',' << session.acceleration_a << ',' << session.acceleration_b
        << ',' << (f.usable ? "ok" : "stale_or_retried")
        << ',' << a.statusword << ',' << b.statusword << ',' << a.alarm_code << ',' << b.alarm_code;
    return session.motion_log->append(row.str());
}

void sync_finish_log(Session& session, const std::string& reason,
                     const Snapshot* a = nullptr, const Snapshot* b = nullptr) {
    if (!session.motion_log) return;
    if (a && b) (void)sync_log_sample(session, *a, *b,
        session.stopped_at == Clock::time_point{} ? "stop_unconfirmed" : "stopped");
    auto relative_ms = [&](Clock::time_point t) {
        return t == Clock::time_point{} ? -1LL : static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(t-session.sync_move_started).count());
    };
    std::ostringstream footer;
    footer << "# requested_ms=" << session.requested_ms
           << " trigger_a_ms=" << relative_ms(session.motor.velocity_triggered_at())
           << " trigger_b_ms=" << relative_ms(session.second->velocity_triggered_at())
           << " stop_request_a_ms=" << relative_ms(session.stop_a_at)
           << " stop_request_b_ms=" << relative_ms(session.stop_b_at)
           << " confirmed_at_ms=" << relative_ms(session.stopped_at)
           << " max_valid_error_deg=" << session.max_error_deg;
    (void)session.motion_log->append(footer.str());
    session.motion_log->finish(reason);
    std::cout << "    CSV日志：" << session.motion_log->path()
              << (session.motion_log->failed() ? " [写入失败，请检查存储]" : "") << "\n"
              << "    " << footer.str().substr(2) << "\n";
    session.motion_log.reset();
}

void sync_report_move(Session& session, const std::string& name, bool velocity_mode) {
    if (!session.second || !session.sync_start_valid) return;
    session.sync_start_valid = false; // 一次动作只汇总一次，避免收尾时重复打印
    const auto at = Clock::now();
    const int64_t elapsed_ms =
        session.sync_move_started.time_since_epoch().count() == 0
            ? 0
            : std::chrono::duration_cast<std::chrono::milliseconds>(at - session.sync_move_started)
                  .count();
    // 收尾要的是"停止之后"的位置，所以重新读一次；读不到就只报时长，不猜偏差。
    const Snapshot a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot b = session.second->snapshot(after_ms(session.timeout_ms));
    if (!a.result || !b.result) {
        std::cout << "[本次动作汇总] " << name << "，实际运行 " << elapsed_ms
                  << " ms；收尾读数读取失败，本次偏差未汇总（执行 sync-status 可以再看）。\n";
        sync_finish_log(session, session.end_reason.empty() ? name : session.end_reason);
        return;
    }
    const int64_t travel_a = static_cast<int64_t>(a.position_pulses) - session.sync_start_a;
    const int64_t travel_b = static_cast<int64_t>(b.position_pulses) - session.sync_start_b;
    const int64_t metric = sync_pair_metric(a.position_pulses, b.position_pulses);
    const double base_tolerance = sync_tolerance_pulses(session);
    // 相对 sync-zero 记录的行程基准，两端又错开了多少（只在记过基准时打印）。
    auto zero_line = [&]() {
        if (!session.sync_zero_valid) return;
        const int64_t tilt =
            metric - (static_cast<int64_t>(session.sync_zero_a) + session.sync_zero_b);
        std::cout << "    相对 sync-zero 基准累计错位 " << travel_text(session.lead_mm, tilt) << "\n";
    };
    if (!velocity_mode) std::cout << "[本次动作汇总] " << name << "，汇总距触发 " << elapsed_ms << " ms\n"
              << "    行程 A " << travel_text(session.lead_mm, travel_a) << "   B "
              << travel_text(session.lead_mm, travel_b)
              << "   ← 镜像正常时两者符号相反、大小接近\n";
    if (velocity_mode) {
        if (!session.motion_log) {
            std::cout << "[本次单轴动作汇总] " << name << "，两端指标相对动作起点变化 "
                      << travel_text(session.lead_mm, metric-session.sync_origin_pulses) << "\n";
            zero_line();
            return;
        }
        (void)sync_log_sample(session, a, b, "final_position");
        const int64_t fixed_error = -sync_sign_a(session) *
            (metric - (static_cast<int64_t>(session.sync_zero_a) + session.sync_zero_b));
        std::cout << "\n[本次动作汇总] " << name << "，结束原因："
                  << (session.end_reason.empty() ? name : session.end_reason) << "\n"
                  << "    行程 A " << travel_text(session.lead_mm, travel_a)
                  << " / B " << travel_text(session.lead_mm, travel_b) << "\n"
                  << "    相对固定零点的两端偏差 " << travel_text(session.lead_mm, fixed_error)
                  << "；最大有效偏差 " << fixed(session.max_error_deg, 4) << "°\n";
        zero_line();
        sync_finish_log(session, session.end_reason.empty() ? name : session.end_reason, &a, &b);
        return;
    }
    const int64_t rem_a = static_cast<int64_t>(session.sync_target_a) - a.position_pulses;
    const int64_t rem_b = static_cast<int64_t>(session.sync_target_b) - b.position_pulses;
    const int64_t deviation = rem_a - rem_b;
    const int64_t magnitude = sync_abs_diff(rem_a, rem_b);
    std::cout << "    到位残差 A " << rem_a << " 脉冲 / B " << rem_b << " 脉冲；判据量（两轴残差之差）"
              << travel_text(session.lead_mm, deviation) << "\n";
    zero_line();
    if (base_tolerance <= 0) {
        std::cout << "    偏差监控已关闭（sync-tol 0）：上面的数只是记录，不会触发急停。\n";
        return;
    }
    std::cout << "    基础阈值 " << fixed(session.sync_tolerance_deg, 4) << "° = "
              << static_cast<int64_t>(base_tolerance) << " 电机脉冲\n";
    if (static_cast<double>(magnitude) <= base_tolerance) {
        std::cout << "    结论：两轴到位偏差在基础阈值内，本次同步到位正常。\n";
    } else {
        std::cout << "    结论：两轴到位偏差超过基础阈值，请查两轴负载与速度环。\n";
    }
}

// 两轴同时急停：先各发一帧未确认急停（彼此相隔约 1 ms），再交替复查直到停稳，最后各自确认。
// 关键点是不能串行地"先确认 A 再确认 B" —— 那会让 B 多跑最多 7 秒。
bool sync_halt_both(Session& session) {
    session.acceleration_a = session.acceleration_b = 0;
    session.motor.clear_motion_deadline();
    if (session.second) session.second->clear_motion_deadline();
    session.stop_a_at = Clock::now();
    const Result sent_a = session.motor.halt_unconfirmed(after_ms(2000));
    session.stop_b_at = Clock::now();
    const Result sent_b = session.second ? session.second->halt_unconfirmed(after_ms(2000)) : Result{};
    const auto until = Clock::now() + std::chrono::milliseconds(8000);
    while (Clock::now() < until) {
        Snapshot a{}, b{};
        const bool ok_a = static_cast<bool>(session.motor.poll_safety(a, std::min(until, after_ms(1500))));
        const bool ok_b = !session.second || static_cast<bool>(session.second->poll_safety(b, std::min(until, after_ms(1500))));
        if (ok_a && ok_b && std::abs(a.speed_rpm) <= 0.5 &&
            (!session.second || std::abs(b.speed_rpm) <= 0.5)) break;
        if (session.motor.unsafe_latched() || (session.second && session.second->unsafe_latched())) break;
        std::this_thread::sleep_until(std::min(until, Clock::now()+std::chrono::milliseconds(30)));
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 10000);
    const Result ra = session.motor.stop(after_ms(budget));
    const Result rb = session.second ? session.second->stop(after_ms(budget)) : Result{};
    if (sent_a && sent_b && ra && rb) {
        session.stopped_at = Clock::now();
        std::cout << "    两轴已确认停止。\n";
        return true;
    }
    std::cout << "    [!] 未能确认两轴都已停止；请按现场急停/断电处置。\n";
    session.command_rc = 4;
    return false;
}

// 跳闸原因里带上 SDO 诊断上下文（phase/elapsed/budget），现场才能看出是哪个对象哪一段超的。
std::string sync_reason_text(const Result& result) {
    return result.context.empty() ? result.message : result.message + " [" + result.context + "]";
}

// 失败归因：两轴都读不到时优先怀疑整条总线（工具、终端电阻、驱动器供电），只读到一轴才归到那一轴。
std::string sync_axis_failure_text(const Result& ok_a, const Result& ok_b) {
    if (!ok_a && !ok_b)
        return "两轴都读不到（优先怀疑整条总线 / 终端电阻 / 驱动器供电）：" + sync_reason_text(ok_a);
    if (!ok_a) return "A 轴：" + sync_reason_text(ok_a);
    return "B 轴：" + sync_reason_text(ok_b);
}

// 判异常后的处置：报原因、打印两轴最后读数，再同时急停。
void sync_trip(Session& session, const std::string& reason, const Snapshot* a = nullptr,
               const Snapshot* b = nullptr) {
    session.end_reason = "fault: " + reason;
    session.command_rc = 4;
    (void)sync_halt_both(session);
    std::cout << "\n[!] 双轴同步异常：" << reason << "\n";
    if (a) std::cout << "    A 轴 " << one_line_snapshot(*a) << "\n";
    if (b) std::cout << "    B 轴 " << one_line_snapshot(*b) << "\n";
    if (session.motion_log) sync_report_move(session, "异常停止", true);
}

bool sync_has_second(Session& session, const char* name) {
    if (!session.second) {
        std::cout << "[失败] " << name << "：未启用双轴；启动时请加 --node2 <第二轴节点号>。\n";
        return false;
    }
    return true;
}

bool sync_needs_enabled(Session& session, const char* name) {
    if (!session.dual_enabled) {
        std::cout << "[失败] " << name << "：两轴尚未使能，请先执行 sync-enable。\n";
        return false;
    }
    return true;
}

// 双轴常转收尾汇总：打印本次 sync-run 的累计运行时长，调用后常转状态即视为结束。
void sync_run_report_elapsed(const Session& session) {
    if (!session.sync_run_active) return;
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - session.sync_run_started).count();
    std::cout << "双轴常转共运行 " << elapsed_ms << " ms（设定 " << session.sync_run_rpm
              << " 电机RPM）。\n";
}

// 两轴各自做一次运动前只读检查；输出所有权已经在 sync-enable 里拿过。
bool sync_arm_both(Session& session) {
    if (!session.motor.armed() &&
        !report(session.motor.arm(after_ms(session.timeout_ms)), "A 轴运动前安全检查（ARM，只读）"))
        return false;
    if (!session.second->armed() &&
        !report(session.second->arm(after_ms(session.timeout_ms)), "B 轴运动前安全检查（ARM，只读）"))
        return false;
    return true;
}

Disposition command_sync_enable(Session& session) {
    if (!sync_has_second(session, "sync-enable")) return Disposition::keep_going;
    if (session.dual_enabled) {
        std::cout << "两轴输出已使能。\n";
        return Disposition::keep_going;
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    // 顺序执行：两个 Motor 共用一条 transport，不能并发发 SDO。
    if (!report(session.motor.arm(after_ms(session.timeout_ms)), "A 轴安全检查（ARM，只读）"))
        return Disposition::keep_going;
    if (!report(session.motor.enable_output(after_ms(budget)), "A 轴使能输出"))
        return Disposition::keep_going;
    session.enabled = true; // retain cleanup ownership even if enabling B fails
    if (!report(session.second->arm(after_ms(session.timeout_ms)), "B 轴安全检查（ARM，只读）")) {
        std::cout << "    B 轴未就绪；A 轴仍在使能，可用 sync-disable 退出。\n";
        return Disposition::keep_going;
    }
    if (!report(session.second->enable_output(after_ms(budget)), "B 轴使能输出")) {
        std::cout << "    B 轴未能使能；A 轴仍在使能，可用 sync-disable 退出。\n";
        return Disposition::keep_going;
    }
    session.enabled = true;
    session.dual_enabled = true;
    std::cout << "两轴已使能。可用 sync-rel / sync-abs / sync-spin / sync-run 做同步运动，"
                 "sync-status 看两轴偏差。\n";
    return Disposition::keep_going;
}

Disposition command_sync_disable(Session& session) {
    if (!sync_has_second(session, "sync-disable")) return Disposition::keep_going;
    sync_run_report_elapsed(session);
    session.sync_run_active = false;
    session.end_reason = "operator sync-disable";
    if (session.motor.cleanup_required() || session.second->cleanup_required()) (void)sync_halt_both(session);
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    const Result ra = session.motor.disable_output(after_ms(budget));
    const Result rb = session.second->disable_output(after_ms(budget));
    session.dual_enabled = false;
    session.enabled = false;
    const bool reported_a = report(ra, "A 轴停止并失能");
    const bool reported_b = report(rb, "B 轴停止并失能");
    if (reported_a && reported_b) {
        sync_report_move(session, "sync-disable 收尾", true);
        return Disposition::keep_going;
    }
    std::cout << "[!] 未能确认两轴都已失能；心跳已停止，驱动约 1 s 后应报 0x20 并停机，\n"
                 "    但现场仍须按急停/断电处置。\n";
    session.end_reason += "; disable unconfirmed";
    session.command_rc = 4;
    sync_report_move(session, "sync-disable 未确认", true);
    return Disposition::keep_going;
}

Disposition command_sync_stop(Session& session) {
    if (!sync_has_second(session, "sync-stop")) return Disposition::keep_going;
    sync_run_report_elapsed(session);
    session.sync_run_active = false;
    if (!session.dual_enabled && !session.motor.cleanup_required() &&
        !session.second->cleanup_required()) {
        std::cout << "两轴均未使能，无需停止。\n";
        return Disposition::keep_going;
    }
    session.end_reason = "operator sync-stop";
    std::cout << "两轴同时停止并确认…\n";
    if (!sync_halt_both(session)) session.end_reason += "; stop unconfirmed";
    sync_report_move(session, "两轴停止收尾", true);
    return Disposition::keep_going;
}

Disposition command_sync_status(Session& session) {
    if (!sync_has_second(session, "sync-status")) return Disposition::keep_going;
    const Snapshot a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot b = session.second->snapshot(after_ms(session.timeout_ms));
    std::cout << "--- A 轴（节点 " << static_cast<unsigned>(session.node) << "）---\n";
    print_snapshot(a, session.node, session.interface_label);
    std::cout << "--- B 轴（节点 " << static_cast<unsigned>(session.node2) << "）---\n";
    print_snapshot(b, session.node2, session.interface_label);
    // 镜像方向与阈值不依赖电机应答，先打出来，接不上电机时也能核对极性。
    std::cout << "镜像方向 " << sync_dir_text(session) << "   一致性指标 = posA + posB\n"
              << "同步偏差阈值 " << fixed(session.sync_tolerance_deg, 4) << "°"
              << (session.sync_tolerance_deg <= 0 ? "（监控已关闭）" : "") << "\n"
              << "速度差告警参考 " << fixed(session.sync_drift_rpm, 2)
              << " 电机RPM（sync-drift；累计保护不随时间放宽）\n"
              << "最大减速修正 " << session.correction_rpm << " RPM（另受名义速度10%限制）\n"
              << "丝杆导程 "
              << (session.lead_mm > 0
                      ? fixed(session.lead_mm, 4) + " mm/输出圈（sync-mm 可用）"
                      : std::string("未设置 —— sync-mm 会被拒绝，先执行 set-lead <mm/圈>"))
              << "\n";
    if (a.result && b.result) {
        const int64_t raw = static_cast<int64_t>(a.position_pulses) - b.position_pulses;
        const int64_t metric = sync_pair_metric(a.position_pulses, b.position_pulses);
        std::cout << "两轴位置读数之差 posA - posB " << travel_text(session.lead_mm, raw)
                  << "   ← 它随行程线性变化（约 2×行程 加一个机械常数），不是同步偏差\n"
                  << "当前一致性指标 posA + posB " << travel_text(session.lead_mm, metric)
                  << "   ← 正比于刷架两端高度差，偏差看它相对基准漂了多少\n";
        if (session.sync_zero_valid) {
            const int64_t travel_a = static_cast<int64_t>(a.position_pulses) - session.sync_zero_a;
            const int64_t travel_b = static_cast<int64_t>(b.position_pulses) - session.sync_zero_b;
            const int64_t tilt =
                metric - (static_cast<int64_t>(session.sync_zero_a) + session.sync_zero_b);
            std::cout << "相对行程基准 行程 A " << travel_text(session.lead_mm, travel_a) << " / B "
                      << travel_text(session.lead_mm, travel_b)
                      << "   ← 镜像正常时两者符号相反、大小接近\n"
                      << "相对行程基准 两端错位 " << travel_text(session.lead_mm, tilt)
                      << "   ← 真正的偏差：两端高度差相对记基准那一刻变了多少\n";
        } else {
            std::cout << "行程基准 未设置 —— 执行 sync-zero 把当前两轴位置记成 0 行程\n";
        }
    }
    return Disposition::keep_going;
}

// Fixed session origin for display, correction and cumulative skew protection. No device write.
Disposition command_sync_zero(Session& session) {
    if (!sync_has_second(session, "sync-zero")) return Disposition::keep_going;
    if (session.sync_run_active || session.motor.sustaining() || session.motion_log) {
        std::cout << "[失败] 运动中不能更换固定零点，请先sync-stop。\n";
        return Disposition::keep_going;
    }
    const Snapshot a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot b = session.second->snapshot(after_ms(session.timeout_ms));
    if (!a.result || !b.result) { report(!a.result ? a.result : b.result, "读取两轴零点"); return Disposition::keep_going; }
    if (!a.position_sample.valid || !b.position_sample.valid ||
        !a.speed_sample.valid || !b.speed_sample.valid || std::abs(a.speed_rpm)>0.5 || std::abs(b.speed_rpm)>0.5) {
        std::cout << "[失败] 两轴未确认静止，零点未更新。\n";
        return Disposition::keep_going;
    }
    const auto now = Clock::now();
    for (const auto& reading : {a.position_sample,b.position_sample,a.speed_sample,b.speed_sample}) {
        if (reading.sampled_at>now || now-reading.sampled_at>std::chrono::milliseconds(400)) {
            std::cout << "[失败] 零点采样过期，请保持两轴静止后重试。\n";
            return Disposition::keep_going;
        }
    }
    session.sync_zero_a = a.position_pulses;
    session.sync_zero_b = b.position_pulses;
    session.sync_zero_valid = true;
    std::cout << "固定软件零点：A=" << a.position_pulses << "，B=" << b.position_pulses
              << "；本会话所有速度动作沿用，动作不会自动重置；未写硬件零点。\n";
    return Disposition::keep_going;
}

Disposition command_sync_tol(Session& session, const std::string& text) {
    if (!sync_has_second(session, "sync-tol")) return Disposition::keep_going;
    double degrees = 0;
    if (!parse_double(text, degrees) || degrees < 0 || degrees > 3600) {
        std::cout << "用法：sync-tol <输出角度，度>（0 表示关闭偏差监控）\n";
        return Disposition::keep_going;
    }
    if (session.sync_run_active) { std::cout << "请先sync-stop再调整保护阈值。\n"; return Disposition::keep_going; }
    session.sync_tolerance_deg = degrees;
    std::cout << "同步偏差阈值 = " << fixed(degrees, 4) << "°"
              << (degrees <= 0 ? "（已关闭偏差监控）" : "") << "；约 "
              << static_cast<int64_t>(sync_tolerance_pulses(session)) << " 电机脉冲。\n";
    return Disposition::keep_going;
}

Disposition command_sync_drift(Session& session, const std::string& text) {
    double rpm = 0;
    if (!parse_double(text, rpm) || rpm < 0 || rpm > yz_aim::kSyncMaxRpm || session.sync_run_active) {
        std::cout << "用法：sync-drift <0~300RPM>，请在静止时设置（仅速度差告警参考）。\n";
        return Disposition::keep_going;
    }
    session.sync_drift_rpm = rpm;
    std::cout << "速度差告警参考=" << rpm << "RPM；累计位置保护不会随时间放宽。\n";
    return Disposition::keep_going;
}
Disposition command_sync_correct(Session& session, const std::string& text) {
    double rpm = 0;
    if (!parse_double(text, rpm) || rpm < 0 || rpm > 10 || session.sync_run_active) {
        std::cout << "用法：sync-correct <0~10RPM>，请在静止时设置。\n";
        return Disposition::keep_going;
    }
    session.correction_rpm = rpm;
    std::cout << "最大减速修正=" << rpm << "RPM，另受名义速度10%限制；不超速、不反向。\n";
    return Disposition::keep_going;
}

// set-lead <mm/圈>：设置丝杆导程 = 输出轴转一整圈时刷架走的毫米数，供 sync-mm 换算行程。
// 只改本会话，不写设备；0 = 清除。导程错了会按错误距离走，所以不设默认值、未设时拒绝 sync-mm。
Disposition command_set_lead(Session& session, const std::string& text) {
    double lead = 0;
    if (!parse_double(text, lead) || lead < 0 || lead > 1000) {
        std::cout << "用法：set-lead <导程 mm/圈，0~1000>（0 = 清除；未设置时 sync-mm 拒绝执行）\n"
                     "    导程 = 输出轴转一整圈时刷架走的毫米数。测法：sync-rel 3600 100（整 10 圈），\n"
                     "    量出刷架实际位移 L 毫米，导程 = L ÷ 10。\n";
        return Disposition::keep_going;
    }
    const double previous = session.lead_mm;
    session.lead_mm = lead;
    if (lead <= 0) {
        std::cout << "已清除导程设置（原 " << fixed(previous, 4) << " mm/圈）；sync-mm 现在会被拒绝。\n";
        return Disposition::keep_going;
    }
    std::cout << "导程 = " << fixed(lead, 4) << " mm/输出圈（1 mm = " << fixed(360.0 / lead, 4)
              << "° 输出 = " << static_cast<int64_t>(
                     yz_aim::kOutputPulsesPerRevolution / lead)
              << " 电机脉冲）。只在本会话有效，quit 后失效。\n";
    return Disposition::keep_going;
}

// 双轴同步位置运动：先各自预置，再连发两条触发帧（这就是同步点），随后监视到两轴都到位。
Disposition command_sync_move(Session& session, const std::vector<std::string>& arguments, bool absolute) {
    const char* name = absolute ? "sync-abs" : "sync-rel";
    if (!sync_has_second(session, name)) return Disposition::keep_going;
    if (!sync_needs_enabled(session, name)) return Disposition::keep_going;
    if (session.sync_run_active) {
        std::cout << "[失败] " << name << "：双轴常转进行中，请先 sync-stop。\n";
        return Disposition::keep_going;
    }
    double degrees = 0;
    if (arguments.empty() || !parse_double(arguments[0], degrees)) {
        std::cout << "用法：" << name << " <输出角度> [电机RPM 1~" << yz_aim::kSyncMaxRpm << "]\n";
        return Disposition::keep_going;
    }
    uint32_t rpm = session.motion_rpm;
    if (arguments.size() >= 2 &&
        !parse_u32(arguments[1], 1, static_cast<uint32_t>(yz_aim::kSyncMaxRpm), rpm)) {
        std::cout << "速度需为 1~" << yz_aim::kSyncMaxRpm << " 电机RPM（双轴同步的硬上限）\n";
        return Disposition::keep_going;
    }
    const int32_t requested = sync_degrees_to_pulses(degrees);
    if (requested == 0) {
        std::cout << "换算结果是 0 脉冲，未发送命令。\n";
        return Disposition::keep_going;
    }
    // 绝对模式以 A 轴坐标为基准算增量，然后两轴走同样的脉冲数 ——
    // 刚性刷架要求两轴行程完全一致，各走各的绝对坐标会把刷架别住。
    int64_t delta = requested;
    int32_t target_a = 0;
    if (absolute) {
        const Snapshot before = session.motor.snapshot(after_ms(session.timeout_ms));
        if (!before.result) { report(before.result, "动作前读取 A 轴位置"); return Disposition::keep_going; }
        target_a = requested;
        delta = static_cast<int64_t>(target_a) - before.position_pulses;
        if (delta == 0) {
            std::cout << "目标与 A 轴当前位置相同，未发送命令。\n";
            return Disposition::keep_going;
        }
        std::cout << "以 A 轴坐标为基准：当前 " << before.position_pulses << " 脉冲（"
                  << fixed(before.output_position_degrees, 4) << "°）→ 目标 " << target_a
                  << " 脉冲（" << fixed(yz_aim::pulses_to_output_degrees(target_a), 4) << "°）\n";
    }
    if (delta < std::numeric_limits<int32_t>::min() || delta > std::numeric_limits<int32_t>::max()) {
        std::cout << "[失败] 增量超出 signed32 范围。\n";
        return Disposition::keep_going;
    }
    const int32_t delta_pulses = static_cast<int32_t>(delta);
    // 绝对模式下 A 轴坐标是显式给出的，极性不改变它；相对模式按"伸出方向"定义符号，
    // 谁取正由 sync-dir 决定，另一轴永远取负。
    const int32_t delta_a = absolute ? delta_pulses : sync_sign_a(session) * delta_pulses;
    const int32_t delta_b = -delta_a;
    if (!sync_arm_both(session)) return Disposition::keep_going;
    if (!report(session.motor.stage_relative(delta_a, after_ms(session.timeout_ms), rpm),
                "A 轴预置相对运动"))
        return Disposition::keep_going;
    if (!report(session.second->stage_relative(delta_b, after_ms(session.timeout_ms), rpm),
                "B 轴预置相对运动")) {
        report(session.motor.abort_staged(after_ms(session.timeout_ms)), "放弃 A 轴预置");
        return Disposition::keep_going;
    }
    const int32_t staged_a = static_cast<int32_t>(session.motor.staged_target_pulses());
    const int32_t staged_b = static_cast<int32_t>(session.second->staged_target_pulses());
    const int64_t staged_metric = sync_pair_metric(staged_a, staged_b);
    // 位置模式的基准是"两个预置目标的指标"，运行时不再重取 —— 位置模式本来就是闭环到位的。
    session.sync_base_pulses = staged_metric;
    session.sync_origin_pulses = staged_metric;
    session.sync_base_at = Clock::now();
    // 记下本次动作的起点与目标坐标，供收尾汇总算各轴行程与到位残差（只用于显示）。
    session.sync_start_valid = true;
    session.sync_start_a = static_cast<int32_t>(static_cast<int64_t>(staged_a) - delta_a);
    session.sync_start_b = static_cast<int32_t>(static_cast<int64_t>(staged_b) - delta_b);
    session.sync_target_a = staged_a;
    session.sync_target_b = staged_b;
    session.sync_move_started = Clock::now();
    std::cout << "A 轴 " << delta_a << " / B 轴 " << delta_b << " 电机脉冲（"
              << fixed(yz_aim::pulses_to_output_degrees(delta_a), 4) << " / "
              << fixed(yz_aim::pulses_to_output_degrees(delta_b), 4) << " 输出度），速度 " << rpm
              << " 电机RPM；正在同步触发…\n";
    const Deadline trigger_deadline = after_ms(session.timeout_ms);
    const Result trigger_a = session.motor.trigger(trigger_deadline);
    const Result trigger_b = session.second->trigger(trigger_deadline);
    if (!trigger_a || !trigger_b) {
        sync_trip(session, trigger_a ? "B 轴触发失败：" + sync_reason_text(trigger_b)
                                     : "A 轴触发失败：" + sync_reason_text(trigger_a));
        return Disposition::keep_going;
    }
    const Deadline until = after_ms(position_budget_ms(delta_pulses, rpm) + 5000);
    const double tolerance = sync_tolerance_pulses(session);
    const int64_t deviation_limit = static_cast<int64_t>(tolerance);
    int over = 0;
    bool have_first_done = false;
    Clock::time_point first_done_at{};
    Snapshot a{}, b{};
    while (Clock::now() < until) {
        if (g_interrupted) break;
        // 两轴都读完再判定：只读到一轴就归因，会把整条总线的问题误报成单轴问题，
        // 而且另一轴的快照会停在上一拍，跳闸时打出来的读数不是当拍的。
        const Result ok_a = session.motor.poll_safety(a, after_ms(2000));
        const Result ok_b = session.second->poll_safety(b, after_ms(2000));
        if (!ok_a || !ok_b) {
            sync_trip(session, sync_axis_failure_text(ok_a, ok_b), &a, &b);
            return Disposition::keep_going;
        }
        int64_t pa = 0, pb = 0;
        sync_align_positions(a, b, pa, pb);
        const int64_t rem_a = std::abs(static_cast<int64_t>(staged_a) - pa);
        const int64_t rem_b = std::abs(static_cast<int64_t>(staged_b) - pb);
        const bool done_a = (a.statusword & (1U << 10)) && std::abs(a.speed_rpm) <= 0.5 &&
                            rem_a <= kSyncArrivePulses;
        const bool done_b = (b.statusword & (1U << 10)) && std::abs(b.speed_rpm) <= 0.5 &&
                            rem_b <= kSyncArrivePulses;
        if (done_a || done_b) {
            if (!have_first_done) { have_first_done = true; first_done_at = Clock::now(); }
        } else {
            have_first_done = false;
        }
        if (deviation_limit > 0) {
            // 一轴先到位后另一轴要在末段自己减速，此时放宽阈值，但真正的跑飞仍会被抓住。
            const int64_t limit = have_first_done ? deviation_limit * 4 : deviation_limit;
            const int64_t deviation = rem_a - rem_b;
            const int64_t magnitude = deviation < 0 ? -deviation : deviation;
            if (magnitude > limit) {
                if (++over >= kSyncTripSamples) {
                    std::cout << "\n[!] 两轴偏差 " << magnitude << " 电机脉冲（"
                              << fixed(yz_aim::pulses_to_output_degrees(static_cast<int32_t>(magnitude)), 4)
                              << "°）连续 " << over << " 拍超过阈值 "
                              << fixed(session.sync_tolerance_deg, 4) << "°\n";
                    sync_trip(session, "位置偏差超限", &a, &b);
                    return Disposition::keep_going;
                }
            } else {
                over = 0;
            }
        }
        if (have_first_done && Clock::now() - first_done_at > std::chrono::milliseconds(kSyncFinalGraceMs)) {
            sync_trip(session, "一轴已到位而另一轴在宽限期内未到位", &a, &b);
            return Disposition::keep_going;
        }
        std::cout << "\r[同步] A " << a.position_pulses << "(" << fixed(a.speed_rpm, 0) << "RPM)  B "
                  << b.position_pulses << "(" << fixed(b.speed_rpm, 0) << "RPM)  偏差 "
                  << (rem_a - rem_b) << " 脉冲   " << std::flush;
        if (done_a && done_b) break;
        std::this_thread::sleep_until(std::min(until, Clock::now() + std::chrono::milliseconds(70)));
    }
    std::cout << "\n";
    if (g_interrupted) {
        g_interrupted = 0;
        std::cout << "[已取消] 收到中断，两轴同时急停。\n";
        sync_halt_both(session);
        sync_report_move(session, name, false);
        return Disposition::keep_going;
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    const Result held_a = session.motor.stop(after_ms(budget));
    const Result held_b = session.second->stop(after_ms(budget));
    if (!held_a || !held_b) {
        std::cout << "[!] 运动结束但未能确认两轴都已停止并保持；请现场确认。\n";
        return Disposition::keep_going;
    }
    std::cout << "两轴同步到位：A " << a.position_pulses << " 脉冲，B " << b.position_pulses
              << " 脉冲。\n";
    sync_report_move(session, name, false);
    return Disposition::keep_going;
}

// sync-mm <毫米> [电机RPM]：按毫米走行程。用 set-lead 设的导程把毫米换算成输出角度，
// 再走与 sync-rel 完全相同的位置模式路径（闭环到位、两轴镜像反向）。
Disposition command_sync_mm(Session& session, const std::vector<std::string>& arguments) {
    if (arguments.empty() || arguments.size() > 2) {
        std::cout << "用法：sync-mm <毫米，可小数可负> [电机RPM 1~" << yz_aim::kSyncMaxRpm << "]\n"
                     "    行程 = 毫米 ÷ 导程 × 360°；导程用 set-lead <mm/圈> 设置。\n";
        return Disposition::keep_going;
    }
    double millimeters = 0;
    if (!parse_double(arguments[0], millimeters)) {
        std::cout << "毫米需为数字（可小数、可负）\n";
        return Disposition::keep_going;
    }
    // 先按 sync-mm 自己的名字把前置条件查一遍，免得后面的报错写成 sync-rel。
    // 导程检查放在使能之前：没导程根本算不出行程，与电机接没接、使没使能都无关。
    if (!sync_has_second(session, "sync-mm")) return Disposition::keep_going;
    if (session.lead_mm <= 0) {
        std::cout << "[失败] sync-mm：尚未设置丝杆导程，无法把毫米换算成行程。\n"
                     "    先执行 set-lead <导程 mm/圈>（导程 = 输出轴转一整圈时刷架走的毫米数）。\n";
        return Disposition::keep_going;
    }
    if (millimeters == 0) {
        std::cout << "行程为 0 毫米，未发送命令。\n";
        return Disposition::keep_going;
    }
    if (!sync_needs_enabled(session, "sync-mm")) return Disposition::keep_going;
    if (session.sync_run_active) {
        std::cout << "[失败] sync-mm：双轴常转进行中，请先 sync-stop。\n";
        return Disposition::keep_going;
    }
    const double degrees = millimeters / session.lead_mm * 360.0;
    std::cout << fixed(millimeters, 4) << " mm ÷ 导程 " << fixed(session.lead_mm, 4)
              << " mm/圈 = " << fixed(degrees, 4) << "° 输出\n";
    std::vector<std::string> forwarded;
    forwarded.push_back(fixed(degrees, 6));
    if (arguments.size() == 2) forwarded.push_back(arguments[1]);
    return command_sync_move(session, forwarded, false);
}

// One sampling/correction path for extend, retract, sync-spin and sync-run.
Result sync_velocity_sample(Session& session, Deadline until, int& over) {
    if (Clock::now() >= until) return {Error::cancelled, 0, "timed motion elapsed"};
    Snapshot a{}, b{};
    Result ra = session.motor.poll_safety(a, std::min(until, after_ms(2000)));
    if (!ra) { ra.message = "A轴 " + ra.message; return ra; }
    Result rb = session.second->poll_safety(b, std::min(until, after_ms(2000)));
    if (!rb) { rb.message = "B轴 " + rb.message; return rb; }
    const auto now = Clock::now();
    auto estimate_acceleration = [](const Snapshot& current, const Snapshot& previous) {
        if (!current.speed_sample.valid || !previous.speed_sample.valid ||
            current.speed_sample.retried || previous.speed_sample.retried) return 0.0;
        const double dt=std::chrono::duration<double>(current.speed_sample.sampled_at-
                                                     previous.speed_sample.sampled_at).count();
        return dt>=0.02 && dt<=1.0 ? std::clamp((current.speed_rpm-previous.speed_rpm)/dt,-500.0,500.0) : 0.0;
    };
    session.acceleration_a = estimate_acceleration(a,session.previous_feedback_a);
    session.acceleration_b = estimate_acceleration(b,session.previous_feedback_b);
    session.previous_feedback_a = a; session.previous_feedback_b = b;
    const auto f = yz_aim::aligned_feedback(a,b,now,session.acceleration_a,session.acceleration_b,std::abs(session.nominal_rpm));
    const auto ramp_ms = std::abs(session.nominal_rpm)*1000/500 + 200;
    const bool settled = now-session.sync_move_started >= std::chrono::milliseconds(ramp_ms);
    if (!sync_log_sample(session, a, b, f.usable ? (settled ? "sample" : "startup_sample") : "quality_warning"))
        return {Error::transport, 0, "CSV日志写入失败或缓冲已满"};
    if (!f.usable) {
        if (now - session.last_usable_feedback > std::chrono::milliseconds(1000))
            return {Error::stale, 0, "持续1000ms未取得可对齐的新鲜两轴反馈"};
        return {};
    }
    session.last_usable_feedback = now;
    const double dt = std::chrono::duration<double>(now-session.feedback_at).count();
    session.feedback_at = now;
    const auto trim = session.synchronizer.update(f, dt, settled);
    if (session.sync_tolerance_deg > 0 && std::abs(trim.error_deg) > session.sync_tolerance_deg) {
        if (++over >= kSyncTripSamples) {
            std::ostringstream reason;
            reason << "固定零点累计偏差 " << trim.error_deg << "度连续" << over
                   << "拍超过" << session.sync_tolerance_deg << "度（未重置基准）";
            return {Error::limit, 0, reason.str()};
        }
    } else over = 0;
    if ((std::abs(a.speed_rpm)>2 && a.speed_rpm*session.target_rpm_a<0) ||
        (std::abs(b.speed_rpm)>2 && b.speed_rpm*session.target_rpm_b<0))
        return {Error::unsafe, 0, "实际转向与命令方向不一致"};
    const double overspeed = std::abs(session.nominal_rpm)+std::max(10.0,std::abs(session.nominal_rpm)*0.10);
    if (std::abs(a.speed_rpm)>overspeed || std::abs(b.speed_rpm)>overspeed)
        return {Error::unsafe, 0, "实测速度显著超过命令名义上限"};
    if (std::abs(a.speed_rpm+b.speed_rpm) > session.sync_drift_rpm)
        if (!sync_log_sample(session, a, b, "speed_difference_warning"))
            return {Error::transport, 0, "CSV速度差告警记录失败"};
    // Reserve both write-ACK windows. The final part of a timed move only reads feedback.
    if (until != Deadline::max() && until-now < std::chrono::milliseconds(300)) return {};
    if (trim.rpm_a != session.target_rpm_a) {
        Result r = session.motor.update_velocity(trim.rpm_a, std::min(until, after_ms(500)));
        if (!r) return r;
        session.target_rpm_a = trim.rpm_a;
    }
    if (trim.rpm_b != session.target_rpm_b) {
        Result r = session.second->update_velocity(trim.rpm_b, std::min(until, after_ms(500)));
        if (!r) return r;
        session.target_rpm_b = trim.rpm_b;
    }
    return {};
}

Disposition command_sync_velocity(Session& session, const std::vector<std::string>& arguments,
                                  bool continuous, const char* label = nullptr) {
    const std::string name = label ? label : (continuous ? "sync-run" : "sync-spin");
    if (!sync_has_second(session, name.c_str()) || !sync_needs_enabled(session, name.c_str()))
        return Disposition::keep_going;
    if (session.sync_run_active || session.motion_log) {
        std::cout << "请先sync-stop结束当前动作。\n";
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    uint32_t duration_ms = 60000;
    if (arguments.empty() || arguments.size() > (continuous ? 1U : 2U) ||
        !parse_i32(arguments[0], -yz_aim::kSyncMaxRpm, yz_aim::kSyncMaxRpm, rpm) || !rpm ||
        (!continuous && arguments.size()>1 && !parse_u32(arguments[1],1,600000,duration_ms))) {
        std::cout << "用法：" << name << " <非零rpm ±1~±300>" << (continuous ? "\n" : " [ms 1~600000]\n");
        return Disposition::keep_going;
    }
    if (!session.sync_zero_valid) {
        std::cout << "[失败] 请先把机构放在正确初始位置，静止时执行sync-zero；程序不会自动重置零点。\n";
        return Disposition::keep_going;
    }
    if (!sync_arm_both(session)) return Disposition::keep_going;
    const Snapshot base_a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot base_b = session.second->snapshot(after_ms(session.timeout_ms));
    if (!base_a.result || !base_b.result) {
        report(!base_a.result ? base_a.result : base_b.result, "读取双轴动作起点");
        return Disposition::keep_going;
    }
    if (session.sync_tolerance_deg>0 &&
        std::abs(double(static_cast<int64_t>(base_a.position_pulses)-session.sync_zero_a+
                        static_cast<int64_t>(base_b.position_pulses)-session.sync_zero_b)
                 *360.0/yz_aim::kOutputPulsesPerRevolution) > session.sync_tolerance_deg) {
        std::cout << "[失败] 起点相对固定零点的错位已超保护值；先检查机构，不自动重新置零。\n";
        return Disposition::keep_going;
    }
    session.nominal_rpm = rpm;
    session.max_error_deg = 0;
    session.acceleration_a = session.acceleration_b = 0;
    session.previous_feedback_a = base_a; session.previous_feedback_b = base_b;
    session.requested_ms = continuous ? 0 : duration_ms;
    session.target_rpm_a = sync_sign_a(session)*rpm;
    session.target_rpm_b = -session.target_rpm_a;
    session.end_reason.clear();
    session.stop_a_at = session.stop_b_at = session.stopped_at = {};
    session.synchronizer.reset(session.sync_zero_a, session.sync_zero_b, sync_sign_a(session), rpm, session.correction_rpm);
    if (session.log_directory.empty()) {
        std::cout << "[失败] 无法定位本程序目录，未启动运动，不能将日志写到其他目录。\n";
        return Disposition::keep_going;
    }
    auto log = std::make_unique<yz_aim::MotionLog>();
    std::ostringstream metadata;
    metadata << "# version=" << kVersion << " build=" __DATE__ " " __TIME__
             << " command=" << name << " nominal_rpm=" << rpm << " requested_ms=" << session.requested_ms
             << " zero_a=" << session.sync_zero_a << " zero_b=" << session.sync_zero_b
             << " node_a=" << unsigned(session.node) << " node_b=" << unsigned(session.node2)
             << " sign_a=" << sync_sign_a(session) << " hard_limit_output_deg=" << session.sync_tolerance_deg
             << " max_trim_rpm=" << session.correction_rpm;
    std::string log_error;
    if (!log->open(session.log_directory, metadata.str(), log_error)) {
        std::cout << "[失败] 日志未就绪，未启动运动：" << log_error << "\n";
        return Disposition::keep_going;
    }
    session.motion_log = std::move(log);
    const auto travel = continuous ? std::chrono::milliseconds(60000) : std::chrono::milliseconds(duration_ms);
    if (!report(session.motor.stage_velocity(session.target_rpm_a, travel, after_ms(session.timeout_ms)), "A轴预置速度模式")) {
        sync_finish_log(session, "A stage failed"); return Disposition::keep_going;
    }
    if (!report(session.second->stage_velocity(session.target_rpm_b, travel, after_ms(session.timeout_ms)), "B轴预置速度模式")) {
        (void)session.motor.abort_staged(after_ms(session.timeout_ms));
        sync_finish_log(session, "B stage failed"); return Disposition::keep_going;
    }
    session.sync_start_a = base_a.position_pulses;
    session.sync_start_b = base_b.position_pulses;
    session.sync_start_valid = true;
    session.sync_origin_pulses = sync_pair_metric(base_a.position_pulses, base_b.position_pulses);
    std::cout << "A轴 " << session.target_rpm_a << " / B轴 " << session.target_rpm_b
              << " 名义电机RPM（" << sync_dir_text(session) << "），"
              << (continuous ? "常转直到sync-stop" : std::to_string(duration_ms)+"ms")
              << "；固定零点同步修正，CSV=" << session.motion_log->path() << "\n" << std::flush;
    // Start before the first target write; do not restart the clock after serial SDO acknowledgements.
    session.sync_move_started = Clock::now();
    session.feedback_at = session.last_usable_feedback = session.sync_move_started;
    const Deadline until = continuous ? Deadline::max() : session.sync_move_started+std::chrono::milliseconds(duration_ms);
    session.motor.set_motion_deadline(until);
    session.second->set_motion_deadline(until);
    const Result trigger = session.motor.trigger_velocity_pair(*session.second, std::min(until, after_ms(session.timeout_ms)));
    if (!trigger) {
        if (g_interrupted && trigger.error==Error::cancelled) {
            g_interrupted = 0;
            session.end_reason = "operator interrupt during trigger";
            if (!sync_halt_both(session)) session.end_reason += "; stop unconfirmed";
            sync_report_move(session, name, true);
            return Disposition::keep_going;
        }
        if (Clock::now()>=until && trigger.error==Error::cancelled) {
            session.end_reason = "requested duration elapsed during trigger";
            const bool stopped = sync_halt_both(session);
            if (!stopped) session.end_reason = "stop unconfirmed after duration";
            sync_report_move(session, name, true);
        } else sync_trip(session, "速度触发未完整确认："+sync_reason_text(trigger));
        return Disposition::keep_going;
    }
    if (continuous) {
        session.sync_run_active = true;
        session.sync_run_rpm = rpm;
        session.sync_run_over = 0;
        session.sync_run_started = session.sync_move_started;
        session.sync_run_next_tick = Clock::now();
        std::cout << "已启动双轴常转，sync-stop停止并结束日志。\n";
        return Disposition::keep_going;
    }
    int over = 0;
    auto next_sample = Clock::now();
    while (Clock::now()<until && !g_interrupted) {
        Result r = sync_velocity_sample(session, until, over);
        if (!r) {
            if (Clock::now()>=until && r.error==Error::cancelled) break;
            if (g_interrupted && r.error==Error::cancelled) break;
            sync_trip(session, sync_reason_text(r));
            return Disposition::keep_going;
        }
        next_sample = std::max(next_sample+std::chrono::milliseconds(100),Clock::now());
        while (Clock::now()<std::min(next_sample,until) && !g_interrupted) {
            Result heart = session.motor.service_watchdog(until);
            if (!heart) { sync_trip(session,sync_reason_text(heart)); return Disposition::keep_going; }
            std::this_thread::sleep_until(std::min({next_sample,until,Clock::now()+std::chrono::milliseconds(10)}));
        }
    }
    const bool interrupted = g_interrupted;
    g_interrupted = 0;
    session.end_reason = interrupted ? "operator interrupt" : "requested duration elapsed";
    // No console or disk I/O ahead of these two stop frames.
    const bool stopped = sync_halt_both(session);
    if (!stopped) session.end_reason += "; stop unconfirmed";
    std::cout << (interrupted ? "已按中断请求停止。\n" : "到时正常停止。\n");
    sync_report_move(session,name,true);
    return Disposition::keep_going;
}

// sync-dir <正转节点> <反转节点>：指定镜像对里哪个节点取正、哪个取负。
// 两个参数必须正好是本会话的 A、B 两个节点各一次；否则整条命令不生效，当前设置保持不变 ——
// 双轴永远是一正一负，不存在"两轴同向"的配置。
// 启动时的默认是 B 轴（--node2）取 +、A 轴（--node）取 −，要反过来就执行一次 sync-dir。
Disposition command_sync_dir(Session& session, const std::vector<std::string>& arguments) {
    if (!sync_has_second(session, "sync-dir")) return Disposition::keep_going;
    const uint32_t node_a = session.node;
    const uint32_t node_b = session.node2;
    auto usage = [&]() {
        std::cout << "用法：sync-dir <正转节点> <反转节点>\n"
                     "    两个参数必须是节点 " << node_a << " 与节点 " << node_b
                  << " 各一个且不重复（当前 " << sync_dir_text(session) << "）。\n"
                     "    例：sync-dir " << node_a << " " << node_b << "  → 节点 " << node_a
                  << " 取 +、节点 " << node_b << " 取 −；调换两个参数即反向。\n";
    };
    if (arguments.size() != 2) { usage(); return Disposition::keep_going; }
    uint32_t plus = 0, minus = 0;
    if (!parse_u32(arguments[0], 1, 127, plus) || !parse_u32(arguments[1], 1, 127, minus)) {
        usage();
        return Disposition::keep_going;
    }
    // 一正一负是两个节点各一次；给成同一个号、或给了不相干的号，命令一律不生效。
    if (plus == minus || !((plus == node_a && minus == node_b) || (plus == node_b && minus == node_a))) {
        std::cout << "[失败] sync-dir：必须是节点 " << node_a << " 与节点 " << node_b
                  << " 各一次（一正一负）；本次输入未生效，当前仍为 " << sync_dir_text(session) << "。\n";
        usage();
        return Disposition::keep_going;
    }
    if (session.sync_run_active) {
        std::cout << "[失败] sync-dir：双轴常转进行中，请先 sync-stop。\n";
        return Disposition::keep_going;
    }
    session.plus_is_a = (plus == node_a);
    std::cout << "镜像方向 = " << sync_dir_text(session)
              << "（节点 " << plus << " 收到 +rpm/+行程，节点 " << minus
              << " 收到 −rpm/−行程）；下一次动作生效。\n";
    return Disposition::keep_going;
}

// extend / retract：镜像同步速度运动。<rpm> 指定大小，方向由命令本身决定；
// extend = 取正的节点正转、取负的节点反转，retract 相反。两轴始终一正一负地把刷架往同一方向推。
// 默认取正的节点是 B 轴（--node2），所以 extend 是 B 轴 +rpm、A 轴 −rpm。
Disposition command_sync_extend(Session& session, const std::vector<std::string>& arguments, bool extend) {
    const char* name = extend ? "extend" : "retract";
    auto usage = [&]() {
        std::cout << "用法：" << name << " <rpm 1~" << yz_aim::kSyncMaxRpm << "> <毫秒 1~600000>\n";
    };
    if (arguments.size() != 2) { usage(); return Disposition::keep_going; }
    uint32_t magnitude = 0;
    uint32_t duration_ms = 0;
    if (!parse_u32(arguments[0], 1, static_cast<uint32_t>(yz_aim::kSyncMaxRpm), magnitude) ||
        !parse_u32(arguments[1], 1, 600000, duration_ms)) {
        usage();
        return Disposition::keep_going;
    }
    if (!sync_has_second(session, name)) return Disposition::keep_going;
    if (!sync_needs_enabled(session, name)) return Disposition::keep_going;
    std::vector<std::string> forwarded;
    forwarded.push_back(std::to_string(extend ? static_cast<int32_t>(magnitude)
                                              : -static_cast<int32_t>(magnitude)));
    forwarded.push_back(std::to_string(duration_ms));
    return command_sync_velocity(session, forwarded, false, extend ? "extend" : "retract");
}

// ===== 单轴单动 =====
// 按节点号取出本会话里的 Motor；节点不在本会话里返回 nullptr。
yz_aim::Motor* motor_of_node(Session& session, uint32_t node) {
    if (node == session.node) return &session.motor;
    if (session.second && node == session.node2) return session.second.get();
    return nullptr;
}

// 与该节点相对的另一轴；单轴模式下返回 nullptr。
yz_aim::Motor* other_motor_of(Session& session, yz_aim::Motor* motor) {
    if (!session.second) return nullptr;
    return motor == &session.motor ? session.second.get() : &session.motor;
}

// 该轴的输出是否已由本会话使能。
bool axis_enabled(const Session& session, uint32_t node) {
    if (node == session.node) return session.enabled || session.dual_enabled;
    if (session.second && node == session.node2) return session.dual_enabled;
    return false;
}

// one <节点> <rpm±1~±300> <毫秒 1~600000>：只让指定的一台电机单独运动，另一轴保持不动。
// 刚性刷架下单动会把架子别歪，只用于对位/测试，所以速度上限按双轴口径限制在 ±300。
// 运行期间两路心跳都必须维持，因此监视循环对两轴各做一次 poll_safety，而不是只盯目标轴。
Disposition command_one_move(Session& session, const std::vector<std::string>& arguments) {
    auto usage = [&]() {
        std::cout << "用法：one <节点号> <rpm±1~±" << yz_aim::kSyncMaxRpm << "> <毫秒 1~600000>\n";
    };
    if (arguments.size() != 3) { usage(); return Disposition::keep_going; }
    uint32_t node = 0;
    int32_t rpm = 0;
    uint32_t duration_ms = 0;
    if (!parse_u32(arguments[0], 1, 127, node) ||
        !parse_i32(arguments[1], -yz_aim::kSyncMaxRpm, yz_aim::kSyncMaxRpm, rpm) || rpm == 0 ||
        !parse_u32(arguments[2], 1, 600000, duration_ms)) {
        usage();
        return Disposition::keep_going;
    }
    yz_aim::Motor* motor = motor_of_node(session, node);
    if (motor == nullptr) {
        std::cout << "[失败] one：节点 " << node << " 不在本会话里（A 轴节点 "
                  << static_cast<unsigned>(session.node) << "，B 轴节点 "
                  << static_cast<unsigned>(session.node2) << "）。\n";
        return Disposition::keep_going;
    }
    if (session.sync_run_active) {
        std::cout << "[失败] one：双轴常转进行中，请先 sync-stop。\n";
        return Disposition::keep_going;
    }
    if (blocked_by_sustain(session, "one")) return Disposition::keep_going;
    if (!axis_enabled(session, node)) {
        std::cout << "[失败] one：节点 " << node << " 的输出尚未使能，请先执行 sync-enable。\n";
        return Disposition::keep_going;
    }
    if (!motor->armed() &&
        !report(motor->arm(after_ms(session.timeout_ms)), "单轴运动前安全检查（ARM，只读）"))
        return Disposition::keep_going;
    std::cout << "[!] 单轴单动：只动节点 " << node << "，另一轴保持不动 —— 刚性刷架会被别歪，"
                 "仅用于对位/测试。\n"
                 "    节点 " << node << " 以 " << rpm << " 电机RPM（"
              << fixed(yz_aim::motor_to_output_rpm(rpm), 3) << " 输出RPM）运行 " << duration_ms
              << " ms；按 Ctrl-C 可提前停。\n";
    // 记下本轴起点，收尾时给出"本次单动让刷架两端错开了多少"（另一轴没动，等于本轴行程）。
    const Snapshot before = motor->snapshot(after_ms(session.timeout_ms));
    const auto travel = std::chrono::milliseconds(duration_ms);
    if (!report(motor->stage_velocity(rpm, travel, after_ms(session.timeout_ms)),
                "预置单轴速度模式")) {
        (void)motor->abort_staged(after_ms(session.timeout_ms));
        return Disposition::keep_going;
    }
    const Result triggered = motor->trigger_velocity(after_ms(session.timeout_ms));
    if (!triggered) {
        (void)motor->abort_staged(after_ms(session.timeout_ms));
        report(triggered, "启动单轴运动");
        return Disposition::keep_going;
    }
    yz_aim::Motor* other = other_motor_of(session, motor);
    // 循环严格到点退出，随后立刻停止；多留时间是让电机白转，不是安全余量。
    const Deadline until = after_ms(static_cast<uint64_t>(duration_ms));
    bool tripped = false;
    Snapshot target{}, peer{};
    while (Clock::now() < until) {
        if (g_interrupted) break;
        const Result ok_target = motor->poll_safety(target, after_ms(2000));
        const Result ok_peer = other != nullptr ? other->poll_safety(peer, after_ms(2000)) : Result{};
        if (!ok_target || !ok_peer) {
            const Result& bad = ok_target ? ok_peer : ok_target;
            std::cout << "\n[!] 单轴单动异常：" << (ok_target ? "另一轴" : "目标轴") << " "
                      << bad.message << "\n";
            tripped = true;
            break;
        }
        std::cout << "\r[单轴] 节点 " << node << " " << target.position_pulses << " 脉冲  "
                  << fixed(target.speed_rpm, 0) << " RPM   " << std::flush;
        std::this_thread::sleep_until(
            std::min(until, Clock::now() + std::chrono::milliseconds(70)));
    }
    std::cout << "\n";
    if (tripped) {
        sync_halt_both(session);
        return Disposition::keep_going;
    }
    if (g_interrupted) {
        g_interrupted = 0;
        std::cout << "[已取消] 收到中断，两轴同时急停。\n";
        sync_halt_both(session);
        return Disposition::keep_going;
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    if (!report(motor->stop(after_ms(budget)), "单轴运动结束，停止并保持")) return Disposition::keep_going;
    std::cout << "节点 " << node << " 停止并保持，位置 " << target.position_pulses
              << " 脉冲（" << fixed(target.output_position_degrees, 4) << "°）；另一轴未参与本次运动。\n";
    if (before.result) {
        const int64_t travel =
            static_cast<int64_t>(target.position_pulses) - before.position_pulses;
        std::cout << "[本次动作汇总] one 节点 " << node << "：本次只动了一轴，刚性刷架两端因此错开 "
                  << travel_text(session.lead_mm, travel)
                  << "（另一轴未动，不做同步判定）。\n"
                     "    要恢复两轴齐平请用 sync-rel / sync-mm 走等量行程；sync-status 可看两端当前错位。\n";
    }
    return Disposition::keep_going;
}

// one-stop：单动/常转的收尾，也是失控时的急停 —— 两轴同时急停并确认（输出仍使能）。
Disposition command_one_stop(Session& session) {
    if (!session.second) {
        if (!session.enabled) {
            std::cout << "本会话未使能，无需停止。\n";
            return Disposition::keep_going;
        }
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
        report(session.motor.stop(after_ms(budget)), "急停保持（0x6040=0x010F）");
        return Disposition::keep_going;
    }
    sync_run_report_elapsed(session);
    session.sync_run_active = false;
    if (!session.dual_enabled && !session.motor.cleanup_required() &&
        !session.second->cleanup_required()) {
        std::cout << "两轴均未使能，无需停止。\n";
        return Disposition::keep_going;
    }
    session.end_reason = "operator sync-stop";
    std::cout << "两轴同时停止并确认…\n";
    sync_halt_both(session);
    sync_report_move(session, "两轴停止收尾", true);
    return Disposition::keep_going;
}

// jog <rpm> [毫秒]：点动，按时长上限 5000 ms 收口。
// 两轴已使能时走双轴镜像点动（两轴同时动、方向相反，rpm 的符号按"伸出方向"，与 extend/sync-spin 一致）；
// 否则走原有的单轴点动（只动 A 轴，rpm 上限 ±3000）。这样现场不必记两套名字。
Disposition command_jog_dispatch(Session& session, const std::vector<std::string>& arguments) {
    // 单轴模式（--node2 0）：保持原有行为，只动 A 轴。
    if (!session.second) return command_jog(session, arguments);
    auto usage = [&]() {
        std::cout << "用法：jog <rpm±1~±" << yz_aim::kSyncMaxRpm << "> [毫秒 1~5000，默认 500]\n"
                     "    双轴镜像点动：两轴同时动、方向相反，正 rpm = 伸出方向。\n"
                     "    要更长的时长用 extend / retract / sync-spin（上限 600000 ms），\n"
                     "    要连续转用 sync-run + sync-stop，只动一台用 one。\n";
    };
    if (arguments.empty() || arguments.size() > 2) { usage(); return Disposition::keep_going; }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -yz_aim::kSyncMaxRpm, yz_aim::kSyncMaxRpm, rpm) || rpm == 0) {
        usage();
        return Disposition::keep_going;
    }
    uint32_t duration_ms = 500;
    if (arguments.size() == 2 && !parse_u32(arguments[1], 1, 5000, duration_ms)) {
        usage();
        return Disposition::keep_going;
    }
    if (!session.dual_enabled) {
        std::cout << "[失败] jog：双轴模式下 jog 会同时动两轴（方向相反），请先执行 sync-enable。\n"
                     "    只动一台电机用 one <节点号> <rpm> <毫秒>。\n";
        return Disposition::keep_going;
    }
    const std::vector<std::string> forwarded{std::to_string(rpm), std::to_string(duration_ms)};
    return command_sync_velocity(session, forwarded, false, "jog");
}

void sync_run_tick(Session& session, bool& prompt_shown) {
    (void)prompt_shown;
    if (!session.sync_run_active || !session.second) return;
    const auto now = Clock::now();
    if (now < session.sync_run_next_tick) return;
    session.sync_run_next_tick = now + std::chrono::milliseconds(100);
    Result r = sync_velocity_sample(session, Deadline::max(), session.sync_run_over);
    if (!r) {
        if (g_interrupted && r.error == Error::cancelled) {
            g_interrupted = 0;
            command_sync_stop(session);
        } else {
            session.sync_run_active = false;
            sync_trip(session, sync_reason_text(r));
        }
    }
}

Disposition command_set_zero(Session& session) {
    if (blocked_by_sustain(session, "set-zero")) return Disposition::keep_going;
    if (!session.assume_yes) {
        std::cout <<
            "[!] set-zero 会把当前轴位设为零点（手册 §5.2 / §6.2：电子齿轮分子先发 60006 再发 60016）。\n"
            "    这会改变全部绝对坐标的基准；机械是否处在允许置零的位置必须由现场判断。\n"
            "    确认后请加 --yes 重新执行。\n";
        return Disposition::keep_going;
    }
    const Snapshot before = session.motor.snapshot(after_ms(session.timeout_ms));
    if (!before.result) { report(before.result, "置零点前读取状态"); return Disposition::keep_going; }
    if (yz_aim::output_is_enabled(before.output_enabled)) {
        std::cout << "[失败] 输出已使能，请先 disable 再置零点。\n";
        return Disposition::keep_going;
    }
    if (std::abs(before.speed_rpm) > 0.5) {
        std::cout << "[失败] 电机读数未静止（" << fixed(before.speed_rpm, 1) << " 电机RPM），先确认停止。\n";
        return Disposition::keep_going;
    }
    yz_aim::SdoClient sdo(session.transport, session.node);
    Result result = sdo.write(0x260a, 0, 2, 60006, after_ms(1500));
    if (!report(result, "置零点第 1 步（0x260A ← 60006）")) return Disposition::keep_going;
    result = sdo.write(0x260a, 0, 2, 60016, after_ms(1500));
    if (!report(result, "置零点第 2 步（0x260A ← 60016）")) return Disposition::keep_going;
    const Snapshot after = session.motor.snapshot(after_ms(session.timeout_ms));
    if (after.position_sample.valid)
        std::cout << "    现在 0x6064 = " << after.position_pulses << " 电机脉冲（应为 0 或接近 0）\n";
    yz_aim::SdoClient probe(session.transport, session.node);
    uint32_t restored = 0;
    if (probe.read(0x260a, 0, 2, restored, after_ms(1500)))
        std::cout << "    0x260A 已回到 " << restored << "（默认 32768）\n";
    return Disposition::keep_going;
}

Disposition command_set_node(Session& session, const std::string& text) {
    session.command_rc = 2;
    if (!session.assume_yes) {
        std::cout << "set-node需--yes；将自动停止、失能、写地址并保存，重新上电后生效。\n";
        return Disposition::keep_going;
    }
    uint32_t node = 0;
    if (!parse_u32(text, 1, 126, node) || node == session.node || session.node == 127) {
        std::cout << "新节点必须与旧节点不同，范围1~126（127为心跳生产者）。\n";
        return Disposition::keep_going;
    }
    if (session.second) {
        std::cout << "改号请使用--node2 0，只接目标电机，避免与第二轴ID冲突。\n";
        return Disposition::keep_going;
    }
    const auto budget = std::max<uint64_t>(session.timeout_ms, 30000);
    if (!report(session.motor.disable_output(after_ms(budget)), "改号前停止并失能输出"))
        return Disposition::quit;
    session.enabled = false;
    const Result r = session.motor.stage_node_id(static_cast<uint8_t>(node), after_ms(budget));
    if (report(r, "暂存并保存新节点号 " + std::to_string(node))) {
        session.command_rc = 0;
        std::cout << "保存已确认；请重新上电后用--node " << node << "读取确认。本会话结束。\n";
    } else std::cout << "改号未完整确认，不继续操作；请检查实际节点和保存状态。\n";
    return Disposition::quit;
}

Disposition command_set_bitrate(Session& session, const std::string& text) {
    if (!session.assume_yes) {
        std::cout << "[!] set-bitrate 会按手册 §7.13 写入内部参数，需重新上电后生效，\n"
                     "    之后 can0 也必须用同一个波特率重新 up。确认后请加 --yes 重新执行。\n";
        return Disposition::keep_going;
    }
    uint32_t kbps = 0;
    if (!parse_u32(text, 125, 1000, kbps) || (kbps != 125 && kbps != 250 && kbps != 500 && kbps != 1000)) {
        std::cout << "用法：set-bitrate <125|250|500|1000>\n";
        return Disposition::keep_going;
    }
    const uint32_t bitrate = kbps * 1000;
    const Result result = session.motor.stage_bitrate(bitrate, after_ms(session.timeout_ms));
    if (report(result, "暂存 CAN 波特率 " + std::to_string(kbps) + " kbit/s")) {
#ifdef YZ_AIM_USB_DOUBLE
        std::cout << "请电机重新上电，并按新波特率重启USB工具；非1M需提供经验证的--abit-timing。\n";
#else
        std::cout << "    请断电重新上电，然后执行："
                     "ip link set " << session.interface_label << " down && ip link set "
                  << session.interface_label << " type can bitrate " << bitrate
                  << " && ip link set " << session.interface_label << " up\n";
#endif
    }
    return Disposition::keep_going;
}

Disposition dispatch(Session& session, const std::vector<std::string>& tokens) {
    session.command_rc = 0;
    const std::string& name = tokens[0];
    const std::vector<std::string> arguments(tokens.begin() + 1, tokens.end());
    auto want_no_arguments = [&]() {
        if (arguments.empty()) return true;
        std::cout << name << " 不接受参数\n";
        return false;
    };
    // 双轴使能期间单轴命令只会动 A 轴，刚性刷架会被别住，因此一律拒绝并指向对应的 sync-* 命令。
    auto dual_blocks = [&](const char* alternative) {
        if (!session.dual_enabled) return false;
        std::cout << "[失败] " << name << "：双轴已使能，单轴命令只动 A 轴，会别住刚性刷架；"
                     "请改用 " << alternative << "，或先执行 sync-disable。\n";
        return true;
    };

    if (name == "help" || name == "?") { print_help(); return Disposition::keep_going; }
    if (name == "quit" || name == "exit" || name == "q") return Disposition::quit;
    if (name == "status" || name == "s") { if (want_no_arguments()) command_status(session); return Disposition::keep_going; }
    if (name == "monitor") return command_monitor(session, arguments);
    if (name == "read-od" || name == "od") return command_read_od(session, arguments);

    if (name == "enable" || name == "e") {
        if (!want_no_arguments()) return Disposition::keep_going;
        if (dual_blocks("sync-enable")) return Disposition::keep_going;
        if (session.enabled) { std::cout << "输出已由本会话使能。\n"; return Disposition::keep_going; }
        if (!report(session.motor.arm(after_ms(session.timeout_ms)), "安全检查（ARM，只读）"))
            return Disposition::keep_going;
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 20000);
        if (report(session.motor.enable_output(after_ms(budget)), "使能输出")) {
            session.enabled = true;
            std::cout << "    本进程会在等待输入期间每约 200 ms 发送心跳 0x77F:05；"
                         "退出请用 disable 或 quit。\n";
        }
        return Disposition::keep_going;
    }
    if (name == "disable" || name == "d") {
        if (!want_no_arguments()) return Disposition::keep_going;
        if (dual_blocks("sync-disable")) return Disposition::keep_going;
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
        const Result result = session.motor.disable_output(after_ms(budget));
        if (result) {
            session.enabled = false;
            report(result, "停止并失能输出");
        } else {
            report(result, "停止并失能输出");
            std::cout << "    [!] 未能确认输出已禁止；如现场允许，请按急停或断电处置。\n";
        }
        return Disposition::keep_going;
    }
    if (name == "stop" || name == "x") {
        if (!want_no_arguments()) return Disposition::keep_going;
        if (dual_blocks("sync-stop")) return Disposition::keep_going;
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
        report(session.motor.stop(after_ms(budget)), "急停保持（0x6040=0x010F）");
        return Disposition::keep_going;
    }

    if (name == "rel" || name == "r") {
        if (dual_blocks("sync-rel")) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：rel <输出角度，度>\n"; return Disposition::keep_going; }
        return command_relative(session, arguments[0]);
    }
    if (name == "abs" || name == "a") {
        if (dual_blocks("sync-abs")) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：abs <输出角度，度>\n"; return Disposition::keep_going; }
        return command_absolute(session, arguments[0], false);
    }
    if (name == "pulses" || name == "p") {
        if (dual_blocks("sync-abs")) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：pulses <电机脉冲>\n"; return Disposition::keep_going; }
        return command_absolute(session, arguments[0], true);
    }
    if (name == "jog" || name == "j") {
        // 双轴使能后 jog 自动变成双轴镜像点动（见 command_jog_dispatch），不再是只动 A 轴。
        return command_jog_dispatch(session, arguments);
    }
    if (name == "spin" || name == "sp") {
        if (dual_blocks("sync-spin")) return Disposition::keep_going;
        return command_spin(session, arguments);
    }
    if (name == "run" || name == "go") {
        if (dual_blocks("sync-run")) return Disposition::keep_going;
        return command_run(session, arguments);
    }

    if (name == "sync-enable") { if (want_no_arguments()) command_sync_enable(session); return Disposition::keep_going; }
    if (name == "sync-disable") { if (want_no_arguments()) command_sync_disable(session); return Disposition::keep_going; }
    if (name == "sync-stop") { if (want_no_arguments()) command_sync_stop(session); return Disposition::keep_going; }
    if (name == "sync-status") { if (want_no_arguments()) command_sync_status(session); return Disposition::keep_going; }
    if (name == "sync-zero") { if (want_no_arguments()) command_sync_zero(session); return Disposition::keep_going; }
    if (name == "sync-tol") {
        if (arguments.size() != 1) { std::cout << "用法：sync-tol <输出角度，度>\n"; return Disposition::keep_going; }
        return command_sync_tol(session, arguments[0]);
    }
    if (name == "sync-correct") {
        if (arguments.size() != 1) { std::cout << "用法：sync-correct <0~10RPM>\n"; return Disposition::keep_going; }
        return command_sync_correct(session, arguments[0]);
    }
    if (name == "sync-drift") {
        if (arguments.size() != 1) { std::cout << "用法：sync-drift <电机RPM>\n"; return Disposition::keep_going; }
        return command_sync_drift(session, arguments[0]);
    }
    if (name == "set-lead") {
        if (arguments.size() != 1) { std::cout << "用法：set-lead <导程 mm/圈>\n"; return Disposition::keep_going; }
        return command_set_lead(session, arguments[0]);
    }
    if (name == "sync-rel") return command_sync_move(session, arguments, false);
    if (name == "sync-abs") return command_sync_move(session, arguments, true);
    if (name == "sync-mm") return command_sync_mm(session, arguments);
    if (name == "sync-spin") return command_sync_velocity(session, arguments, false);
    if (name == "sync-run") return command_sync_velocity(session, arguments, true);
    if (name == "sync-dir") return command_sync_dir(session, arguments);
    if (name == "extend") return command_sync_extend(session, arguments, true);
    if (name == "retract") return command_sync_extend(session, arguments, false);

    // 单轴单动：只让一个节点动，另一轴保持不动，用于对位/测试。
    if (name == "one") return command_one_move(session, arguments);
    if (name == "one-stop") { if (want_no_arguments()) command_one_stop(session); return Disposition::keep_going; }

    if (name == "speed" || name == "v") {
        uint32_t rpm = 0;
        if (arguments.size() != 1 || !parse_u32(arguments[0], 1, yz_aim::kMaxMotorRpm, rpm)) {
            std::cout << "用法：speed <1~" << yz_aim::kMaxMotorRpm << ">（只改本会话，不写设备）\n";
            return Disposition::keep_going;
        }
        session.motion_rpm = rpm;
        std::cout << "本会话运动速度 = " << rpm << " 电机RPM（"
                  << fixed(yz_aim::motor_to_output_rpm(rpm), 3) << " 输出RPM）；未写设备。\n";
        return Disposition::keep_going;
    }
    if (name == "set-profile-speed") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后逐轴设置")) return Disposition::keep_going;
        uint32_t rpm = 0;
        if (arguments.size() != 1 || !parse_u32(arguments[0], 0, yz_aim::kMaxMotorRpm, rpm)) {
            std::cout << "用法：set-profile-speed <0~" << yz_aim::kMaxMotorRpm << ">\n";
            return Disposition::keep_going;
        }
        report(session.motor.set_profile_speed(rpm, after_ms(session.timeout_ms)), "写入 0x6081 位置轮廓速度");
        return Disposition::keep_going;
    }
    if (name == "set-accel") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后逐轴设置")) return Disposition::keep_going;
        uint32_t acceleration = 0;
        if (arguments.size() != 1 || !parse_u32(arguments[0], 10, 1000, acceleration)) {
            std::cout << "用法：set-accel <10~1000>（电机RPM/s；低值会与限位功能开关重叠）\n";
            return Disposition::keep_going;
        }
        report(session.motor.set_acceleration(acceleration, after_ms(session.timeout_ms)), "写入 0x6083 加速度");
        return Disposition::keep_going;
    }
    if (name == "set-current") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后逐轴设置")) return Disposition::keep_going;
        uint32_t milliamps = 0;
        if (arguments.size() != 1 || !parse_u32(arguments[0], 10, 10000, milliamps) || milliamps % 10) {
            std::cout << "用法：set-current <10~10000，10 的整数倍>（mA）\n";
            return Disposition::keep_going;
        }
        report(session.motor.set_current_limit(static_cast<uint16_t>(milliamps), after_ms(session.timeout_ms)),
               "写入 0x261D 最大电流");
        return Disposition::keep_going;
    }

    if (name == "set-zero") {
        if (!want_no_arguments()) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后逐轴置零")) return Disposition::keep_going;
        return command_set_zero(session);
    }
    if (name == "set-node") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后逐轴设置节点号")) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：set-node <1~127>\n"; return Disposition::keep_going; }
        return command_set_node(session, arguments[0]);
    }
    if (name == "set-bitrate") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (dual_blocks("sync-disable 后设置波特率")) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：set-bitrate <125|250|500|1000>\n"; return Disposition::keep_going; }
        return command_set_bitrate(session, arguments[0]);
    }

    std::cout << "未知命令：" << name << "（输入 help 查看命令表）\n";
    return Disposition::keep_going;
}

std::vector<std::string> split_fields(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (stream >> field) fields.push_back(field);
    return fields;
}

// 退出前必须确认输出失能；确认失败时报告现场需自行处置，不把未确认的停止当作成功。
int cleanup_and_exit(Session& session) {
    if (session.second && (session.dual_enabled || session.motor.cleanup_required() || session.second->cleanup_required())) {
        std::cout << "退出前两轴同时停止并失能输出…\n";
        // 顺序：先对两轴各发一帧未确认急停（串行地"先确认 A 再确认 B"会让 B 多跑最多 7 秒），
        // 再交替复查到两轴都静止，最后各自失能。
        sync_halt_both(session);
        session.sync_run_active = false;
        if (session.motion_log) {
            session.end_reason = "operator quit";
            sync_report_move(session, "quit 收尾", true);
        }
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
        const Result ra = session.motor.disable_output(after_ms(budget));
        const Result rb = session.second->disable_output(after_ms(budget));
        session.dual_enabled = false;
        session.enabled = false;
        if (ra && rb) return 0;
        std::cout << "[!] 未能确认两轴输出都已禁止；心跳已停止，驱动约 1 s 后应报 0x20 并停机，"
                     "但现场仍须按急停/断电处置。\n";
        return 4;
    }
    if (!session.enabled && !session.motor.cleanup_required()) return 0;
    std::cout << "退出前停止并失能输出…\n";
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    const Result result = session.motor.disable_output(after_ms(budget));
    if (report(result, "停止并失能输出")) {
        session.enabled = false;
        return 0;
    }
    std::cout << "[!] 未能确认输出已禁止；心跳已停止，驱动约 1 s 后应报 0x20 并停机，"
                 "但现场仍须按急停/断电处置。\n";
    return 4;
}

int run_session(Session& session) {
    const bool interactive = ::isatty(STDIN_FILENO) == 1;
    std::string buffer;
    bool eof = false;
    bool prompt_shown = false;

    for (;;) {
        if (g_motor != nullptr && session.motor.watchdog_active()) {
            const Result serviced = session.motor.service_watchdog(after_ms(kInputPollMs));
            if (!serviced) {
                report(serviced, "维持所属心跳");
                break;
            }
        }
        // Both objects service the same bus producer; the shared timestamp suppresses duplicates.
        if (session.second && session.second->watchdog_active()) {
            const Result serviced = session.second->service_watchdog(after_ms(kInputPollMs));
            if (!serviced) {
                report(serviced, "维持第二轴心跳");
                break;
            }
        }
        if (g_interrupted) {
            if (session.motor.sustaining()) {
                // 常转期间 Ctrl-C 等同于 stop：确认停止并保持，会话继续可用。
                g_interrupted = 0;
                std::cout << "\n[常转] 收到中断，执行确认停止…\n";
                const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
                report(session.motor.stop(after_ms(budget)), "停止常转");
                continue;
            }
            if (session.sync_run_active) {
                // 双轴常转期间 Ctrl-C 等同于 sync-stop：两轴同时急停并确认。
                g_interrupted = 0;
                std::cout << "\n[双轴常转] 收到中断，两轴同时急停…\n";
                command_sync_stop(session);
                prompt_shown = false;
                continue;
            }
            std::cout << "\n收到中断信号。\n";
            break;
        }
        if (eof && buffer.empty()) break;
        sustain_tick(session, prompt_shown);
        sync_run_tick(session, prompt_shown);

        if (!eof) {
            if (interactive && !prompt_shown) {
                std::cout << (session.sync_run_active ? "yz(双轴常转)> "
                              : session.motor.sustaining() ? "yz(常转)> "
                              : session.enabled ? "yz(已使能)> " : "yz(只读)> ") << std::flush;
                prompt_shown = true;
            }
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&input, 1, kInputPollMs);
            if (ready < 0) {
                if (errno == EINTR) continue;
                std::cout << "读取标准输入失败，退出。\n";
                break;
            }
            if (ready == 0) continue;
            char chunk[256];
            const ssize_t received = ::read(STDIN_FILENO, chunk, sizeof(chunk));
            if (received < 0) {
                if (errno == EINTR || errno == EAGAIN) continue;
                std::cout << "读取标准输入失败，退出。\n";
                break;
            }
            if (received == 0) { eof = true; continue; }
            buffer.append(chunk, static_cast<std::size_t>(received));
        }

        const std::size_t newline = buffer.find('\n');
        if (newline == std::string::npos) {
            if (!eof) continue;
        }
        std::string line;
        if (newline == std::string::npos) {
            line = buffer;
            buffer.clear();
        } else {
            line = buffer.substr(0, newline);
            buffer.erase(0, newline + 1);
        }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        prompt_shown = false;

        const std::vector<std::string> tokens = split_fields(line);
        if (tokens.empty()) continue;
        if (dispatch(session, tokens) == Disposition::quit) break;
    }

    const int code = cleanup_and_exit(session);
    return code;
}

int run_single_command(Session& session, const std::vector<std::string>& tokens) {
    if (tokens.empty()) return 0;
    if (tokens[0] == "run" || tokens[0] == "go") {
        std::cout << "run 只能在交互模式下使用：单命令模式执行后会立刻停止并失能输出。\n";
        return 0;
    }
    // 双轴命令依赖跨命令的会话状态（两轴使能标志、偏差基准），只能在交互模式下使用。
    const bool dual_only = tokens[0].rfind("sync-", 0) == 0 || tokens[0] == "extend" ||
                           tokens[0] == "retract" || tokens[0] == "one" || tokens[0] == "one-stop";
    if (dual_only && tokens[0] != "sync-status") {
        std::cout << tokens[0] << " 只能在交互模式下使用：双轴使能状态与偏差基准要跨命令保持。\n"
                     "  单命令模式可用 sync-status（只读）。\n";
        return 0;
    }
    if (tokens[0] == "enable" || tokens[0] == "e") {
        std::cout << "提示：单命令模式下进程随后会退出，退出前会自动停止并失能输出；"
                     "连续操作请使用交互模式。\n";
    }
    (void)dispatch(session, tokens);
    const int cleanup = cleanup_and_exit(session);
    return cleanup ? cleanup : session.command_rc;
}

#ifndef YZ_AIM_USB_DOUBLE
void print_banner(const Options& options, const SocketCanTransport& transport) {
    std::cout << "=== YZ-AIM CANopen 板载 CAN 直连维护工具 " << kVersion
              << " (" __DATE__ " " __TIME__ ") ===\n";
    std::cout << "接口 " << transport.interface_name() << "：已 UP";
    if (transport.reported_bitrate())
        std::cout << "，内核报告波特率 " << transport.reported_bitrate() << " bit/s";
    else
        std::cout << "，内核未报告波特率（可能由设备树固定）";
    std::cout << "\n节点 " << options.node << "   期望波特率 " << options.bitrate << " bit/s\n";
    if (options.node2)
        std::cout << "双轴镜像同步模式：A 轴节点 " << options.node << "，B 轴节点 " << options.node2
                  << "（一条 " << transport.interface_name()
                  << " 上两个节点，默认 sync-dir " << options.node2 << " " << options.node
                  << " = 节点 " << options.node2 << " 取 +、节点 " << options.node << " 取 −）\n";
    if (transport.reported_bitrate() && transport.reported_bitrate() != options.bitrate)
        std::cout << "[!] 内核报告的波特率与 --bitrate 不一致；若读不到电机，先核对 "
                     "ip -details link show " << transport.interface_name() << "\n";
    if (transport.controller_state() > 0)
        std::cout << "[!] 控制器状态 " << yz_aim::can_state_text(transport.controller_state())
                  << "：此状态下收发都不可信（总线上没有节点应答时发帧就会掉到这里）。\n"
                     "    先执行 sh tools/can0_up.sh " << transport.interface_name() << " " << options.bitrate
                  << " 复位到 ERROR-ACTIVE 再试。\n";
    std::cout << "启动为只读：不写控制字、不使能、不发心跳。输入 help 查看命令，quit 退出。\n";
}

#endif
} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, options)) return 2;
    if (options.help) { print_usage(); return 0; }
    if (options.version) { std::cout << kProgram << " " << kVersion << " (" __DATE__ " " __TIME__ ")\n"; return 0; }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    if (options.node2 && options.node2 == options.node) {
        std::cerr << "[失败] 两轴节点号必须不同。\n"; return 2;
    }
#ifdef YZ_AIM_USB_DOUBLE
    std::string error;
    const std::string label="usb"+std::to_string(options.device)+"/ch"+std::to_string(options.channel);
    yz_aim::FrameTrace trace(label, options.trace);
    const auto local_library=motion_log_directory().parent_path()/"lib/liblyscan.so";
    if(options.library=="liblyscan.so" && std::filesystem::is_regular_file(local_library))
        options.library=local_library.string();
    auto physical=yz_aim::open_lys_can(options.library,options.bitrate,error,
        options.device,options.channel,options.nominal_timing,&trace);
    if (!physical) { std::cerr << "[失败] 打开USB-CAN：" << error << "\n"; return 3; }
    yz_aim::SharedCanBus bus(*physical,static_cast<uint8_t>(options.node),static_cast<uint8_t>(options.node2));
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    Motor motor(bus.first(),static_cast<uint8_t>(options.node),heartbeat);
    g_motor=&motor;
    Session session{motor,bus.first(),static_cast<uint8_t>(options.node),options.timeout_ms,options.assume_yes,label};
    session.log_directory=motion_log_directory();
    session.trace_idle=options.trace;
    if (options.node2) {
        session.transport2=&bus.second();
        session.second=std::make_unique<Motor>(bus.second(),static_cast<uint8_t>(options.node2),heartbeat);
        session.node2=static_cast<uint8_t>(options.node2);
    }
    std::cout << "=== YZ-AIM USB-CAN 双轴镜像工具 " << kVersion << " ===\n"
              << "适配器 " << options.library << " @ " << label << " " << options.bitrate << " bit/s\n"
              << "A节点 " << options.node << "，B节点 " << options.node2 << "；extend 默认A负、B正。\n"
              << "启动只读；sync-enable → sync-zero → extend/retract RPM 毫秒。\n";
    if (options.trace) std::cout << "CAN TX/RX帧持续输出到stderr；TX仅表示SDK接受，需RX应答确认。\n";
    const int code=options.command.empty()?run_session(session):run_single_command(session,options.command);
    g_motor=nullptr;
    return code;
#else

    const auto node = static_cast<uint8_t>(options.node);
    yz_aim::FrameTrace trace(options.interface_name, options.trace);
    SocketCanTransport transport(options.interface_name, node, &trace);
    std::string error;
    if (!transport.open(error)) {
        std::cerr << "[失败] 打开 " << options.interface_name << "：" << error << "\n";
        return 3;
    }
    // 每轴一条自己的套接字：接收过滤器只放行本节点的 SDO 应答、TPDO 与心跳，与单轴版一致。
    // 两轴合用一条 socket 会把对方节点的心跳与应答混进本会话队列，收帧窗口被挤占。
    std::unique_ptr<SocketCanTransport> transport2;
    if (options.node2) {
        const auto node2 = static_cast<uint8_t>(options.node2);
        if (node2 == node) {
            std::cerr << "[失败] --node2 与 --node 相同（" << static_cast<unsigned>(node)
                      << "）；两轴必须是设备里已改成不同节点号的两台驱动器。\n";
            return 2;
        }
        transport2 = std::make_unique<SocketCanTransport>(options.interface_name, node2, &trace);
        std::string error2;
        if (!transport2->open(error2)) {
            std::cerr << "[失败] 打开 " << options.interface_name << "（B 轴节点 "
                      << static_cast<unsigned>(node2) << "）：" << error2 << "\n";
            return 3;
        }
    }
    print_banner(options, transport);
    if (options.trace) std::cout << "CAN TX/RX帧持续输出到stderr；TX表示内核已接受，需RX应答确认。\n";

    auto shared_heartbeat = std::make_shared<yz_aim::SharedHeartbeat>();
    Motor motor(transport, node, shared_heartbeat);
    g_motor = &motor;

    Session session{motor, transport, node, options.timeout_ms, options.assume_yes};
    session.log_directory = motion_log_directory();
    session.trace_idle=options.trace;
    if (transport2) {
        session.transport2 = transport2.get();
        session.second = std::make_unique<yz_aim::Motor>(*transport2, options.node2, shared_heartbeat);
        session.node2 = static_cast<uint8_t>(options.node2);
    }
    const int code = options.command.empty() ? run_session(session) : run_single_command(session, options.command);
    g_motor = nullptr;
    return code;
#endif
}
