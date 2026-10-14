// yz_aim_can0 —— 用板载 CAN（SocketCAN，如 can0）直连维护 YZ-AIM CANopen 一体伺服。
#include "yz_aim.h"
#include "socketcan_transport.h"
#include "can_link_query.h"

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

constexpr const char* kProgram = "yz_aim_can0";
constexpr const char* kVersion = "1.0.0";
constexpr uint32_t kDefaultNode = 1;
constexpr uint32_t kDefaultBitrate = 1000000;
constexpr uint64_t kDefaultTimeoutMs = 20000;
constexpr int kInputPollMs = 100;

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
        return "本会话的 SDO 会话已被判为不可信，请退出并重新启动本工具。";
    switch (error) {
        case Error::timeout:
            return "检查：can0 波特率与电机一致、CANH/CANL 未接反、总线末端 120Ω 终端电阻、节点号正确；\n"
                   "        另外先看 ip -details link show can0 的 can state —— 若是 ERROR-PASSIVE/BUS-OFF，\n"
                   "        控制器收发已不可信（常见于之前发过帧没被应答），跑 sh tools/can0_up.sh can0 1000000 复位后再试。";
        case Error::transport:
            return "检查接线与 ip -details -statistics link show can0 的收发计数、bus-off 状态。";
        case Error::alarm:
            return "读 0x260E 与红灯闪烁次数对照手册：0x20 通信掉线、0x10 电池掉电、0x12 堵转、0x14 失速、0x15 过压。";
        case Error::unsafe:
            return "本会话可能已锁止；先 stop／disable，必要时退出工具后重新开始。";
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
    uint32_t bitrate = kDefaultBitrate;
    uint64_t timeout_ms = kDefaultTimeoutMs;
    bool assume_yes = false;
    bool help = false;
    bool version = false;
    std::vector<std::string> command;
};

void print_usage() {
    std::cout <<
        "用法：" << kProgram << " [选项] [命令 [参数…]]\n"
        "\n"
        "  不带命令时进入交互模式（逐行输入命令，输入 help 查看全部命令）。\n"
        "  只读命令可直接作为单条命令执行；控制类命令在单命令模式下会先执行、退出前\n"
        "  自动停止并失能输出。\n"
        "\n"
        "选项：\n"
        "  --if NAME       CAN 接口名（默认 can0）\n"
        "  --node N        CANopen 节点 1~127（默认 1）\n"
        "  --bitrate BPS   期望波特率 125000/250000/500000/1000000（默认 1000000），\n"
        "                  仅用于与内核报告的波特率比对；can0 的波特率由 ip 命令或设备树决定\n"
        "  --timeout MS    单次动作总预算（默认 20000；单条 SDO 仍最多等 120 ms）\n"
        "  --yes           允许 set-zero / set-node / set-bitrate 这类写内部参数的操作\n"
        "  --version | --help\n"
        "\n"
        "命令：\n"
        "  status                             只读：读取一次完整快照\n"
        "  monitor [秒] [间隔ms]              只读：周期刷新（默认 10 秒 / 500 ms）\n"
        "  read-od <索引>[:<子索引>] [宽度]   只读：读任意对象字典（宽度 1/2/4，缺省自动试探）\n"
        "  enable                             安全检查后使能输出（之后本进程维持心跳）\n"
        "  disable                            停止并失能输出\n"
        "  stop                               急停保持 0x6040=0x010F（输出仍使能）\n"
        "  rel <输出角度>                     相对位置运动（度，可小数、可负；基准是动作前重读的 0x6064）\n"
        "  abs <输出角度>                     绝对位置运动（输出轴角度坐标，度）\n"
        "  pulses <电机脉冲>                  绝对位置运动（0x6064 原始坐标）\n"
        "  jog <rpm> [毫秒]                   速度模式点动（默认 500 ms，最长 5000 ms）\n"
        "  spin <rpm> [秒]                    速度模式连续运转（默认 60 s，最长 600 s；Ctrl-C 随时停）\n"
        "  run <rpm>                          速度模式常转：启动后立即回到提示符，一直转到 stop 为止\n"
        "  speed <rpm>                        选择本会话的运动速度 1~3000（只改本会话，不写设备）\n"
        "  set-profile-speed <rpm>            写 0x6081 位置轮廓速度 0~3000（需输出失能且静止）\n"
        "  set-accel <rpm/s>                  写 0x6083 加速度 10~1000（需输出失能且静止）\n"
        "  set-current <mA>                   写 0x261D 最大电流 10~10000，10 的整数倍（需输出失能且静止）\n"
        "  set-zero                           置当前轴位为零点（需 --yes；手册 §5.2/§6.2 的 60006→60016）\n"
        "  set-node <1~127>                   暂存新节点号并保存（改号前自动停止并失能输出；需 --yes；断电重新上电后生效）\n"
        "  set-bitrate <125|250|500|1000>     暂存 CAN 波特率（需 --yes；断电重新上电后生效）\n"
        "  help | quit / exit\n"
        "\n"
        "换算基准：32768 电机脉冲/电机圈，51:1 减速 → 1,671,168 电机脉冲/输出圈。\n";
}

void print_help() {
    print_usage();
    std::cout <<
        "\n运行提示：\n"
        "  1) 本工具直接用板载 CAN 控制器，不经过 USB-CAN 适配器，也不需要 liblyscan.so。\n"
        "  2) 交互模式下 enable 之后，等待输入时每 " << kInputPollMs << " ms 的循环会维持 0x77F:05 心跳；\n"
        "     直接 kill 进程会导致心跳中断，驱动在约 1 s 后报 0x20 并停机。请用 disable 或 quit 退出。\n"
        "  3) 每次运动前驱动都会重新做只读安全检查（ARM）并重读 0x6064，目标坐标须在手册范围内。\n"
        "  4) 停止与输出失能是两个不同操作，也都不代表机械抱闸已生效。\n"
        "  5) 一次运动超时可能使本会话锁止；此时退出并按现场措施处置，不要依赖本工具。\n"
        "  6) run 常转期间提示符变为 yz(常转)>，每 200 ms 复查一次安全项、每秒打印一行状态；\n"
        "     只允许 stop / disable / quit / status / monitor / read-od，其余写操作一律拒绝。\n"
        "     常转没有时长上限，请务必留人看管，并注意 0x6064 不要逼近 ±" << yz_aim::kMaxPositionPulses << "。\n";
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
        if (argument == "--yes") { options.assume_yes = true; continue; }
        if (argument == "--if") {
            if (!next_value("--if", options.interface_name)) return false;
            continue;
        }
        if (argument == "--node") {
            std::string value;
            if (!next_value("--node", value) || !parse_u32(value, 1, 127, options.node)) {
                std::cerr << "--node 需为 1~127 的整数\n";
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
        if (!argument.empty() && argument[0] == '-' && argument != "-") {
            std::cerr << "未知选项：" << argument << "\n";
            return false;
        }
        options.command.push_back(argument);
    }
    return true;
}

struct Session {
    Motor& motor;
    SocketCanTransport& transport;
    uint8_t node = kDefaultNode;
    bool enabled = false;
    uint32_t motion_rpm = static_cast<uint32_t>(yz_aim::kPositionProfileRpm);
    uint64_t timeout_ms = kDefaultTimeoutMs;
    bool assume_yes = false;
    Snapshot sustain_last{};
    Clock::time_point sustain_started{};
    Clock::time_point sustain_next_sweep{};
    Clock::time_point sustain_next_line{};
};

enum class Disposition { keep_going, quit };

// 常转期间除停止类命令外的写操作一律拒绝，避免半途改变设定点或误触发安全锁止。
bool blocked_by_sustain(Session& session, const std::string& name) {
    if (!session.motor.sustaining()) return false;
    std::cout << "[失败] " << name << "：常转进行中，请先输入 stop。\n";
    return true;
}

Disposition command_status(Session& session) {
    const Snapshot snapshot = session.motor.snapshot(after_ms(session.timeout_ms));
    print_snapshot(snapshot, session.node, session.transport.interface_name());
    if (!snapshot.result) report(snapshot.result, "读取状态");
    else if (snapshot.unsafe_latched) std::cout << "[!] 本会话已锁止，只能 stop / disable / 退出。\n";
    return Disposition::keep_going;
}

Disposition command_monitor(Session& session, const std::vector<std::string>& arguments) {
    uint32_t seconds = 10;
    uint32_t period_ms = 500;
    if (!arguments.empty() && !parse_u32(arguments[0], 1, 3600, seconds)) {
        std::cout << "用法：monitor [秒] [间隔ms]\n";
        return Disposition::keep_going;
    }
    if (arguments.size() >= 2 && !parse_u32(arguments[1], 100, 5000, period_ms)) {
        std::cout << "用法：monitor [秒] [间隔ms]\n";
        return Disposition::keep_going;
    }
    const Deadline until = after_ms(static_cast<uint64_t>(seconds) * 1000);
    std::cout << "只读监视 " << seconds << " 秒，每 " << period_ms << " ms 一次…\n";
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
        std::cout << "用法：spin <rpm> [秒]   （rpm 正=正转、负=反转；秒 默认 60、最长 600）\n";
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -static_cast<int32_t>(yz_aim::kMaxMotorRpm),
                   static_cast<int32_t>(yz_aim::kMaxMotorRpm), rpm) || rpm == 0) {
        std::cout << "spin 转速需为 ±1~±" << yz_aim::kMaxMotorRpm << " 电机RPM（不能为 0）\n";
        return Disposition::keep_going;
    }
    uint32_t seconds = 60;
    if (arguments.size() >= 2 && !parse_u32(arguments[1], 1, 600, seconds)) {
        std::cout << "spin 时长需为 1~600 秒\n";
        return Disposition::keep_going;
    }
    const auto duration = std::chrono::seconds(seconds);
    const auto budget = yz_aim::jog_action_budget(duration);
    const uint64_t budget_ms = static_cast<uint64_t>(budget.count()) + 5000;
    std::cout << "连续运转 " << rpm << " 电机RPM（" << fixed(yz_aim::motor_to_output_rpm(rpm), 3)
              << " 输出RPM），最多 " << seconds << " 秒；按 Ctrl-C 可随时停。\n"
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
    if (!session.assume_yes) {
        std::cout << "[!] set-node 会写入设备地址并保存（0x2615 → 0x2614=1），"
                     "改号前会先停止并失能输出，需重新上电后生效。\n"
                     "    确认后请加 --yes 重新执行。\n";
        return Disposition::keep_going;
    }
    uint32_t node = 0;
    if (!parse_u32(text, 1, 127, node)) {
        std::cout << "用法：set-node <1~127>\n";
        return Disposition::keep_going;
    }
    // 手册要求改设备地址前必须"电机静止 + 输出已禁止"，这里自动先做一次停止并失能（与 disable 同一条路径），
    // 不再因为读到输出使能就直接拒绝。
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    if (!report(session.motor.disable_output(after_ms(budget)), "改号前停止并失能输出")) {
        std::cout << "    [!] 未能确认输出已禁止，改设备地址不安全，本次不执行。\n";
        return Disposition::keep_going;
    }
    session.enabled = false;
    const Result result = session.motor.stage_node_id(static_cast<uint8_t>(node), after_ms(budget));
    if (report(result, "暂存新节点号 " + std::to_string(node))) {
        std::cout << "    地址已写入并保存；给驱动器断电重新上电后生效，上电后用 --node " << node
                  << " 重新连接（在那之前本工具连不上它）。\n";
    }
    return Disposition::keep_going;
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
        std::cout << "    请断电重新上电，然后执行："
                     "ip link set " << session.transport.interface_name() << " down && ip link set "
                  << session.transport.interface_name() << " type can bitrate " << bitrate
                  << " && ip link set " << session.transport.interface_name() << " up\n";
    }
    return Disposition::keep_going;
}

Disposition dispatch(Session& session, const std::vector<std::string>& tokens) {
    const std::string& name = tokens[0];
    const std::vector<std::string> arguments(tokens.begin() + 1, tokens.end());
    auto want_no_arguments = [&]() {
        if (arguments.empty()) return true;
        std::cout << name << " 不接受参数\n";
        return false;
    };

    if (name == "help" || name == "?") { print_help(); return Disposition::keep_going; }
    if (name == "quit" || name == "exit" || name == "q") return Disposition::quit;
    if (name == "status" || name == "s") { if (want_no_arguments()) command_status(session); return Disposition::keep_going; }
    if (name == "monitor") return command_monitor(session, arguments);
    if (name == "read-od" || name == "od") return command_read_od(session, arguments);

    if (name == "enable" || name == "e") {
        if (!want_no_arguments()) return Disposition::keep_going;
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
        const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
        report(session.motor.stop(after_ms(budget)), "急停保持（0x6040=0x010F）");
        return Disposition::keep_going;
    }

    if (name == "rel" || name == "r") {
        if (arguments.size() != 1) { std::cout << "用法：rel <输出角度，度>\n"; return Disposition::keep_going; }
        return command_relative(session, arguments[0]);
    }
    if (name == "abs" || name == "a") {
        if (arguments.size() != 1) { std::cout << "用法：abs <输出角度，度>\n"; return Disposition::keep_going; }
        return command_absolute(session, arguments[0], false);
    }
    if (name == "pulses" || name == "p") {
        if (arguments.size() != 1) { std::cout << "用法：pulses <电机脉冲>\n"; return Disposition::keep_going; }
        return command_absolute(session, arguments[0], true);
    }
    if (name == "jog" || name == "j") return command_jog(session, arguments);
    if (name == "spin" || name == "sp") return command_spin(session, arguments);
    if (name == "run" || name == "go") return command_run(session, arguments);

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
        return command_set_zero(session);
    }
    if (name == "set-node") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
        if (arguments.size() != 1) { std::cout << "用法：set-node <1~127>\n"; return Disposition::keep_going; }
        return command_set_node(session, arguments[0]);
    }
    if (name == "set-bitrate") {
        if (blocked_by_sustain(session, name)) return Disposition::keep_going;
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
    if (!session.enabled) return 0;
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
        if (g_interrupted) {
            if (session.motor.sustaining()) {
                // 常转期间 Ctrl-C 等同于 stop：确认停止并保持，会话继续可用。
                g_interrupted = 0;
                std::cout << "\n[常转] 收到中断，执行确认停止…\n";
                const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
                report(session.motor.stop(after_ms(budget)), "停止常转");
                continue;
            }
            std::cout << "\n收到中断信号。\n";
            break;
        }
        if (eof && buffer.empty()) break;
        sustain_tick(session, prompt_shown);

        if (!eof) {
            if (interactive && !prompt_shown) {
                std::cout << (session.motor.sustaining() ? "yz(常转)> "
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
    if (tokens[0] == "enable" || tokens[0] == "e") {
        std::cout << "提示：单命令模式下进程随后会退出，退出前会自动停止并失能输出；"
                     "连续操作请使用交互模式。\n";
    }
    (void)dispatch(session, tokens);
    return cleanup_and_exit(session);
}

void print_banner(const Options& options, const SocketCanTransport& transport) {
    std::cout << "=== YZ-AIM CANopen 板载 CAN 直连维护工具 " << kVersion << " ===\n";
    std::cout << "接口 " << transport.interface_name() << "：已 UP";
    if (transport.reported_bitrate())
        std::cout << "，内核报告波特率 " << transport.reported_bitrate() << " bit/s";
    else
        std::cout << "，内核未报告波特率（可能由设备树固定）";
    std::cout << "\n节点 " << options.node << "   期望波特率 " << options.bitrate << " bit/s\n";
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

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, options)) return 2;
    if (options.help) { print_usage(); return 0; }
    if (options.version) { std::cout << kProgram << " " << kVersion << "\n"; return 0; }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const auto node = static_cast<uint8_t>(options.node);
    SocketCanTransport transport(options.interface_name, node);
    std::string error;
    if (!transport.open(error)) {
        std::cerr << "[失败] 打开 " << options.interface_name << "：" << error << "\n";
        return 3;
    }
    print_banner(options, transport);

    Motor motor(transport, node);
    g_motor = &motor;

    Session session{motor, transport, node, false,
                    static_cast<uint32_t>(yz_aim::kPositionProfileRpm), options.timeout_ms, options.assume_yes};
    const int code = options.command.empty() ? run_session(session) : run_single_command(session, options.command);
    g_motor = nullptr;
    return code;
}
