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
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
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
constexpr const char* kVersion = "1.1.0-TUI";
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
// 全屏界面开关：由 --plain / --tui 设置，默认按终端能力自动决定。
bool g_tui_disabled = false;
bool g_tui_forced = false;

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
            return "检查：can0 波特率与电机一致、CANH/CANL 未接反、总线末端 120Ω 终端电阻、节点号正确；\n"
                   "        另外先看 ip -details link show can0 的 can state —— 若是 ERROR-PASSIVE/BUS-OFF，\n"
                   "        控制器收发已不可信（常见于之前发过帧没被应答），跑 sh tools/can0_up.sh can0 1000000 复位后再试。";
        case Error::transport:
            return "检查接线与 ip -details -statistics link show can0 的收发计数、bus-off 状态。";
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
    bool assume_yes = false;
    bool plain_ui = false;
    bool force_tui = false;
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
        "  --node2 N       第二轴节点 0~126（默认 2 = 双轴；0 = 只用单轴）\n"
        "  --bitrate BPS   期望波特率 125000/250000/500000/1000000（默认 1000000），\n"
        "                  仅用于与内核报告的波特率比对；can0 的波特率由 ip 命令或设备树决定\n"
        "  --timeout MS    单次动作总预算（默认 20000；单条 SDO 仍最多等 120 ms）\n"
        "  --yes           允许 set-zero / set-node / set-bitrate 这类写内部参数的操作\n"
        "  --tui           强制启用全屏界面（终端能力探测不到时也可用）\n"
        "  --plain         关闭全屏界面，回到原来的逐行命令行（管道/脚本时自动如此）\n"
        "  --version | --help\n"
        "\n"
        "命令：\n"
        "  status                             只读：读取一次完整快照\n"
        "  monitor [毫秒] [间隔ms]            只读：周期刷新（默认 10000 ms / 500 ms）\n"
        "  read-od <索引>[:<子索引>] [宽度]   只读：读任意对象字典（宽度 1/2/4，缺省自动试探）\n"
        "  enable                             安全检查后使能输出（之后本进程维持心跳）\n"
        "  disable                            停止并失能输出\n"
        "  stop                               急停保持 0x6040=0x010F（输出仍使能）\n"
        "  rel <输出角度>                     相对位置运动（度，可小数、可负；基准是动作前重读的 0x6064）\n"
        "  abs <输出角度>                     绝对位置运动（输出轴角度坐标，度）\n"
        "  pulses <电机脉冲>                  绝对位置运动（0x6064 原始坐标）\n"
        "  jog <rpm> [毫秒]                   速度模式点动（默认 500 ms，最长 5000 ms）；两轴已使能时\n"
        "                                     自动变成双轴镜像点动（见下）\n"
        "  spin <rpm> [毫秒]                  速度模式连续运转（默认 60000 ms，最长 600000 ms；Ctrl-C 随时停）\n"
        "  run <rpm>                          速度模式常转：启动后立即回到提示符，一直转到 stop 为止\n"
        "  speed <rpm>                        选择本会话的运动速度 1~3000（只改本会话，不写设备）\n"
        "\n"
        "双轴镜像同步（本程序默认 --node2 2，两轴必须已在设备里改成不同节点号）：\n"
        "  两轴是镜像安装（转向相反），所以每个动作都是一轴 +、一轴 −，不存在同向配置。\n"
        "  extend <rpm> <毫秒>                刷架伸出：取正的节点 +rpm、取负的节点 −rpm\n"
        "  retract <rpm> <毫秒>               刷架缩进：方向与 extend 相反\n"
        "  jog <rpm> [毫秒]                   双轴镜像点动（默认 500 ms，上限 5000 ms；正 rpm = 伸出方向）\n"
        "  sync-dir <正转节点> <反转节点>     指定哪个节点取 +（默认本会话的 --node2 取 +、--node 取 −）；\n"
        "                                     两个参数必须是本会话的 A、B 节点各一个，否则整条命令不生效\n"
        "  sync-enable                        两轴依次做安全检查并使能输出\n"
        "  sync-disable                       两轴同时停止并失能输出\n"
        "  sync-stop                          两轴同时急停并确认（输出仍使能）\n"
        "  sync-status                        只读：打印两轴完整快照、镜像方向与一致性指标\n"
        "  sync-zero                          只读：把此刻两轴位置记成 0 行程基准（会话级，只影响显示）\n"
        "  sync-tol <输出角度>                位置偏差阈值（默认 0.5°，0 = 关闭偏差监控）\n"
        "  sync-drift <电机RPM>               速度模式每拍允许的两轴速度差（默认 10）。偏差在阈值内时\n"
        "                                     基准每拍重取，所以门限不随总运行时间变松\n"
        "  set-lead <mm/圈>                   丝杆导程（输出轴转一整圈走的毫米数），供 sync-mm 换算；\n"
        "                                     0 = 清除。只在本会话有效，不写设备\n"
        "  sync-rel <输出角度> [rpm]          两轴各走同样大小的行程（镜像反向；rpm 默认同 speed）\n"
        "  sync-abs <输出角度> [rpm]          两轴走到同一绝对坐标（以 A 轴坐标为基准）\n"
        "  sync-mm <毫米> [rpm]               按毫米走行程：用 set-lead 的导程把毫米换算成输出角度，\n"
        "                                     再走与 sync-rel 相同的位置模式（闭环到位；未设导程时拒绝）\n"
        "  sync-spin <rpm> [毫秒]             两轴镜像同步速度运转（默认 60000 ms，上限 600000 ms）\n"
        "  sync-run <rpm>                     两轴镜像同步常转：一直转到 sync-stop 为止\n"
        "  （每个双轴动作结束、两轴都停下后会自动打印一段\"本次动作汇总\"：两轴各自行程、本次\n"
        "    两端错位量、折算等效速度差；单位同时给 电机脉冲 / 输出角度° / 毫米，其中毫米只在\n"
        "    set-lead 设过导程时才出现。sync-run 的汇总在 sync-stop / sync-disable 时打印）\n"
        "\n"
        "单轴单动（只让一台电机动，另一轴不动；刚性刷架下会别歪，仅用于对位/测试）：\n"
        "  one <节点号> <rpm> <毫秒>          只让指定节点单独运动（rpm ±1~±300，毫秒 1~600000）\n"
        "  one-stop                           两轴同时急停并确认（单动/常转的收尾）\n"
        "\n"
        "内部参数（需输出失能且静止）：\n"
        "  set-profile-speed <rpm>            写 0x6081 位置轮廓速度 0~3000\n"
        "  set-accel <rpm/s>                  写 0x6083 加速度 10~1000\n"
        "  set-current <mA>                   写 0x261D 最大电流 10~10000，10 的整数倍\n"
        "  set-zero                           置当前轴位为零点（需 --yes；手册 §5.2/§6.2 的 60006→60016）\n"
        "  set-node <1~127>                   暂存新节点号并保存（需 --yes；断电重新上电后生效）\n"
        "  set-bitrate <125|250|500|1000>     暂存 CAN 波特率（需 --yes；断电重新上电后生效）\n"
        "\n"
        "全屏界面（TUI，交互模式且标准输入输出都是终端时自动启用）：\n"
        "  画面上半部常驻显示两轴状态与一致性指标，中间是本次会话的输出，底部是输入行；\n"
        "  命令、参数、判据与逐行模式完全一样，只是打字的地方固定在最下面一行。\n"
        "  tui-refresh <毫秒>                 面板自动刷新间隔 200~60000（默认 1000；0 = 关闭）\n"
        "                                     面板只在空闲、本会话未使能输出、can0 处于 ERROR-ACTIVE，\n"
        "                                     并且已经成功读到过一次两轴时才自动刷新；第一次读按 Ctrl-R。\n"
        "                                     （空总线上第一帧没人应答就会把 can0 推进 ERROR-PASSIVE，\n"
        "                                     所以第一帧必须由你按 Ctrl-R 明确要求。）\n"
        "  tui-size <行> <列>                 固定界面尺寸（默认按终端报告；串口终端报不出尺寸时用，\n"
        "                                     两个 0 表示恢复自动探测；也可先 export COLUMNS= / LINES=）\n"
        "  键盘：Tab 补全命令，↑↓ 翻输入历史，Ctrl-A/Ctrl-E 行首行尾，Ctrl-U 清空输入行，\n"
        "        Ctrl-P/Ctrl-N 逐行翻日志、PageUp/PageDown 整页翻，Ctrl-L 清日志，\n"
        "        Ctrl-R 立即读一次面板，Ctrl-D 在空行时退出，\n"
        "        Ctrl-C 与逐行模式一致（常转期间等同 stop）。\n"
        "  help | quit / exit\n"
        "\n"
        "时间参数一律用毫秒（ms）。\n"
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
        "     常转没有时长上限，请务必留人看管，并注意 0x6064 不要逼近 ±" << yz_aim::kMaxPositionPulses << "。\n"
        "\n"
        "双轴镜像同步说明：\n"
        "  1) 本程序默认 --node2 2，用一条 can0 同时管两个节点；每轴各开一条套接字，\n"
        "     接收过滤器只放行本节点的 SDO 应答/TPDO/心跳（与单轴版一致），所有 SDO 严格\n"
        "     串行，不引入后台线程。只用一台电机时加 --node2 0。\n"
        "  2) 本驱动器没有 CANopen 从动/跟随模式，做不到微秒级硬件同步。同步点是\n"
        "     \"两轴各自完成全部设定点写入后，连续发出两条触发帧\"，间隔只有一个 SDO 往返（约\n"
        "     1~5 ms）。30 电机RPM 时对应输出轴约 0.018°，300 电机RPM 时约 0.18°。\n"
        "  3) 两轴镜像安装，转向必然相反，所以每个动作都是一轴 +、一轴 −，没有\"两轴同向\"\n"
        "     这种配置。默认是 --node2 取 +、--node 取 −：--node 1 --node2 2 时，\"extend 300 10000\"\n"
        "     就是节点 2 +300、节点 1 −300。sync-dir 的两个参数按 <取 + 的节点> <取 − 的节点> 给，\n"
        "     要调换正负就执行 sync-dir 1 2（--node 1 --node2 2 时）。\n"
        "  4) 偏差监控看的是 posA+posB 之和（一致性指标），不是差 —— 镜像安装时刷架正常\n"
        "     移动本来就会让一个位置增大、另一个减小，用差会把正常运动当成两倍行程的超差。\n"
        "     两轴的机械零点差（编码器零点偏移、或装配上本来就歪着）表现为这个指标上的一个常数，\n"
        "     判据只看它的变化量，所以既不会导致失步、也不会被误判。想看它就用 sync-zero 把此刻\n"
        "     记成 0 行程基准，之后 sync-status 与每个动作的收尾汇总都会给出\"两端错位\"相对基准变了\n"
        "     多少。sync-zero 只记录、不写设备、不改运动；要真正改机械零点得用 set-zero\n"
        "     （逐轴、写设备、掉电保持、需 --yes；双轴使能期间会被拒绝）。\n"
        "  5) 位置模式下两轴走同样大小的行程：sync-abs 以 A 轴坐标为基准算增量，\n"
        "     不能让两轴各走各的绝对坐标，否则刚性刷架会被别住。\n"
        "  6) 运行中每拍读两轴状态字/报警/转速/位置，位置偏差连续 " << kSyncTripSamples << " 拍超过门限即判定异常：\n"
        "     对两轴各发一帧未确认急停，再交替复查确认。位置模式的门限就是 sync-tol，一轴先到位后\n"
        "     放宽 4 倍并给 " << (kSyncFinalGraceMs / 1000) << " s 宽限。速度模式里两轴各走自己的速度环，位置之和\n"
        "     会以恒定速率缓慢漂移；基准每拍重取一次（跟着当前位置走），所以偏差衡量的是\"相邻两拍\n"
        "     之间的变化量\"：匀速漂移被完全吸收、门限锁在 sync-tol 附近不随运行时间变松，一次性\n"
        "     错位只算一拍，只有持续跑飞才会连续超限。门限 = sync-tol + sync-drift × 两拍间隔秒数，\n"
        "     采样间隔变长会自动放宽一点。跳闸时会把本拍偏差、已运行时长、自起点累计偏差折算的\n"
        "     等效速度差与两轴实时转速一起打出来。动作正常收尾时也会先打印一段收尾汇总，\n"
        "     不用等跳闸。\n"
        "  7) 双轴同步速度硬上限 " << yz_aim::kSyncMaxRpm << " 电机RPM（include/yz_aim.h 的 kSyncMaxRpm）。\n"
        "  8) 双轴使能期间 rel/abs/pulses/spin/run 一律拒绝（只动一轴会把刷架别歪）；jog 不拒绝，\n"
        "     而是自动变成双轴镜像点动（上限 5000 ms）。确实需要单动时用 one / one-stop ——\n"
        "     它只动一个节点、另一轴保持不动，速度上限同样是 ±300。\n"
        "  9) sync-disable / quit 会对两轴各自确认失能；未能确认时会明确提示按急停/断电处置。\n"
        " 10) 位置模式（sync-rel / sync-abs / sync-mm）单位是输出轴角度，闭环到位，重复精度远高于\n"
        "     按时间跑的速度模式（extend / retract：走多远由时间决定，负载一变就变）。\n"
        "     程序不知道机械导程，所以没有毫米单位：先用 set-lead <mm/圈> 告知导程，\n"
        "     之后就能用 sync-mm <毫米> 直接给行程。测导程：先 sync-rel 3600 100（输出轴整 10 圈），\n"
        "     量出刷架实际位移 L 毫米，导程 = L ÷ 10。\n"
        " 11) 偏差怎么读：一致性指标 posA+posB 正比于刷架两端的高度差，所以它的漂移量（电机脉冲）\n"
        "     就等于\"两端错位了多少\"，系数 1:1 —— 换成输出角度 = 脉冲 × 360 ÷ 1671168，\n"
        "     换成毫米 = 脉冲 ÷ 1671168 × 导程。位置模式看到位偏差（对 sync-tol），\n"
        "     速度模式看折算等效速度差（对 sync-drift），判据不同，见各自动作的收尾汇总。\n"
        "\n"
        "全屏界面（TUI）说明：\n"
        "  1) 只在交互模式、且标准输入输出都是真终端时启用；管道、重定向、单命令模式一律走\n"
        "     原来的逐行界面，所以脚本与 \"printf 'help\\nquit\\n' | ./build/yz_aim_can0\" 不受影响。\n"
        "     用 --plain 可以随时强制回到逐行界面，用 --tui 可以强制启用。\n"
        "  2) 全屏层只负责画屏和收键盘：命令解析、参数校验、偏差判据、收尾汇总、安全锁止\n"
        "     与逐行模式完全相同，没有任何一条命令被删改。日志区里看到的就是原来的输出。\n"
        "  3) 面板自动读两轴读数，但只在\"两轴都没有运动、本会话未使能输出、can0 处于\n"
        "     ERROR-ACTIVE，并且已经成功读到过一次\"时才读，间隔由 tui-refresh 控制（默认 1000 ms）。\n"
        "     面板不会主动发出本会话的第一帧：空总线上第一帧没人应答就会把 can0 推进\n"
        "     ERROR-PASSIVE，所以这第一帧要你按 Ctrl-R 明确要求（等价于敲 sync-status）。\n"
        "     读到过一次之后就一直自动刷新；使能/运动期间不自动读，按 Ctrl-R 手动读。\n"
        "     连续两次读不到就停止自动发帧并清空面板旧读数（显示 --），避免空总线上反复发帧\n"
        "     把 can0 拖在 ERROR-PASSIVE，也避免把过期的读数当现值看。\n"
        "  4) 标题栏的 CAN 状态每 2 s 用只读 netlink 查询刷新一次（不发任何 CAN 帧），\n"
        "     会话中途掉进 ERROR-PASSIVE / BUS-OFF 时标题栏和日志提示都会跟上。\n"
        "  5) 退出（quit / Ctrl-D / Ctrl-C）会先恢复终端再打印本次会话的输出；被 kill -9\n"
        "     除外 —— 那种情况下终端可能停在无回显状态，敲 reset 或 stty sane 即可恢复。\n";
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
        if (argument == "--plain") { options.plain_ui = true; continue; }
        if (argument == "--tui") { options.force_tui = true; continue; }
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
    Session(Motor& motor_ref, SocketCanTransport& transport_ref, uint8_t node_id,
            uint64_t budget_ms, bool yes)
        : motor(motor_ref), transport(transport_ref), node(node_id),
          timeout_ms(budget_ms), assume_yes(yes) {}
    Motor& motor;
    SocketCanTransport& transport;
    // B 轴自己的传输层（单轴模式为空）。两轴各用一条套接字，接收过滤器只放行本节点报文。
    SocketCanTransport* transport2 = nullptr;
    uint8_t node = kDefaultNode;
    bool enabled = false;
    uint32_t motion_rpm = static_cast<uint32_t>(yz_aim::kPositionProfileRpm);
    uint64_t timeout_ms = kDefaultTimeoutMs;
    bool assume_yes = false;
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
    int64_t sync_base_pulses = 0;         // 判定用的基准指标；速度模式每拍重取，吸收缓慢漂移
    int64_t sync_origin_pulses = 0;       // 运动起点的指标，只用于显示累计偏差
    Clock::time_point sync_base_at{};     // 上面那个基准的取样时刻，门限只按它算"已跑多久"
    // 速度模式：门限里"允许的两轴速度差"（电机 RPM），乘自上次重取基准以来的时长；0 = 不放宽。
    double sync_drift_rpm = yz_aim::kSyncDefaultDriftRpm;
    Clock::time_point sync_move_started{}; // 本次双轴动作的启动时刻（只用于打印已运行时长）
    // 丝杆导程（mm/输出圈），由 set-lead 设置；0 = 未设置，sync-mm 拒绝执行。
    double lead_mm = yz_aim::kSyncDefaultLeadMm;
    // sync-zero 记录的行程基准（会话级）：两轴在那一刻的原始 0x6064，只用于显示，不参与判据。
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
    print_snapshot(snapshot, session.node, session.transport.interface_name());
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

// 速度模式的偏差门限：基础阈值 + 允许的两轴速度差 × 自上次重取基准以来的时长。
// 两轴各走自己的速度环，稳态下必有微小速度差，位置之和会以恒定速率漂移；每拍重取基准
// （sync_rebase）就把这种匀速漂移吸收掉，所以门限不会随总运行时间无限放宽 —— 这里乘的
// "时长"是自上次成功重取基准以来的时间，正常只有一拍。真跑飞时单拍偏差就爆表，几拍内超限。
// 返回 0 表示关掉偏差监控。
int64_t sync_speed_limit(const Session& session, int64_t since_rebase_ms) {
    const double base = sync_tolerance_pulses(session);
    if (base <= 0) return 0;
    const double drift = session.sync_drift_rpm / 60.0 *
                         static_cast<double>(yz_aim::kMotorPulsesPerRevolution);
    const double seconds = static_cast<double>(since_rebase_ms) / 1000.0;
    const double limit = base + drift * seconds;
    return limit > 2.0e9 ? 2000000000LL : static_cast<int64_t>(limit);
}

// 偏差等效的两轴速度差（电机 RPM）= 偏差 ÷ 已运行秒数 ÷ 每圈脉冲 × 60。跳闸时打印用。
double sync_deviation_rpm(int64_t deviation, int64_t elapsed_ms) {
    const double seconds = static_cast<double>(elapsed_ms) / 1000.0;
    if (seconds < 0.05) return 0;
    return static_cast<double>(deviation) / seconds * 60.0 /
           static_cast<double>(yz_aim::kMotorPulsesPerRevolution);
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

// 两轴位置按采样时刻对齐后的一致性指标；不对齐的话，采样时差会被算成偏差。
int64_t sync_aligned_metric(const Snapshot& a, const Snapshot& b) {
    int64_t pa = 0, pb = 0;
    sync_align_positions(a, b, pa, pb);
    return sync_pair_metric(pa, pb);
}

// 自判定基准取样以来经过的毫秒；没设过基准时返回 0。速度模式的门限只按这个时长放宽。
int64_t sync_base_age_ms(const Session& session, Clock::time_point now) {
    if (session.sync_base_at.time_since_epoch().count() == 0) return 0;
    const int64_t age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - session.sync_base_at).count();
    return age < 0 ? 0 : age;
}

// 每拍重取判定基准（跟着当前位置走）。速度模式里两轴各走自己的速度环，位置之和会以恒定
// 速率缓慢漂移；基准每拍跟着走，偏差衡量的就变成"相邻两拍之间的位置变化量"：匀速漂移被
// 完全吸收（每拍变化很小），一次性错位只影响一拍，只有持续跑飞才会让连续几拍的变化量都
// 超限。这样门限也就锁在 sync-tol 附近，不随总运行时间变松。
void sync_rebase(Session& session, int64_t metric, Clock::time_point now) {
    session.sync_base_pulses = metric;
    session.sync_base_at = now;
}

// 一次双轴动作结束（两轴都已停止）后的收尾汇总 —— 现场"每次动作后先看一眼偏差"看的就是这一块。
// 两端错位量 = 一致性指标 posA+posB 相对本次动作起点的漂移（位置模式相对目标坐标），它正比于
// 刷架两端的高度差，系数是 1:1；单位同时给电机脉冲（判据原生）、输出角度（位置模式命令的单位）、
// 毫米（只在 set-lead 设过导程时才给）。
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
    std::cout << "[本次动作汇总] " << name << "，实际运行 " << elapsed_ms << " ms\n"
              << "    行程 A " << travel_text(session.lead_mm, travel_a) << "   B "
              << travel_text(session.lead_mm, travel_b)
              << "   ← 镜像正常时两者符号相反、大小接近\n";
    if (velocity_mode) {
        const int64_t offset = metric - session.sync_origin_pulses;
        const int64_t magnitude = sync_abs_diff(metric, session.sync_origin_pulses);
        std::cout << "    本次两端错位（一致性指标相对动作起点）"
                  << travel_text(session.lead_mm, offset) << "\n";
        zero_line();
        if (base_tolerance <= 0) {
            std::cout << "    偏差监控已关闭（sync-tol 0）：上面的数只是记录，不会触发急停。\n";
            return;
        }
        // 速度模式两轴各走自己的速度环，稳态下必然有微小速度差、位置之和匀速漂移，所以不看
        // 累计量本身，而是把它折算成等效速度差，与允许的两轴速度差 sync-drift 比。
        const double equivalent_rpm = sync_deviation_rpm(magnitude, elapsed_ms);
        std::cout << "    折算等效速度差 " << fixed(equivalent_rpm, 2)
                  << " 电机RPM（门限里的让步项 sync-drift = " << fixed(session.sync_drift_rpm, 2)
                  << " 电机RPM）\n";
        if (equivalent_rpm <= session.sync_drift_rpm + 0.05) {
            std::cout << "    结论：两轴稳态速度差在 sync-drift 内，本次同步正常。\n";
        } else {
            std::cout << "    结论：两轴稳态速度差比 sync-drift 大 —— 这种匀速差不会跳闸（门限每拍跟着\n"
                         "          当前位置走，只抓持续性跑飞），但两端会以约 "
                      << fixed(equivalent_rpm / 60.0 *
                                   static_cast<double>(yz_aim::kMotorPulsesPerRevolution), 0)
                      << " 脉冲/秒 持续错开，\n"
                         "          走久了要留意；可降速、查两轴负载，或调 sync-drift。\n";
        }
        if (magnitude > static_cast<int64_t>(base_tolerance)) {
            std::cout << "    提示：速度模式按时间开环计时，累计错位随时间线性增长；要可重复的行程\n"
                         "          精度请用位置模式 sync-rel / sync-mm（闭环到位，不累积）。\n";
        }
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
void sync_halt_both(Session& session) {
    (void)session.motor.halt_unconfirmed(after_ms(2000));
    if (session.second) (void)session.second->halt_unconfirmed(after_ms(2000));
    const auto until = Clock::now() + std::chrono::milliseconds(8000);
    bool stopped = false;
    while (Clock::now() < until) {
        Snapshot a{}, b{};
        const bool ok_a = static_cast<bool>(session.motor.poll_safety(a, after_ms(1500)));
        const bool ok_b = !session.second || static_cast<bool>(session.second->poll_safety(b, after_ms(1500)));
        if (ok_a && ok_b && std::abs(a.speed_rpm) <= 0.5 &&
            (!session.second || std::abs(b.speed_rpm) <= 0.5)) {
            stopped = true;
            break;
        }
        std::this_thread::sleep_until(std::min(until, Clock::now() + std::chrono::milliseconds(70)));
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    // 两轴的 stop() 各自只服务自己的心跳，而 verify_stopped 最长可等 7 秒；串行执行时
    // 后一轴的心跳会超过 700 ms 被判 overdue，在 Motor::write → heartbeat 里 fail(unsafe)
    // 把那一轴永久锁死。所以每次 stop 前后都补一手对侧心跳。
    (void)session.motor.service_watchdog(after_ms(1000));
    if (session.second) (void)session.second->service_watchdog(after_ms(1000));
    const Result ra = session.motor.stop(after_ms(budget));
    (void)session.motor.service_watchdog(after_ms(1000));
    if (session.second) (void)session.second->service_watchdog(after_ms(1000));
    const Result rb = session.second ? session.second->stop(after_ms(budget)) : Result{};
    if (stopped && ra && rb) {
        std::cout << "    两轴已确认停止并保持。\n";
        return;
    }
    std::cout << "    [!] 未能确认两轴都已停止；请按现场急停/断电处置，不要依赖本工具。\n";
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
    std::cout << "\n[!] 双轴同步异常：" << reason << "\n";
    if (a) std::cout << "    A 轴 " << one_line_snapshot(*a) << "\n";
    if (b) std::cout << "    B 轴 " << one_line_snapshot(*b) << "\n";
    std::cout << "    正在对两轴各发一帧未确认急停…\n";
    sync_halt_both(session);
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
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 30000);
    const Result ra = session.motor.disable_output(after_ms(budget));
    const Result rb = session.second->disable_output(after_ms(budget));
    session.dual_enabled = false;
    session.enabled = false;
    if (report(ra, "A 轴停止并失能") && report(rb, "B 轴停止并失能")) {
        sync_report_move(session, "sync-disable 收尾", true);
        return Disposition::keep_going;
    }
    std::cout << "[!] 未能确认两轴都已失能；心跳已停止，驱动约 1 s 后应报 0x20 并停机，\n"
                 "    但现场仍须按急停/断电处置。\n";
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
    std::cout << "两轴同时急停并确认…\n";
    sync_halt_both(session);
    sync_report_move(session, "两轴急停收尾", true);
    return Disposition::keep_going;
}

Disposition command_sync_status(Session& session) {
    if (!sync_has_second(session, "sync-status")) return Disposition::keep_going;
    const Snapshot a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot b = session.second->snapshot(after_ms(session.timeout_ms));
    std::cout << "--- A 轴（节点 " << static_cast<unsigned>(session.node) << "）---\n";
    print_snapshot(a, session.node, session.transport.interface_name());
    std::cout << "--- B 轴（节点 " << static_cast<unsigned>(session.node2) << "）---\n";
    print_snapshot(b, session.node2, session.transport.interface_name());
    // 镜像方向与阈值不依赖电机应答，先打出来，接不上电机时也能核对极性。
    std::cout << "镜像方向 " << sync_dir_text(session) << "   一致性指标 = posA + posB\n"
              << "同步偏差阈值 " << fixed(session.sync_tolerance_deg, 4) << "°"
              << (session.sync_tolerance_deg <= 0 ? "（监控已关闭）" : "") << "\n"
              << "速度模式每拍允许的两轴速度差 " << fixed(session.sync_drift_rpm, 2)
              << " 电机RPM（sync-drift；0 = 门限退化为固定的 sync-tol）\n"
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

// sync-zero：把此刻两轴位置记为"行程基准"（会话级）。只记录与显示，不写设备，也不改运动行为。
// 两轴机械零点差是安装的固有量、不参与同步判据（判据用位置之和），所以这里不做补偿，
// 只给现场一个"从这一刻算起了多少行程"的参考。
Disposition command_sync_zero(Session& session) {
    if (!sync_has_second(session, "sync-zero")) return Disposition::keep_going;
    const Snapshot a = session.motor.snapshot(after_ms(session.timeout_ms));
    const Snapshot b = session.second->snapshot(after_ms(session.timeout_ms));
    if (!a.result) { report(a.result, "读 A 轴位置"); return Disposition::keep_going; }
    if (!b.result) { report(b.result, "读 B 轴位置"); return Disposition::keep_going; }
    session.sync_zero_valid = true;
    session.sync_zero_a = a.position_pulses;
    session.sync_zero_b = b.position_pulses;
    std::cout << "行程基准已记录（" << timestamp_text() << "）：\n"
              << "    A 轴（节点 " << static_cast<unsigned>(session.node) << "）" << a.position_pulses
              << " 脉冲（" << fixed(a.output_position_degrees, 4) << "°）\n"
              << "    B 轴（节点 " << static_cast<unsigned>(session.node2) << "）" << b.position_pulses
              << " 脉冲（" << fixed(b.output_position_degrees, 4) << "°）\n"
              << "    一致性指标 posA + posB = "
              << (static_cast<int64_t>(a.position_pulses) + b.position_pulses)
              << " 电机脉冲，它正比于此刻两端的高度差；后面所有显示都以它为 0 基准。\n"
              << "    机械零点差（编码器零点偏移、或此刻姿态本来就歪着）都表现为这个指标上的一个\n"
              << "    常数，而判据只看它的变化量，所以既不会被误判、也不需要做补偿。\n"
              << "    只在本会话有效，quit 后失效；sync-status 和每个动作的收尾汇总都会给出相对本基准错开了多少。\n";
    return Disposition::keep_going;
}

Disposition command_sync_tol(Session& session, const std::string& text) {
    if (!sync_has_second(session, "sync-tol")) return Disposition::keep_going;
    double degrees = 0;
    if (!parse_double(text, degrees) || degrees < 0 || degrees > 3600) {
        std::cout << "用法：sync-tol <输出角度，度>（0 表示关闭偏差监控）\n";
        return Disposition::keep_going;
    }
    session.sync_tolerance_deg = degrees;
    std::cout << "同步偏差阈值 = " << fixed(degrees, 4) << "°"
              << (degrees <= 0 ? "（已关闭偏差监控）" : "") << "；约 "
              << static_cast<int64_t>(sync_tolerance_pulses(session)) << " 电机脉冲。\n";
    return Disposition::keep_going;
}

// sync-drift <电机RPM>：速度模式下允许的两轴稳态速度差，门限按它随时间线性放宽。
// 两轴各走自己的速度环必然有速度差，现场用这个值把"正常漂移"和"真跑飞"分开。
Disposition command_sync_drift(Session& session, const std::string& text) {
    if (!sync_has_second(session, "sync-drift")) return Disposition::keep_going;
    double rpm = 0;
    if (!parse_double(text, rpm) || rpm < 0 || rpm > yz_aim::kSyncMaxRpm) {
        std::cout << "用法：sync-drift <电机RPM 0~" << yz_aim::kSyncMaxRpm
                  << ">（0 表示门限不随时间放宽）\n";
        return Disposition::keep_going;
    }
    session.sync_drift_rpm = rpm;
    const double per_second = rpm / 60.0 * static_cast<double>(yz_aim::kMotorPulsesPerRevolution);
    std::cout << "速度模式每拍允许的两轴速度差 = " << fixed(rpm, 2) << " 电机RPM"
              << (rpm <= 0 ? "（门限退化为固定的 sync-tol）" : "")
              << "；门限按两拍间隔放宽 " << static_cast<int64_t>(per_second) << " 电机脉冲/秒。\n";
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

// 双轴同步速度运动：预置两轴速度模式后连发两条启动帧，定时或常转。
// label 只用于打印（extend / retract / jog 都复用这条路径，汇总里要显示自己的命令名）。
Disposition command_sync_velocity(Session& session, const std::vector<std::string>& arguments,
                                  bool continuous, const char* label = nullptr) {
    const std::string name = label != nullptr ? label : (continuous ? "sync-run" : "sync-spin");
    if (!sync_has_second(session, name.c_str())) return Disposition::keep_going;
    if (!sync_needs_enabled(session, name.c_str())) return Disposition::keep_going;
    if (session.sync_run_active) {
        std::cout << "[失败] " << name << "：双轴常转进行中，请先 sync-stop。\n";
        return Disposition::keep_going;
    }
    if (arguments.empty()) {
        std::cout << "用法：" << name << " <rpm±1~±" << yz_aim::kSyncMaxRpm << ">"
                  << (continuous ? "（一直转到 sync-stop）\n" : " [毫秒 1~600000，默认 60000]\n");
        return Disposition::keep_going;
    }
    int32_t rpm = 0;
    if (!parse_i32(arguments[0], -yz_aim::kSyncMaxRpm, yz_aim::kSyncMaxRpm, rpm) || rpm == 0) {
        std::cout << "转速需为 ±1~±" << yz_aim::kSyncMaxRpm << " 电机RPM（不能为 0）\n";
        return Disposition::keep_going;
    }
    uint32_t duration_ms = 60000;
    if (!continuous && arguments.size() >= 2 && !parse_u32(arguments[1], 1, 600000, duration_ms)) {
        std::cout << "时长需为 1~600000 毫秒\n";
        return Disposition::keep_going;
    }
    if (!sync_arm_both(session)) return Disposition::keep_going;
    // 速度模式的偏差基准：启动前的两轴位置指标。两轴必须静止（速度 0），采样时差不会带来偏移。
    const Snapshot base_a = session.motor.snapshot(after_ms(session.timeout_ms));
    if (!base_a.result) { report(base_a.result, "读取 A 轴启动基准位置"); return Disposition::keep_going; }
    const Snapshot base_b = session.second->snapshot(after_ms(session.timeout_ms));
    if (!base_b.result) { report(base_b.result, "读取 B 轴启动基准位置"); return Disposition::keep_going; }
    const int64_t start_metric = sync_pair_metric(base_a.position_pulses, base_b.position_pulses);
    session.sync_base_pulses = start_metric;   // 判定基准：运行中每拍重取，吸收匀速漂移
    session.sync_origin_pulses = start_metric; // 运动起点：只用于打印累计偏差
    session.sync_base_at = Clock::now();
    // 记下本次动作起点的两轴位置，供收尾汇总算各轴行程（只用于显示）。
    session.sync_start_valid = true;
    session.sync_start_a = base_a.position_pulses;
    session.sync_start_b = base_b.position_pulses;
    // 镜像：一轴 +、一轴 −，谁取正由 sync-dir 决定（+rpm 即"伸出方向"）。
    const int32_t rpm_a = sync_sign_a(session) * rpm;
    const int32_t rpm_b = -rpm_a;
    const auto travel = continuous ? std::chrono::seconds(60)
                                   : std::chrono::milliseconds(duration_ms);
    if (!report(session.motor.stage_velocity(rpm_a, travel, after_ms(session.timeout_ms)),
                "A 轴预置速度模式"))
        return Disposition::keep_going;
    if (!report(session.second->stage_velocity(rpm_b, travel, after_ms(session.timeout_ms)),
                "B 轴预置速度模式")) {
        report(session.motor.abort_staged(after_ms(session.timeout_ms)), "放弃 A 轴预置");
        return Disposition::keep_going;
    }
    std::cout << "A 轴 " << rpm_a << " / B 轴 " << rpm_b << " 电机RPM（"
              << sync_dir_text(session) << "，A 轴 " << fixed(yz_aim::motor_to_output_rpm(rpm_a), 3)
              << " 输出RPM）\n"
              << (continuous ? "一直转到 sync-stop" : "运行 " + std::to_string(duration_ms) + " 毫秒")
              << "；正在同步触发…\n";
    const Deadline trigger_deadline = after_ms(session.timeout_ms);
    const Result trigger_a = session.motor.trigger_velocity(trigger_deadline);
    const Result trigger_b = session.second->trigger_velocity(trigger_deadline);
    if (!trigger_a || !trigger_b) {
        sync_trip(session, trigger_a ? "B 轴启动失败：" + sync_reason_text(trigger_b)
                                     : "A 轴启动失败：" + sync_reason_text(trigger_a));
        return Disposition::keep_going;
    }
    if (continuous) {
        session.sync_run_active = true;
        session.sync_run_rpm = rpm;
        session.sync_run_over = 0;
        session.sync_run_started = Clock::now();
        session.sync_run_next_tick = Clock::now();
        session.sync_run_next_line = Clock::now();
        session.sync_move_started = session.sync_run_started; // 收尾汇总用同一个起点
        std::cout << "已启动双轴常转。提示符变为 yz(双轴常转)>，输入 sync-stop 停止。\n";
        return Disposition::keep_going;
    }
    const Deadline until = after_ms(static_cast<uint64_t>(duration_ms));
    session.sync_move_started = Clock::now();
    int over = 0;
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
        const auto tick = Clock::now();
        const int64_t metric = sync_aligned_metric(a, b);
        const int64_t deviation = sync_abs_diff(metric, session.sync_base_pulses);
        const int64_t cumulative = sync_abs_diff(metric, session.sync_origin_pulses);
        const int64_t limit = sync_speed_limit(session, sync_base_age_ms(session, tick));
        if (limit > 0 && deviation > limit) {
            if (++over >= kSyncTripSamples) {
                const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    tick - session.sync_move_started).count();
                std::cout << "\n[!] 两轴本拍偏差 " << deviation << " 电机脉冲（"
                          << fixed(yz_aim::pulses_to_output_degrees(static_cast<int32_t>(deviation)), 4)
                          << "°）连续 " << over << " 拍超过门限 " << limit << " 电机脉冲\n"
                          << "    已运行 " << elapsed_ms << " ms；基础阈值 "
                          << fixed(session.sync_tolerance_deg, 4) << "°，允许速度差 "
                          << fixed(session.sync_drift_rpm, 2) << " 电机RPM\n"
                          << "    自起点累计偏差 " << cumulative << " 电机脉冲，折算等效速度差 "
                          << fixed(sync_deviation_rpm(cumulative, elapsed_ms), 2)
                          << " 电机RPM；两轴实时转速 " << fixed(a.speed_rpm, 1) << " / "
                          << fixed(b.speed_rpm, 1) << " 电机RPM\n";
                sync_trip(session, "速度模式位置偏差超限", &a, &b);
                return Disposition::keep_going;
            }
        } else {
            over = 0;
        }
        // 无论当拍是否超限都重取基准：一次性错位只算一拍，只有持续跑飞才会连续几拍都超限。
        sync_rebase(session, metric, tick);
        std::cout << "\r[同步速度] A " << fixed(a.speed_rpm, 0) << "RPM  B " << fixed(b.speed_rpm, 0)
                  << "RPM  本拍 " << deviation << " / 累计 " << cumulative << " 脉冲   " << std::flush;
        std::this_thread::sleep_until(std::min(until, Clock::now() + std::chrono::milliseconds(70)));
    }
    std::cout << "\n";
    if (g_interrupted) {
        g_interrupted = 0;
        std::cout << "[已取消] 收到中断，两轴同时急停。\n";
        sync_halt_both(session);
        sync_report_move(session, name, true);
        return Disposition::keep_going;
    }
    std::cout << "到时，两轴同时急停…\n";
    sync_halt_both(session);
    sync_report_move(session, name, true);
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
    std::cout << "两轴同时急停并确认…\n";
    sync_halt_both(session);
    sync_report_move(session, "两轴急停收尾", true);
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

// 双轴常转的主循环看护：每 200 ms 复查两轴安全项与偏差，每秒打印一行。
void sync_run_tick(Session& session, bool& prompt_shown) {
    if (!session.sync_run_active || !session.second) return;
    const auto now = Clock::now();
    if (now < session.sync_run_next_tick) return;
    session.sync_run_next_tick = now + std::chrono::milliseconds(200);
    Snapshot a{}, b{};
    const Result ok_a = session.motor.poll_safety(a, after_ms(2000));
    const Result ok_b = session.second->poll_safety(b, after_ms(2000));
    if (!ok_a || !ok_b) {
        prompt_shown = false;
        std::cout << "[失败] 双轴常转看护：" << sync_axis_failure_text(ok_a, ok_b) << "\n";
        sync_run_report_elapsed(session);
        session.sync_run_active = false;
        sync_trip(session, "常转期间失去安全观察");
        return;
    }
    const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - session.sync_run_started).count();
    const int64_t metric = sync_aligned_metric(a, b);
    const int64_t deviation = sync_abs_diff(metric, session.sync_base_pulses);
    const int64_t cumulative = sync_abs_diff(metric, session.sync_origin_pulses);
    const int64_t limit = sync_speed_limit(session, sync_base_age_ms(session, now));
    if (limit > 0 && deviation > limit) {
        if (++session.sync_run_over >= kSyncTripSamples) {
            prompt_shown = false;
            sync_run_report_elapsed(session);
            session.sync_run_active = false;
            std::cout << "\n[!] 两轴本拍偏差 " << deviation << " 电机脉冲连续 " << session.sync_run_over
                      << " 拍超过门限 " << limit << " 电机脉冲（已运行 " << elapsed_ms << " ms；基础阈值 "
                      << fixed(session.sync_tolerance_deg, 4) << "°，允许速度差 "
                      << fixed(session.sync_drift_rpm, 2) << " 电机RPM；自起点累计偏差 " << cumulative
                      << " 电机脉冲，折算等效速度差 "
                      << fixed(sync_deviation_rpm(cumulative, elapsed_ms), 2) << " 电机RPM）\n";
            sync_trip(session, "常转位置偏差超限");
            return;
        }
    } else {
        session.sync_run_over = 0;
    }
    // 无论当拍是否超限都重取基准：一次性错位只算一拍，只有持续跑飞才会连续几拍都超限。
    sync_rebase(session, metric, now);
    if (now < session.sync_run_next_line) return;
    session.sync_run_next_line = now + std::chrono::seconds(1);
    prompt_shown = false;
    std::cout << "[双轴常转 " << elapsed_ms << "ms 设定 " << session.sync_run_rpm << " 电机RPM] A "
              << one_line_snapshot(a) << "\n"
              << "                  B " << one_line_snapshot(b) << "  累计偏差 " << cumulative
              << " 脉冲\n";
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
        std::cout << "[!] set-node 会写入设备地址并保存（0x2615 → 0x2614=1），需重新上电后生效。\n"
                     "    确认后请加 --yes 重新执行。\n";
        return Disposition::keep_going;
    }
    uint32_t node = 0;
    if (!parse_u32(text, 1, 127, node)) {
        std::cout << "用法：set-node <1~127>\n";
        return Disposition::keep_going;
    }
    const Result result = session.motor.stage_node_id(static_cast<uint8_t>(node), after_ms(session.timeout_ms));
    if (report(result, "暂存新节点号 " + std::to_string(node))) {
        std::cout << "    本工具仍用旧节点 " << static_cast<unsigned>(session.node)
                  << " 通信；请断电重新上电，再用 --node " << node << " 重新连接。\n";
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

// ===== 全屏界面（TUI）=====
// 交互模式外挂的一层外壳：把原有输出收进环形日志区、把提示符挪到屏幕底部的输入行、
// 把两轴读数画成常驻面板。命令解析与执行仍然走原来的 dispatch()，所以命令集、两种运行
// 模式、偏差判据、收尾汇总、安全锁止的行为一律不变 —— 这一层只负责画屏和收键盘。
namespace tui {

constexpr int kMaxLogLines = 1000;     // 日志环形缓冲保留的行数
constexpr int kMaxLineChars = 4096;    // 单行字符上限，防止进度行无限增长
constexpr int kMaxHistoryLines = 200;  // 输入历史条数
constexpr int kMinRefreshMs = 200;
constexpr int kMaxRefreshMs = 60000;
constexpr int kFrameIdleMs = 200;      // 无输入时最长多久重画一次（面板里的秒数在走）
constexpr int kLogKeepOnExit = 200;    // 退出时回吐给终端的输出行数上限

// 命令名与别名，供输入行 Tab 补全；与 dispatch() 里的分支一一对应。
const char* const kCommands[] = {
    "help", "quit", "exit", "status", "monitor", "read-od", "enable", "disable", "stop",
    "rel", "abs", "pulses", "jog", "spin", "run", "speed",
    "sync-enable", "sync-disable", "sync-stop", "sync-status", "sync-zero", "sync-tol",
    "sync-drift", "set-lead", "sync-rel", "sync-abs", "sync-mm", "sync-spin", "sync-run",
    "sync-dir", "extend", "retract", "one", "one-stop",
    "set-profile-speed", "set-accel", "set-current", "set-zero", "set-node", "set-bitrate",
    "tui-refresh", "tui-size",
};

struct State {
    bool active = false;
    bool color = false;
    termios saved{};
    int rows = 24;
    int cols = 80;
    int fixed_rows = 0;                  // tui-size 指定的行数，0 = 自动探测
    int fixed_cols = 0;
    int log_height = 10;
    std::streambuf* saved_out = nullptr;
    std::streambuf* saved_err = nullptr;
    // 进入全屏时 dup 出来的原始 stdout（画屏只写这个 fd）与原始 stderr。
    int screen_out_fd = -1;
    int err_backup_fd = -1;
    int capture_read = -1;               // 捕获管道读端（非阻塞）
    int capture_write = -1;              // 捕获管道写端，被 dup2 到 fd 1/2
    std::mutex lock;
    std::deque<std::string> log;
    std::string partial;                 // 尚未换行的当前行（运动进度行会反复重写它）
    int log_view = 0;                    // 日志往上翻的行数，0 = 跟到最新
    std::string input;
    std::size_t cursor = 0;
    std::vector<std::string> history;
    int history_index = -1;
    std::string scratch;
    std::vector<std::string> pending;    // 已回车、等待主循环取走的命令行
    bool quit_requested = false;
    bool dirty = true;
    int escape = 0;                      // 0 无、1 收到 ESC、2 收到 CSI
    std::string escape_text;
    Snapshot panel_a{};
    Snapshot panel_b{};
    bool panel_valid = false;
    std::string panel_note;
    Clock::time_point panel_at{};
    Clock::time_point next_poll{};
    int failures = 0;
    bool state_hint_done = false;
    int link_state = -1;
    uint32_t link_bitrate = 0;
    Clock::time_point next_link_check{};
    int refresh_ms = 1000;
    bool force_poll = false;
    Clock::time_point last_frame{};
};

State g;

// cout / cerr 的落点：把每个字节收进环形日志，不直接写屏幕（否则会撕坏画面）。
class LogBuf final : public std::streambuf {
public:
    void feed(const char* text, std::size_t count) {
        std::lock_guard<std::mutex> guard(g.lock);
        for (std::size_t i = 0; i < count; ++i) {
            const unsigned char byte = static_cast<unsigned char>(text[i]);
            if (escape_) {                                   // 整个 ANSI 转义序列丢掉
                if (byte >= 0x40 && byte <= 0x7e) escape_ = false;
                continue;
            }
            if (byte == 0x1b) { escape_ = true; continue; }
            if (byte == '\n') { commit(); continue; }
            if (byte == '\r') { g.partial.clear(); continue; } // 进度行整行重写
            if (byte == '\t') {
                if (g.partial.size() + 4 <= static_cast<std::size_t>(kMaxLineChars))
                    g.partial.append(4, ' ');
                continue;
            }
            if (byte < 0x20 || byte == 0x7f) continue;
            if (g.partial.size() < static_cast<std::size_t>(kMaxLineChars))
                g.partial.push_back(text[i]);
        }
        g.dirty = true;
    }
protected:
    int_type overflow(int_type ch) override {
        if (traits_type::eq_int_type(ch, traits_type::eof())) return traits_type::not_eof(ch);
        const char byte = static_cast<char>(ch);
        feed(&byte, 1);
        return ch;
    }
    std::streamsize xsputn(const char* text, std::streamsize count) override {
        feed(text, static_cast<std::size_t>(count));
        return count;
    }
    int sync() override { return 0; }
private:
    void commit() {
        while (!g.partial.empty() && g.partial.back() == ' ') g.partial.pop_back();
        g.log.push_back(g.partial);
        g.partial.clear();
        while (static_cast<int>(g.log.size()) > kMaxLogLines) g.log.pop_front();
    }
    bool escape_ = false;
};

LogBuf g_sink_out_object;
LogBuf g_sink_err_object;

// 解一个 UTF-8 码点；非法序列返回 U+FFFD 并至少前进一个字节。
uint32_t decode_utf8(const std::string& text, std::size_t& index) {
    const unsigned char first = static_cast<unsigned char>(text[index]);
    if (first < 0x80) { ++index; return first; }
    int extra = 0;
    uint32_t code = 0;
    if ((first & 0xe0) == 0xc0) { extra = 1; code = first & 0x1fu; }
    else if ((first & 0xf0) == 0xe0) { extra = 2; code = first & 0x0fu; }
    else if ((first & 0xf8) == 0xf0) { extra = 3; code = first & 0x07u; }
    else { ++index; return 0xfffd; }
    if (index + static_cast<std::size_t>(extra) + 1 > text.size()) {
        index = text.size();
        return 0xfffd;
    }
    for (int k = 1; k <= extra; ++k) {
        const unsigned char next = static_cast<unsigned char>(text[index + static_cast<std::size_t>(k)]);
        if ((next & 0xc0) != 0x80) { ++index; return 0xfffd; }
        code = (code << 6) | (next & 0x3fu);
    }
    index += static_cast<std::size_t>(extra) + 1;
    return code;
}

// 终端显示列数：中日韩与全角标点算 2 列，组合符算 0 列，其余算 1 列。
int char_columns(uint32_t code) {
    if (code == 0) return 0;
    if ((code >= 0x0300 && code <= 0x036f) || code == 0x200b || code == 0xfeff) return 0;
    if ((code >= 0x1100 && code <= 0x115f) || (code >= 0x2e80 && code <= 0x303e) ||
        (code >= 0x3041 && code <= 0x33ff) || (code >= 0x3400 && code <= 0x4dbf) ||
        (code >= 0x4e00 && code <= 0x9fff) || (code >= 0xa000 && code <= 0xa4cf) ||
        (code >= 0xac00 && code <= 0xd7a3) || (code >= 0xf900 && code <= 0xfaff) ||
        (code >= 0xfe30 && code <= 0xfe6f) || (code >= 0xff00 && code <= 0xff60) ||
        (code >= 0xffe0 && code <= 0xffe6) || (code >= 0x20000 && code <= 0x3fffd))
        return 2;
    return 1;
}

int text_columns(const std::string& text) {
    int total = 0;
    std::size_t index = 0;
    while (index < text.size()) total += char_columns(decode_utf8(text, index));
    return total;
}

// 截断/补齐到正好 columns 列，不切断 UTF-8 序列。
std::string fit_columns(const std::string& text, int columns) {
    if (columns <= 0) return std::string();
    std::string out;
    int used = 0;
    std::size_t index = 0;
    while (index < text.size()) {
        const std::size_t start = index;
        const int width = char_columns(decode_utf8(text, index));
        if (used + width > columns) break;
        out.append(text, start, index - start);
        used += width;
    }
    if (used < columns) out.append(static_cast<std::size_t>(columns - used), ' ');
    return out;
}

// 前 columns 列对应的字节偏移，用来把反显区切在显示列边界上。
std::size_t bytes_for_columns(const std::string& text, int columns) {
    std::size_t index = 0;
    int used = 0;
    while (index < text.size()) {
        const std::size_t start = index;
        const int width = char_columns(decode_utf8(text, index));
        if (used + width > columns) return start;
        used += width;
    }
    return text.size();
}

// 按优先级从左到右拼装一行：整段放不下就丢掉它和它后面的段，不在词中间截断。
// 第一项必须留下 —— 它是这一行最重要的一项，超宽时交给 put_row 按列截断，
// 否则一行最重要的信息会因为"装不下"而整行变成空白。
std::string join_segments(const std::vector<std::string>& segments, int width) {
    std::string line;
    int used = 0;
    for (const std::string& segment : segments) {
        const int need = text_columns(segment);
        if (used + need > width) {
            if (line.empty()) line = segment;
            break;
        }
        line += segment;
        used += need;
    }
    return line;
}

// 画屏只写这个 fd：进入全屏后 fd 1 已经被重定向到捕获管道，不能再往那里画。
void write_raw(const std::string& text) {
    const int fd = g.screen_out_fd >= 0 ? g.screen_out_fd : STDOUT_FILENO;
    std::size_t offset = 0;
    while (offset < text.size()) {
        const ssize_t written = ::write(fd, text.data() + offset, text.size() - offset);
        if (written > 0) { offset += static_cast<std::size_t>(written); continue; }
        if (written < 0 && errno == EINTR) continue;
        return;
    }
}

// 把重定向进管道的 fd 1/2 输出收进日志区。底层模块（transport）用 std::fprintf(stderr,…)
// 报警，不经过 iostream，只有靠 fd 重定向才拦得住 —— 否则这些字会落在光标位置把画面写花。
void pump_capture() {
    if (g.capture_read < 0) return;
    char buffer[512];
    for (;;) {
        const ssize_t received = ::read(g.capture_read, buffer, sizeof(buffer));
        if (received > 0) {
            g_sink_out_object.feed(buffer, static_cast<std::size_t>(received));
            continue;
        }
        if (received < 0 && errno == EINTR) continue;
        return;
    }
}

int env_int(const char* name, int minimum, int maximum, int fallback) {
    const char* text = std::getenv(name);
    if (text == nullptr || *text == '\0') return fallback;
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || value < minimum || value > maximum) return fallback;
    return static_cast<int>(value);
}

// 尺寸优先级：tui-size 指定 > 环境变量 COLUMNS/LINES > 终端报告 > 80x24 兜底。
// 串口终端经常报不出窗口尺寸，这时候要用环境变量或 tui-size 指定。
void detect_size() {
    struct winsize window{};
    const bool probed = ::ioctl(STDOUT_FILENO, TIOCGWINSZ, &window) == 0;
    int rows = env_int("LINES", 8, 300, 0);
    int cols = env_int("COLUMNS", 40, 600, 0);
    if (rows <= 0 && probed && window.ws_row > 0) rows = window.ws_row;
    if (cols <= 0 && probed && window.ws_col > 0) cols = window.ws_col;
    if (rows <= 0) rows = 24;
    if (cols <= 0) cols = 80;
    if (g.fixed_rows > 0) rows = g.fixed_rows;
    if (g.fixed_cols > 0) cols = g.fixed_cols;
    g.rows = std::max(8, std::min(300, rows));
    g.cols = std::max(40, std::min(600, cols));
}

void push_log(const std::string& line) {
    std::lock_guard<std::mutex> guard(g.lock);
    g.log.push_back(line);
    while (static_cast<int>(g.log.size()) > kMaxLogLines) g.log.pop_front();
    g.log_view = 0;
    g.dirty = true;
}

// 取日志区要显示的那几行：始终贴底（最新在最后一行），log_view 决定往上翻多少行。
std::vector<std::string> log_window(int count) {
    std::vector<std::string> lines;
    {
        std::lock_guard<std::mutex> guard(g.lock);
        lines.assign(g.log.begin(), g.log.end());
        if (!g.partial.empty()) lines.push_back(g.partial);
    }
    const int total = static_cast<int>(lines.size());
    int start = total - count - g.log_view;
    int lead = 0;
    if (start < 0) { lead = -start; start = 0; }
    std::vector<std::string> out;
    out.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const int slot = i - lead;
        if (slot < 0) { out.push_back(std::string()); continue; }
        const int index = start + slot;
        out.push_back(index >= 0 && index < total ? lines[static_cast<std::size_t>(index)]
                                                  : std::string());
    }
    return out;
}

int log_total_lines() {
    std::lock_guard<std::mutex> guard(g.lock);
    return static_cast<int>(g.log.size()) + (g.partial.empty() ? 0 : 1);
}

void scroll_log(int direction, bool page) {
    const int total = log_total_lines();
    const int step = page ? std::max(1, g.log_height - 2) : 1;
    g.log_view += direction * step;
    if (g.log_view > std::max(0, total - 1)) g.log_view = std::max(0, total - 1);
    if (g.log_view < 0) g.log_view = 0;
    g.dirty = true;
}

void clear_log() {
    {
        std::lock_guard<std::mutex> guard(g.lock);
        g.log.clear();
        g.partial.clear();
    }
    g.log_view = 0;
    g.dirty = true;
}

void force_refresh() {
    g.force_poll = true;
    g.failures = 0;
    g.next_poll = Clock::now();
    g.panel_note.clear();
    g.dirty = true;
}

// 命令名补全：只在补第一个词（命令名本身）时工作，多个候选时列出候选。
void complete_command() {
    if (g.input.find(' ') != std::string::npos) return;
    std::vector<std::string> hits;
    for (const char* name : kCommands) {
        const std::string candidate = name;
        if (candidate.compare(0, g.input.size(), g.input) == 0) hits.push_back(candidate);
    }
    if (hits.empty()) return;
    if (hits.size() == 1) {
        g.input = hits[0] + " ";
        g.cursor = g.input.size();
        g.dirty = true;
        return;
    }
    std::string common = hits[0];
    for (const std::string& hit : hits) {
        std::size_t shared = 0;
        while (shared < common.size() && shared < hit.size() && common[shared] == hit[shared]) ++shared;
        common.resize(shared);
    }
    if (common.size() > g.input.size()) {
        g.input = common;
        g.cursor = common.size();
    }
    std::string listing = "补全候选（" + std::to_string(hits.size()) + "）：";
    for (const std::string& hit : hits) listing += " " + hit;
    push_log(listing);
}

void insert_utf8(unsigned char byte) {
    if (g.input.size() >= 256) return;
    g.input.insert(g.cursor, 1, static_cast<char>(byte));
    ++g.cursor;
    g.dirty = true;
    g.log_view = 0;
}

void backspace_input() {
    if (g.cursor == 0) return;
    std::size_t start = g.cursor - 1;
    while (start > 0 && (static_cast<unsigned char>(g.input[start]) & 0xc0) == 0x80) --start;
    g.input.erase(start, g.cursor - start);
    g.cursor = start;
    g.dirty = true;
}

void delete_input() {
    if (g.cursor >= g.input.size()) return;
    std::size_t end = g.cursor + 1;
    while (end < g.input.size() && (static_cast<unsigned char>(g.input[end]) & 0xc0) == 0x80) ++end;
    g.input.erase(g.cursor, end - g.cursor);
    g.dirty = true;
}

void kill_word_input() {
    while (g.cursor > 0 && g.input[g.cursor - 1] == ' ') backspace_input();
    while (g.cursor > 0 && g.input[g.cursor - 1] != ' ') backspace_input();
}

void history_step(int direction) {
    if (g.history.empty()) return;
    if (g.history_index < 0) {
        g.scratch = g.input;
        g.history_index = static_cast<int>(g.history.size());
    }
    g.history_index += direction;
    if (g.history_index < 0) g.history_index = 0;
    if (g.history_index > static_cast<int>(g.history.size())) g.history_index = static_cast<int>(g.history.size());
    g.input = g.history_index == static_cast<int>(g.history.size())
                  ? g.scratch
                  : g.history[static_cast<std::size_t>(g.history_index)];
    g.cursor = g.input.size();
    g.dirty = true;
}

void submit_input() {
    const std::string line = g.input;
    g.input.clear();
    g.cursor = 0;
    g.history_index = -1;
    g.scratch.clear();
    if (!line.empty() && (g.history.empty() || g.history.back() != line)) {
        g.history.push_back(line);
        while (static_cast<int>(g.history.size()) > kMaxHistoryLines) g.history.erase(g.history.begin());
    }
    g.pending.push_back(line);
    g.log_view = 0;
    g.dirty = true;
}

void handle_escape(unsigned char byte) {
    if (g.escape == 1) {
        if (byte == '[' || byte == 'O') { g.escape = 2; g.escape_text.clear(); return; }
        g.escape = 0;
        return;
    }
    g.escape_text.push_back(static_cast<char>(byte));
    if (!(byte >= 0x40 && byte <= 0x7e)) {          // 还没收到终止字节
        if (g.escape_text.size() > 8) g.escape = 0;
        return;
    }
    const std::string sequence = g.escape_text;
    g.escape = 0;
    if (sequence == "A") history_step(-1);
    else if (sequence == "B") history_step(1);
    else if (sequence == "C") { if (g.cursor < g.input.size()) ++g.cursor; g.dirty = true; }
    else if (sequence == "D") { if (g.cursor > 0) --g.cursor; g.dirty = true; }
    else if (sequence == "H") { g.cursor = 0; g.dirty = true; }
    else if (sequence == "F") { g.cursor = g.input.size(); g.dirty = true; }
    else if (sequence == "3~") delete_input();
    else if (sequence == "5~") scroll_log(1, true);      // PageUp：整页上翻
    else if (sequence == "6~") scroll_log(-1, true);     // PageDown：整页下翻
}

// 键盘分流：可打印字节进输入行，控制键走各自的快捷键，ESC 序列单独攒齐再处理。
void feed_keys(const char* data, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        const unsigned char byte = static_cast<unsigned char>(data[i]);
        if (g.escape > 0) { handle_escape(byte); continue; }
        switch (byte) {
        case 0x1b: g.escape = 1; g.escape_text.clear(); break;
        case '\r': case '\n': submit_input(); break;
        case 0x7f: case 0x08: backspace_input(); break;
        case 0x09: complete_command(); break;
        case 0x01: g.cursor = 0; g.dirty = true; break;
        case 0x05: g.cursor = g.input.size(); g.dirty = true; break;
        case 0x02: scroll_log(1, true); break;
        case 0x06: scroll_log(-1, true); break;
        case 0x0b: g.input.erase(g.cursor); g.dirty = true; break;
        case 0x15: g.input.clear(); g.cursor = 0; g.dirty = true; break;
        case 0x17: kill_word_input(); break;
        case 0x10: scroll_log(1, false); break;
        case 0x0e: scroll_log(-1, false); break;
        case 0x0c: clear_log(); break;
        case 0x12: force_refresh(); break;
        case 0x04:
            if (g.input.empty()) g.quit_requested = true;
            else delete_input();
            break;
        default:
            if (byte >= 0x20) insert_utf8(byte);
            break;
        }
    }
}

std::string mode_text(const Session& session) {
    if (session.sync_run_active) return "双轴常转";
    if (session.motor.sustaining()) return "常转";
    if (session.enabled) return "已使能";
    return "只读";
}

std::string metric_degrees_text(int64_t pulses) {
    return fixed(static_cast<double>(pulses) * 360.0 /
                     static_cast<double>(yz_aim::kOutputPulsesPerRevolution), 4) + "°";
}

std::string axis_head_line(uint8_t node, const char* tag, const Snapshot& sample, int width) {
    std::vector<std::string> segments;
    segments.push_back(" " + std::string(tag) + " 轴 节点 " + std::to_string(static_cast<unsigned>(node)));
    segments.push_back("   状态 " +
        field_or_dash(sample.status_sample.valid, hex_text(sample.statusword, 4)));
    segments.push_back("   报警 " +
        field_or_dash(sample.alarm_sample.valid, hex_text(sample.alarm_code, 4)));
    segments.push_back("   输出 " + output_state_text(sample));
    if (sample.unsafe_latched) segments.push_back("   [本会话已锁止]");
    return join_segments(segments, width);
}

std::string axis_body_line(const Snapshot& sample, int width) {
    std::vector<std::string> segments;
    segments.push_back("       位置 " +
        field_or_dash(sample.position_sample.valid, std::to_string(sample.position_pulses)) +
        " 电机脉冲（" +
        field_or_dash(sample.position_sample.valid, fixed(sample.output_position_degrees, 3)) + "°）");
    segments.push_back("   转速 " +
        field_or_dash(sample.speed_sample.valid, fixed(sample.speed_rpm, 1)) + " 电机RPM");
    segments.push_back("   电压 " +
        field_or_dash(sample.voltage_sample.valid, fixed(sample.voltage_v, 2)) + " V");
    segments.push_back("   电流 " +
        field_or_dash(sample.current_sample.valid, fixed(sample.current_a, 3)) + " A");
    segments.push_back("   温度 " +
        field_or_dash(sample.temperature_sample.valid, std::to_string(sample.temperature_c)) + " ℃");
    return join_segments(segments, width);
}

// 面板状态的一句话说明：为什么没有读数、自动刷新是否被关掉、上次读数有多旧。
std::string panel_status_text() {
    if (g.refresh_ms <= 0) return "面板自动刷新已关闭（Ctrl-R 手动读）";
    if (!g.panel_note.empty()) return g.panel_note;
    if (g.panel_valid && g.panel_at.time_since_epoch().count() != 0) {
        const double age = std::chrono::duration<double>(Clock::now() - g.panel_at).count();
        return "面板 " + fixed(age, 1) + " s 前读到";
    }
    return std::string();
}

std::string metric_line_one(const Session& session, int width) {
    std::vector<std::string> segments;
    const std::string status = panel_status_text();
    if (!g.panel_valid) {
        // 未读到时把状态并进这一句里，这样窄终端下丢的也只是后面的段，状态不会被挤掉。
        std::string note = " 两轴读数：尚未读到";
        if (!status.empty())
            note += "（" + status + "）";
        else
            note += " —— 会话空闲时会自动读一次，也可以按 Ctrl-R 立即读。";
        segments.push_back(note);
        return join_segments(segments, width);
    }
    const int64_t metric = sync_pair_metric(g.panel_a.position_pulses, g.panel_b.position_pulses);
    segments.push_back(" 一致性指标 posA+posB = " + std::to_string(metric) + " 电机脉冲（" +
                       metric_degrees_text(metric) + "）");
    if (session.sync_zero_valid) {
        const int64_t tilt =
            metric - (static_cast<int64_t>(session.sync_zero_a) + session.sync_zero_b);
        segments.push_back("   相对 sync-zero 两端错位 " + travel_text(session.lead_mm, tilt));
    } else {
        segments.push_back("   行程基准未设置（sync-zero 把此刻记成 0 行程）");
    }
    if (!status.empty()) segments.push_back("   " + status);
    return join_segments(segments, width);
}

// 只放配置项：镜像方向、两个判据、导程 —— 窄终端下丢的永远是最后面的导程。
std::string metric_line_two(const Session& session, int width) {
    std::vector<std::string> segments;
    segments.push_back(" 镜像 " + std::to_string(static_cast<unsigned>(session.node)) +
                       (session.plus_is_a ? "(+)" : "(-)") + "/" +
                       std::to_string(static_cast<unsigned>(session.node2)) +
                       (session.plus_is_a ? "(-)" : "(+)"));
    segments.push_back("   sync-tol " + fixed(session.sync_tolerance_deg, 4) + "° = " +
                       std::to_string(static_cast<int64_t>(sync_tolerance_pulses(session))) + "脉冲");
    segments.push_back("   sync-drift " + fixed(session.sync_drift_rpm, 2) + "RPM");
    segments.push_back("   导程 " +
        (session.lead_mm > 0 ? fixed(session.lead_mm, 4) + " mm/圈" : std::string("未设置")));
    return join_segments(segments, width);
}

// 面板上只放错误种类：Result::message 是英文长句，放上去会把阈值挤掉；完整原因写进日志。
std::string short_error_text(const Result& result) {
    switch (result.error) {
    case Error::timeout: return "SDO 超时";
    case Error::transport: return "传输层故障";
    case Error::malformed_reply: return "应答格式异常";
    case Error::abort_reply: return "驱动器中止";
    case Error::unsafe: return "状态不安全";
    case Error::alarm: return "驱动器报警";
    case Error::limit: return "超出限位";
    case Error::moving: return "仍在运动";
    case Error::stale: return "数据过期";
    case Error::cancelled: return "已取消";
    case Error::invalid_argument: return "参数无效";
    default: return "未知原因";
    }
}

// 控制器的状态只在打开接口时读了一次；这里用只读 netlink 查询定期刷新，
// 状态在会话中途变化（例如总线上没人应答掉进 ERROR-PASSIVE）时标题栏与提示能跟上。
// 只查内核，不发任何 CAN 帧。
void refresh_link_state(const Session& session) {
    const auto now = Clock::now();
    if (now < g.next_link_check) return;
    g.next_link_check = now + std::chrono::milliseconds(2000);
    yz_aim::CanLinkInfo link;
    std::string error;
    if (yz_aim::query_can_link(session.transport.interface_name(), link, error)) {
        g.link_state = link.state;
        g.link_bitrate = link.bitrate;
    }
}

// 面板读数：只在"两轴都没有运动、且本会话没有持有输出使能"时自动读。
// 本会话持有输出时 snapshot() 若安全读数失败可能补发一帧未确认急停，面板刷新不该产生
// 这种副作用；Ctrl-R 手动读属于用户明确要求，等同于敲一次 sync-status。
void maybe_poll(const Session& session) {
    const bool forced = g.force_poll;
    if (g.refresh_ms <= 0 && !forced) return;
    const auto now = Clock::now();
    if (!forced && now < g.next_poll) return;
    g.force_poll = false;
    // 控制器不在 ERROR-ACTIVE 时收发都不可信，自动读既不会成功也会继续发帧把状态拖住，
    // 所以先不读，隔一秒再看一次；需要时按 Ctrl-R 顶着读。
    if (!forced && g.link_state > 0) {
        g.panel_note = "can0 未处于 ERROR-ACTIVE，面板不自动读";
        g.next_poll = now + std::chrono::milliseconds(1000);
        return;
    }
    // 成功读到过一次之前不自动发帧：空总线上第一帧没人应答就会把 can0 推进 ERROR-PASSIVE，
    // 这一帧必须由用户明确要求（Ctrl-R），成功一次之后就一直自动刷新。
    if (!forced && !g.panel_valid) {
        if (g.panel_note.empty()) g.panel_note = "按 Ctrl-R 读一次，之后会自动刷新";
        g.next_poll = now + std::chrono::milliseconds(1000);
        return;
    }
    if (!forced && (session.motor.watchdog_active() || session.motor.sustaining() ||
                    session.sync_run_active)) {
        g.panel_note = "使能/运动中不自动读";
        g.next_poll = now + std::chrono::milliseconds(500);
        return;
    }
    const uint64_t budget = std::max<uint64_t>(session.timeout_ms, 5000);
    const Snapshot a = session.motor.snapshot(after_ms(budget));
    Snapshot b{};
    bool second_ok = true;
    if (session.second) {
        b = session.second->snapshot(after_ms(budget));
        second_ok = static_cast<bool>(b.result);
    }
    if (a.result && second_ok) {
        g.panel_a = a;
        g.panel_b = b;
        g.panel_valid = true;
        g.panel_at = Clock::now();
        g.panel_note.clear();
        g.failures = 0;
        g.next_poll = now + std::chrono::milliseconds(g.refresh_ms);
        return;
    }
    ++g.failures;
    const Result& cause = a.result ? b.result : a.result;
    g.panel_note = short_error_text(cause) + " ×" + std::to_string(g.failures);
    if (g.failures == 1) push_log("面板读数失败：" + cause.message);
    if (g.failures >= 2) {
        // 连读两次都读不到：不再自动发帧（空总线上继续发帧只会把 can0 拖在 ERROR-PASSIVE），
        // 面板上的旧读数也一起清掉 —— 读不到就不显示可疑的旧值，要看按 Ctrl-R 再读一次。
        g.panel_valid = false;
        g.panel_a = Snapshot{};
        g.panel_b = Snapshot{};
    }
    const int base = g.refresh_ms > 0 ? g.refresh_ms : 1000;
    const int backoff = std::min(base * (1 << std::min(g.failures, 5)), 30000);
    g.next_poll = now + std::chrono::milliseconds(backoff);
    g.dirty = true;
}

void put_row(std::string& frame, int line, const std::string& sgr, const std::string& text, int width) {
    frame += "\033[" + std::to_string(line) + ";1H";
    if (g.color && !sgr.empty()) frame += sgr;
    frame += fit_columns(text, width);
    if (g.color && !sgr.empty()) frame += "\033[0m";
}

void render(const Session& session) {
    if (!g.active) return;
    pump_capture();                       // 先收走底层直接写到 fd 2 的报警
    refresh_link_state(session);          // 控制器状态会变，标题栏按最新状态画
    const auto now = Clock::now();
    if (!g.dirty && now - g.last_frame < std::chrono::milliseconds(kFrameIdleMs)) return;
    g.last_frame = now;
    g.dirty = false;

    detect_size();
    maybe_poll(session);

    struct Row {
        std::string sgr;
        std::string text;
    };
    const bool show_axes = g.rows >= 20;
    const bool show_metrics = g.rows >= 16;
    // 最底一行留空、最后列也不写：整屏写满会触发终端滚动，画面会自己往上跑。
    const int width = g.cols - 1;
    std::vector<Row> header;

    {
        // 标题栏按重要性从左到右排：会话模式（输出是否已使能）在最前，内核状态名的中文解释
        // 放最后 —— 窄终端截断时只丢解释，不会丢掉"现在能不能动"这件事。
        const uint32_t bitrate = g.link_bitrate > 0 ? g.link_bitrate
                                                    : session.transport.reported_bitrate();
        std::string rate = "?";
        if (bitrate > 0) {
            if (bitrate % 1000000 == 0) rate = std::to_string(bitrate / 1000000) + "M";
            else if (bitrate % 1000 == 0) rate = std::to_string(bitrate / 1000) + "k";
            else rate = std::to_string(bitrate);
        }
        // 拆出内核状态名的中文解释，窄终端下把它整段丢掉而不是拦腰截断；
        // 修复命令只在日志里提示一次，标题栏不留半截句子。
        // 注意这里必须用字符串字面量：'（' 是多字节字符常量，不是 UTF-8 字符。
        std::string state_name = yz_aim::can_state_text(g.link_state);
        std::string state_note;
        const std::size_t note_start = state_name.find("（");
        if (note_start != std::string::npos) {
            state_note = state_name.substr(note_start);
            state_name.resize(note_start);
        }
        // 只有非 0（不是 ERROR-ACTIVE）才提示，且每"掉进去一次"提示一次；恢复正常后可以再次提示。
        // 拆成两行：整句太长会被日志区按列截成半句，看起来像命令没写完。
        if (g.link_state > 0 && !g.state_hint_done) {
            g.state_hint_done = true;
            push_log("提示：控制器状态 " + state_name + state_note + "。");
            push_log("    先执行 sh tools/can0_up.sh can0 1000000 复位到 ERROR-ACTIVE 再试。");
        } else if (g.link_state == 0) {
            g.state_hint_done = false;
        }
        // 标题栏按三档降级，保证"会话模式 / CAN 接口 / 两轴节点号"永远看得见：
        //   完整标识 + 内核状态 + 波特率 → 短标识 + 内核状态 + 波特率 → 短标识 + 接口 + 节点号。
        // 中文解释是整段加在后面的，放不下就整段丢掉，不留半句。
        const std::string mode = mode_text(session);
        const std::string iface = session.transport.interface_name();
        const std::string nodes = "   A" + std::to_string(static_cast<unsigned>(session.node)) +
                                  "/B" + std::to_string(static_cast<unsigned>(session.node2));
        const std::string full = " YZ-AIM 双轴镜像维护工具 TUI  " + mode + "   " + iface + " " +
                                 state_name + " " + rate + nodes;
        const std::string brief = " YZ-AIM 双轴 TUI  " + mode + "   " + iface + " " + state_name +
                                  " " + rate + nodes;
        const std::string least = " YZ-AIM 双轴 TUI  " + mode + "   " + iface + nodes;
        std::string title = full;
        if (text_columns(title) > width) title = brief;
        if (text_columns(title) > width) title = least;
        if (!state_note.empty()) {
            const std::string note = "   " + state_note;
            if (text_columns(title) + text_columns(note) <= width) title += note;
        }
        header.push_back({"\033[7m", title});
    }
    if (show_metrics) {
        header.push_back({std::string(), std::string()});
        if (show_axes) {
            header.push_back({std::string(), axis_head_line(session.node, "A", g.panel_a, width)});
            header.push_back({std::string(), axis_body_line(g.panel_a, width)});
            header.push_back({std::string(), axis_head_line(session.node2, "B", g.panel_b, width)});
            header.push_back({std::string(), axis_body_line(g.panel_b, width)});
            header.push_back({std::string(), std::string()});
        }
        header.push_back({std::string(), metric_line_one(session, width)});
        header.push_back({std::string(), metric_line_two(session, width)});
    }

    const int input_row = g.rows - 1;
    const int hint_row = input_row - 1;
    const int separator_row = input_row - 2;
    const int log_top = static_cast<int>(header.size()) + 2;
    int log_bottom = separator_row - 1;
    if (log_bottom < log_top) log_bottom = log_top;
    g.log_height = std::max(1, log_bottom - log_top + 1);

    std::string frame;
    frame.reserve(static_cast<std::size_t>(g.rows) * static_cast<std::size_t>(width) * 3 + 1024);

    int line = 1;
    for (const Row& row : header) put_row(frame, line++, row.sgr, row.text, width);

    const std::string rule(static_cast<std::size_t>(width), '-');
    put_row(frame, line++, "\033[36m", rule, width);

    const std::vector<std::string> lines = log_window(g.log_height);
    for (int i = 0; i < g.log_height; ++i)
        put_row(frame, log_top + i, std::string(), lines[static_cast<std::size_t>(i)], width);

    put_row(frame, separator_row, "\033[36m", rule, width);

    if (g.log_view > 0) {
        put_row(frame, hint_row, "\033[33m",
                " 日志已上翻 " + std::to_string(g.log_view) + " 行（共 " +
                    std::to_string(log_total_lines()) +
                    " 行；Ctrl-N 或 PageDown 回到底部）",
                width);
    } else {
        // 提示行也按项拼装：窄终端下丢掉末尾几项，不会把一个快捷键说明写一半。
        std::vector<std::string> hints;
        hints.push_back(" Ctrl-R 立即读一次");
        hints.push_back(" | Ctrl-L 清日志");
        hints.push_back(" | Ctrl-P/Ctrl-N 逐行翻日志");
        hints.push_back(" | PgUp/PgDn 整页翻日志");
        hints.push_back(" | Tab 补全");
        hints.push_back(" | 上/下箭头翻历史");
        hints.push_back(" | Ctrl-D 退出");
        hints.push_back(" | help 命令表");
        put_row(frame, hint_row, "\033[36m", join_segments(hints, width), width);
    }

    const std::string prompt = "yz(" + mode_text(session) + ")> ";
    const std::string body = fit_columns(prompt + g.input, width);
    const int prompt_columns = text_columns(prompt);
    frame += "\033[" + std::to_string(input_row) + ";1H";
    if (g.color) frame += "\033[7m";
    const std::size_t split = bytes_for_columns(body, prompt_columns);
    frame.append(body, 0, split);
    if (g.color) frame += "\033[0m";
    frame.append(body, split, std::string::npos);

    const int cursor_columns = prompt_columns + text_columns(g.input.substr(0, g.cursor));
    const int cursor_col = std::min(width, std::max(1, cursor_columns + 1));
    frame += "\033[" + std::to_string(input_row) + ";" + std::to_string(cursor_col) + "H";

    write_raw(frame);
}

void end() {
    if (!g.active) return;
    g.active = false;
    pump_capture();                       // 收尾前把管道里剩下的拿走
    if (g.saved_out != nullptr) std::cout.rdbuf(g.saved_out);
    if (g.saved_err != nullptr) std::cerr.rdbuf(g.saved_err);
    g.saved_out = nullptr;
    g.saved_err = nullptr;
    write_raw(g.color ? "\033[0m\033[?1049l" : "\033[?1049l");   // 回到进入前的那个屏幕
    ::tcsetattr(STDIN_FILENO, TCSANOW, &g.saved);
    // 把 fd 1/2 还给终端：之后的 std::cout 就是直接写屏幕了。
    if (g.screen_out_fd >= 0) {
        (void)::dup2(g.screen_out_fd, STDOUT_FILENO);
        ::close(g.screen_out_fd);
        g.screen_out_fd = -1;
    }
    if (g.err_backup_fd >= 0) {
        (void)::dup2(g.err_backup_fd, STDERR_FILENO);
        ::close(g.err_backup_fd);
        g.err_backup_fd = -1;
    }
    if (g.capture_read >= 0) { ::close(g.capture_read); g.capture_read = -1; }
    if (g.capture_write >= 0) { ::close(g.capture_write); g.capture_write = -1; }
    // 退出前把本次会话的输出交回终端，否则它会随全屏一起消失。
    std::vector<std::string> lines;
    {
        std::lock_guard<std::mutex> guard(g.lock);
        lines.assign(g.log.begin(), g.log.end());
        if (!g.partial.empty()) lines.push_back(g.partial);
    }
    if (lines.empty()) return;
    const std::size_t keep = static_cast<std::size_t>(kLogKeepOnExit);
    const std::size_t first = lines.size() > keep ? lines.size() - keep : 0;
    std::cout << "=== 已退出全屏界面，下面是本次会话的输出"
              << (first > 0 ? "（只保留最后 " + std::to_string(keep) + " 行）" : std::string())
              << " ===\n";
    for (std::size_t i = first; i < lines.size(); ++i) std::cout << lines[i] << "\n";
    std::cout << std::flush;
}

// 进入全屏：只在真终端上做，且把终端属性原样保存，退出时恢复。
bool begin(bool interactive) {
    if (g.active) return true;
    if (!interactive || g_tui_disabled) return false;
    if (::isatty(STDOUT_FILENO) != 1) return false;
    const char* term = std::getenv("TERM");
    const bool dumb = term == nullptr || *term == '\0' || std::strcmp(term, "dumb") == 0;
    if (dumb && !g_tui_forced) return false;
    if (::tcgetattr(STDIN_FILENO, &g.saved) != 0) return false;
    termios raw = g.saved;
    // 只关掉行缓冲与回显：ISIG 保持开着，Ctrl-C 仍然走信号处理，与逐行模式完全一致。
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
    g.saved_out = std::cout.rdbuf(&g_sink_out_object);
    g.saved_err = std::cerr.rdbuf(&g_sink_err_object);
    // fd 1/2 也收进管道：底层 transport 用 std::fprintf(stderr,…) 报警，不经过 iostream。
    g.screen_out_fd = ::dup(STDOUT_FILENO);
    g.err_backup_fd = ::dup(STDERR_FILENO);
    int pipe_fds[2] = {-1, -1};
    if (::pipe(pipe_fds) == 0) {
        g.capture_read = pipe_fds[0];
        g.capture_write = pipe_fds[1];
        const int flags = ::fcntl(g.capture_read, F_GETFL, 0);
        if (flags >= 0) (void)::fcntl(g.capture_read, F_SETFL, flags | O_NONBLOCK);
        if (g.screen_out_fd >= 0) (void)::dup2(g.capture_write, STDOUT_FILENO);
        if (g.err_backup_fd >= 0) (void)::dup2(g.capture_write, STDERR_FILENO);
    }
    g.color = !dumb && std::getenv("NO_COLOR") == nullptr;
    g.active = true;
    g.next_poll = Clock::now() + std::chrono::milliseconds(500);
    std::atexit(&end);
    write_raw("\033[?1049h\033[2J\033[H");
    return true;
}

// 取一行命令：full 为真时 line 有效，返回 false 表示输入结束（EOF / Ctrl-D）该退出。
bool read_line(std::string& line, bool& ready) {
    ready = false;
    if (g.quit_requested) return false;
    if (!g.pending.empty()) {
        line = g.pending.front();
        g.pending.erase(g.pending.begin());
        g.dirty = true;
        ready = true;
        return true;
    }
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready_count = ::poll(&input, 1, kInputPollMs);
    if (ready_count < 0) return errno == EINTR || errno == EAGAIN;
    if (ready_count == 0) return true;
    char chunk[256];
    const ssize_t received = ::read(STDIN_FILENO, chunk, sizeof(chunk));
    if (received < 0) return errno == EINTR || errno == EAGAIN;
    if (received == 0) {
        std::cout << "标准输入已结束，退出。\n";
        return false;
    }
    feed_keys(chunk, static_cast<std::size_t>(received));
    if (g.quit_requested) return false;
    if (!g.pending.empty()) {
        line = g.pending.front();
        g.pending.erase(g.pending.begin());
        ready = true;
    }
    return true;
}

Disposition command_refresh(Session& session, const std::string& text) {
    (void)session;
    uint32_t milliseconds = 0;
    if (!parse_u32(text, 0, kMaxRefreshMs, milliseconds)) {
        std::cout << "用法：tui-refresh <0~" << kMaxRefreshMs << ">（毫秒；0 = 关闭面板自动刷新）\n";
        return Disposition::keep_going;
    }
    if (milliseconds > 0 && milliseconds < kMinRefreshMs) {
        std::cout << "最小 " << kMinRefreshMs << " ms；0 表示关闭自动刷新。\n";
        return Disposition::keep_going;
    }
    g.refresh_ms = static_cast<int>(milliseconds);
    g.failures = 0;
    g.panel_note.clear();
    g.next_poll = Clock::now();
    g.dirty = true;
    if (milliseconds == 0)
        std::cout << "面板自动刷新已关闭（Ctrl-R 仍可手动读一次）。\n";
    else
        std::cout << "面板自动刷新间隔 = " << milliseconds
                  << " ms（只在两轴都没有运动、本会话未使能输出、且已经成功读到过一次时才读）。\n";
    return Disposition::keep_going;
}

Disposition command_size(Session& session, const std::string& rows_text, const std::string& cols_text) {
    (void)session;
    uint32_t rows = 0;
    uint32_t cols = 0;
    if (!parse_u32(rows_text, 0, 300, rows) || !parse_u32(cols_text, 0, 600, cols) ||
        (rows != 0 && rows < 8) || (cols != 0 && cols < 40) || (rows == 0) != (cols == 0)) {
        std::cout << "用法：tui-size <行 8~300> <列 40~600>（两个 0 = 恢复按终端自动探测）\n";
        return Disposition::keep_going;
    }
    g.fixed_rows = static_cast<int>(rows);
    g.fixed_cols = static_cast<int>(cols);
    g.dirty = true;
    if (rows == 0) std::cout << "界面尺寸改为按终端自动探测。\n";
    else std::cout << "界面尺寸固定为 " << rows << " 行 × " << cols << " 列。\n";
    return Disposition::keep_going;
}

} // namespace tui

Disposition dispatch(Session& session, const std::vector<std::string>& tokens) {
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

    if (name == "tui-refresh") {
        if (arguments.size() != 1) {
            std::cout << "用法：tui-refresh <毫秒>（0 = 关闭面板自动刷新）\n";
            return Disposition::keep_going;
        }
        return tui::command_refresh(session, arguments[0]);
    }
    if (name == "tui-size") {
        if (arguments.size() != 2) {
            std::cout << "用法：tui-size <行> <列>（两个 0 = 恢复自动探测）\n";
            return Disposition::keep_going;
        }
        return tui::command_size(session, arguments[0], arguments[1]);
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
    if (session.dual_enabled && session.second) {
        std::cout << "退出前两轴同时停止并失能输出…\n";
        // 顺序：先对两轴各发一帧未确认急停（串行地"先确认 A 再确认 B"会让 B 多跑最多 7 秒），
        // 再交替复查到两轴都静止，最后各自失能。
        sync_halt_both(session);
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
    // 全屏界面只在真正的交互终端上启用；管道、重定向与单命令模式一律走原来的逐行界面。
    const bool fullscreen = tui::begin(interactive);
    std::string line;

    for (;;) {
        if (g_motor != nullptr && session.motor.watchdog_active()) {
            const Result serviced = session.motor.service_watchdog(after_ms(kInputPollMs));
            if (!serviced) {
                report(serviced, "维持所属心跳");
                break;
            }
        }
        // 双轴模式下第二轴有自己的心跳生产者，等待输入期间同样要维持。
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

        line.clear();
        bool have_line = false;
        if (fullscreen) {
            tui::render(session);
            if (!tui::read_line(line, have_line)) break;
            if (!have_line) continue;
        } else {
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
            if (newline == std::string::npos) {
                line = buffer;
                buffer.clear();
            } else {
                line = buffer.substr(0, newline);
                buffer.erase(0, newline + 1);
            }
        }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        prompt_shown = false;

        const std::vector<std::string> tokens = split_fields(line);
        if (tokens.empty()) continue;
        if (dispatch(session, tokens) == Disposition::quit) break;
    }

    // 先退出全屏再收尾，这样 cleanup_and_exit 的提示能留在终端上看得见。
    tui::end();
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

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, options)) return 2;
    if (options.help) { print_usage(); return 0; }
    if (options.version) { std::cout << kProgram << " " << kVersion << "\n"; return 0; }
    g_tui_disabled = options.plain_ui;
    g_tui_forced = options.force_tui;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const auto node = static_cast<uint8_t>(options.node);
    SocketCanTransport transport(options.interface_name, node);
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
        transport2 = std::make_unique<SocketCanTransport>(options.interface_name, node2);
        std::string error2;
        if (!transport2->open(error2)) {
            std::cerr << "[失败] 打开 " << options.interface_name << "（B 轴节点 "
                      << static_cast<unsigned>(node2) << "）：" << error2 << "\n";
            return 3;
        }
    }
    print_banner(options, transport);

    Motor motor(transport, node);
    g_motor = &motor;

    Session session{motor, transport, node, options.timeout_ms, options.assume_yes};
    if (transport2) {
        session.transport2 = transport2.get();
        session.second = std::make_unique<yz_aim::Motor>(*transport2, options.node2);
        session.node2 = static_cast<uint8_t>(options.node2);
    }
    const int code = options.command.empty() ? run_session(session) : run_single_command(session, options.command);
    g_motor = nullptr;
    return code;
}
