// Hobbywing X6 PLUS 推进电调（HW-UAVCAN，29 位扩展帧）：编解码、保活线程与遥测统计。
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mvd/bus.h"
#include "mvd/protocol.h"

namespace mvd {

// 电调协议常量
constexpr uint8_t kHwSrcNode = 0x64;      // 主机源节点
constexpr uint8_t kHwNodeLeft = 0x01;     // 左电调，取 CH1
constexpr uint8_t kHwNodeRight = 0x02;    // 右电调，取 CH2
constexpr uint16_t kHwMaxThrottle = 8191;  // 14 位油门
constexpr uint16_t kHwReadyOn = 0xAA;      // 油门使能
constexpr uint16_t kHwReadyOff = 0x55;     // 油门失能

constexpr uint16_t kHwMsgReadyControl = 20012;  // 0x4E2C
constexpr uint16_t kHwMsgGetEscId = 20013;      // 0x4E2D
constexpr uint16_t kHwMsgTelemetry1 = 20050;    // 转速/油门/状态
constexpr uint16_t kHwMsgTelemetry2 = 20051;    // 电压/电流/温度
constexpr uint16_t kHwMsgTelemetry3 = 20052;    // MOS/电容/电机温度
constexpr uint16_t kHwMsgRawCommand = 20100;    // 0x4E84

constexpr uint8_t kHwSrvSetId = 210;          // 0xD2
constexpr uint8_t kHwSrvBaud = 211;           // 0xD3
constexpr uint8_t kHwSrvThrottleSelect = 215; // 0xD7

constexpr uint8_t kHwPriorityHighest = 0;
constexpr uint8_t kHwPriorityMedium = 16;

// 29 位 ID：priority(5) | data_type_id(16) | source_node(7)
uint32_t hw_message_id(uint8_t priority, uint16_t data_type_id, uint8_t source_node);
// 29 位 ID：priority(5) | service_type(8) | 1 | dest(7) | 1 | source(7)
uint32_t hw_service_request_id(uint8_t priority, uint8_t service_type, uint8_t dest,
                               uint8_t source);
// 四路 14 位油门打包成 7 字节（每通道低字节先、MSB 先入，共 56 bit）
void hw_pack4_throttle(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3, Frame* out);
std::array<uint16_t, 4> hw_unpack4_throttle(const Frame& f);

// 一条控制帧（ID + 数据 + 长度）
struct HwFrame {
  uint32_t id = 0;
  Frame data{};
  int len = 0;
};

struct FanCfg {
  bool control = true;          // 是否下发控制帧（--fan-disable 关闭）
  uint16_t throttle = 500;      // 油门原始值 0~8191
  int period_ms = 20;           // 保活周期，50Hz
  double init_zero_sec = 2.0;   // 初始化后零油门保持时长
  int stop_zero_frames = 20;    // 收尾零油门帧数
  double stale_sec = 3.0;       // 遥测超过该时长无更新视为失联
  std::vector<uint8_t> nodes{0x01, 0x02};
  std::vector<int> channels{0, 1};
};

// 单台电调的遥测累计
struct FanNode {
  uint8_t node = 0;
  int channel = 0;
  long long msg1 = 0;
  long long msg2 = 0;
  long long msg3 = 0;
  long long dup = 0;
  bool has_rpm = false;
  int rpm = 0;
  bool has_thr = false;
  int thr_raw = 0;
  bool has_status = false;
  int status = 0;
  bool has_volt = false;
  double volt = 0.0;
  bool has_cur = false;
  double cur = 0.0;
  bool has_temp = false;
  double temp = 0.0;
  bool has_mos = false;
  int mos_temp = 0;
  bool has_cap = false;
  int cap_temp = 0;
  bool has_motor_temp = false;
  int motor_temp = 0;
  double last_rx = 0.0;
  double last_unique_rx = 0.0;
  double max_gap_ms = 0.0;
  long long frames() const { return msg1 + msg2 + msg3; }
};

struct FanStats {
  long long tx = 0;
  long long tx_fail = 0;
  long long late = 0;
  long long telemetry = 0;
  std::vector<double> period_ms;
  std::map<uint8_t, FanNode> nodes;
};

// 待写入 can-frames.log 的抽样记录
struct FanLogItem {
  std::string dir;
  uint32_t id = 0;
  Frame data{};
  int len = 0;
  std::string note;
};

// 风机链路：初始化、50Hz 保活线程、遥测解析与统计。
class FanLink {
 public:
  using Wait = std::function<void(int)>;  // 等待期间由主线程持续收 CAN，参数为 ms
  FanLink(CanBus* bus, FanCfg cfg);
  ~FanLink();

  // 选 CAN 油门 -> 使能 -> 连续零油门；期间阻塞约 init_zero_sec 秒。
  bool init(std::string* err, const Wait& wait = {});
  void start();     // 启动保活线程
  void stop();      // 停止保活线程（不发帧）
  bool shutdown(const Wait& wait = {});  // 返回失能帧是否成功入队

  // 主线程收到扩展帧时调用；返回 true 表示已识别为风机帧。
  bool on_frame(uint32_t can_id, const Frame& data, int dlc, double ts);
  FanStats stats() const;
  std::vector<FanLogItem> take_logs();
  void log_control_sample(const std::string& dir, const HwFrame& f, const std::string& note);

  bool inited() const { return inited_; }
  bool needs_shutdown() const { return ready_sent_; }
  bool control_enabled() const { return cfg_.control; }
  uint16_t throttle() const { return cfg_.throttle; }
  const FanCfg& cfg() const { return cfg_; }
  // 最近一次遥测距今秒数，无遥测返回 -1。
  double last_rx_age() const;

 private:
  // 同一节点同一类消息的重复帧判定状态
  struct DupState {
    bool has = false;
    uint8_t tid = 0;
    Frame data{};
    double ts = 0.0;
  };

  void worker();
  bool send(HwFrame& f, int* err);
  void push_log(const std::string& dir, uint32_t id, const Frame& data, int len,
                const std::string& note);

  CanBus* bus_ = nullptr;
  FanCfg cfg_;
  bool inited_ = false;
  bool ready_sent_ = false;  // 即使后续零油门初始化失败，也必须尝试失能
  std::map<uint32_t, uint8_t> tx_tids_;  // 按消息类型 / 服务目标独立递增

  mutable std::mutex mu_;
  FanStats stats_;
  std::map<std::pair<uint8_t, int>, DupState> dup_;
  std::vector<FanLogItem> pending_;
  long long rx_seen_ = 0;
  bool has_last_tx_ = false;
  double last_tx_ = 0.0;

  std::atomic<bool> run_{false};
  std::thread th_;
};

}  // namespace mvd
