// 测试运行时：参数、阶段调度、统计采集、日志与报告。
#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "mvd/bus.h"
#include "mvd/fan.h"
#include "mvd/json_mini.h"
#include "mvd/protocol.h"

namespace mvd {

// 周期任务：心跳、读角度、写位置。
enum Task { kHb = 0, kPoll = 1, kWrite = 2 };
// 一组循环内的三个阶段。
enum Phase { kFwd = 0, kHold = 1, kRev = 2 };

extern const char* const kPhaseName[3];  // fwd / hold / rev
extern const char* const kPhaseCn[3];    // 正转 / 保持 / 反转

struct Config {
  double minutes = 30.0;
  std::string motors_mode = "auto";  // auto / 1 / 2
  std::vector<int> ids{1, 2};
  std::string iface = "can0";
  std::string mode = "time";  // time 按秒数 / turns 按圈数
  double speed_out = 20.0;    // 输出轴 °/s
  double speed_dps = 720.0;   // 电机轴 °/s
  int fwd_ms = 3000;
  int hold_ms = 1000;
  int rev_ms = 3000;
  double turns_fwd = 2.0;
  double turns_rev = 2.0;
  int poll_ms = 70;
  int read_ms = 50;
  int write_ms = 100;
  int hb_ms = 50;
  double motor_lost_sec = 10.0;
  std::string log_root = "/root/MVD-X6PLUS/logs";
  bool check_only = false;
  bool read_only = false;  // 只读长测：不使能、不松闸、不写位置
  FanCfg fan;

  // 派生量
  bool turns_mode = false;
  double poll_s = 0.07;
  double read_budget = 0.05;
  double write_s = 0.1;
  double hb_s = 0.05;
  double cycle_s = 7.0;
  int64_t off_fwd_raw = 0;  // 正转目标相对原点的偏移
  int64_t off_rev_raw = 0;  // 反转目标相对原点的偏移
  std::string stamp;
  int motors_used = 0;

  void derive();           // 由基础参数算出派生量
  std::string plan_line();  // 时序说明，用于日志与报告
  mj::Value to_json() const;
};

// 一类请求的统计。
struct ReqStat {
  long long req = 0;
  long long ok = 0;
  long long timeout = 0;
  std::vector<double> lat;  // ms
};

struct MotorStat {
  long long tx = 0;
  long long rx = 0;
  long long tx_fail = 0;
  long long rx_other = 0;
  long long rx_err_frame = 0;
  ReqStat hb, poll, write;
};

struct MotorState {
  bool has_temp = false;
  int temp = 0;
  bool has_volt = false;
  double volt = 0.0;
  bool has_cur = false;
  double cur = 0.0;
  bool has_state = false;
  int state = 0;
  bool has_err = false;
  int err = 0;
  double last_rx = 0.0;
  bool has_angle = false;
  int64_t angle = 0;
  int speed = 0;
  int iq = 0;
  int err_seen = 0;
};

// 某阶段内的采样。
struct PhaseStat {
  std::vector<int64_t> pos;
  std::vector<int> speed;
};

// 一组循环中某台电机某阶段的首末值。
struct CyclePhase {
  int samples_pos = 0;
  bool has_pos = false;
  int64_t pos_start = 0;
  int64_t pos_end = 0;
  bool has_speed = false;
  int speed_min = 0;
  int speed_max = 0;
};

struct CycleRec {
  int cycle = 0;
  std::map<int, std::array<CyclePhase, 3>> per_motor;
};

// 停机收尾的确认结果。
struct StopInfo {
  bool has_hold = false;
  int64_t hold_raw = 0;
  std::string brake;
  bool brake_ok = false;
  int speed_dps = 0;
  bool has_final = false;
  int64_t final_raw = 0;
  bool has_return_err = false;
  int64_t return_err_raw = 0;
};

// 收到中断信号时抛出，由入口统一收尾。
struct Interrupted {};

class Runtime {
 public:
  explicit Runtime(Config cfg);
  ~Runtime();

  void setup();   // 建立连接、探测电机、释放抱闸、读原点
  void run();     // 主循环
  std::string finish(const std::string& outcome, const std::string& reason = std::string());
  void log(const std::string& msg);

  // 收到 Ctrl+C 后由主循环退出。
  static void request_stop();

 private:
  void open_logs();
  void relocate_logs();
  void event(const std::string& name, const mj::Value& fields);
  void frames(const char* dir, uint32_t can_id, const Frame& data, int len,
              const std::string& note);
  bool request(int mid, uint8_t cmd, const Frame& payload, double budget_s, Frame* out_data,
               double* lat_ms, std::string* err);
  bool probe(int mid, double budget_s);
  bool receive_frame(double timeout_s, double* ts, uint32_t* cid, int* dlc, Frame* data);
  void receive_for(double seconds);
  bool read_angle(int mid, int64_t* out);
  void do_task(int mid, int task);
  void sample(int mid, int kind, int64_t value);
  void sample_speed(int mid, int value);
  void enter_phase(int phase);
  void health();
  std::map<int, StopInfo> shutdown_motors();
  void record_cycle(int no);
  void minute_summary(bool final_one);
  void drain_fan_logs();
  mj::Value build_summary(const std::map<int, StopInfo>& stop);
  std::string build_report(const mj::Value& summary, const std::map<int, StopInfo>& stop);
  CyclePhase stat_for(int mid, int phase) const;

  Config cfg_;
  CanBus bus_;
  bool bus_open_ = false;
  FanLink fan_;

  std::map<int, MotorStat> stats_;
  std::map<int, MotorState> mstate_;
  std::map<int, int64_t> origin_;
  std::map<int, int64_t> target_;
  std::map<int, std::array<PhaseStat, 3>> ph_stats_;
  std::map<int, std::array<PhaseStat, 3>> cy_stats_;
  std::map<std::pair<int, int>, double> due_;
  std::vector<CycleRec> cycles_;
  std::vector<int> motors_;

  CanSnap can_before_;
  CanSnap can_after_;
  int phase_ = -1;
  int cycle_of_ = -1;
  double t_start_ = 0.0;
  double t_end_ = 0.0;
  double last_health_ = 0.0;
  std::string prev_ctrl_state_;
  std::string outcome_ = "running";
  std::string reason_;

  // 每分钟通信小结用
  int minute_no_ = 0;
  double last_minute_ = 0.0;
  std::vector<double> minute_lat_;
  std::map<int, std::array<long long, 3>> prev_to_;
  std::map<uint8_t, long long> prev_fan_frames_;
  // 风机遥测失联告警的限频时刻
  double last_fan_warn_ = 0.0;

  std::string run_dir_;
  std::string started_at_;
  std::ofstream f_test_;
  std::ofstream f_frames_;
  std::ofstream f_events_;
  std::ofstream f_health_;
  long long frame_no_ = 0;
  bool files_open_ = false;
};

// 时间与格式化小工具。
std::string iso_now();
std::string hm_now();
std::string stamp_now();
std::string fmt_d(double v, int nd);
std::string fmt_i(int v);
std::string num_text(double v);
double percentile(std::vector<double> v, double p);

}  // namespace mvd
