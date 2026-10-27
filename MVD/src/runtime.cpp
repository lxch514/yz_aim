// 测试运行时：参数派生、日志、收发调度与动作阶段。
#include "mvd/runtime.h"

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdexcept>

namespace mvd {

const char* const kPhaseName[3] = {"fwd", "hold", "rev"};
const char* const kPhaseCn[3] = {"正转", "保持", "反转"};

namespace {

volatile std::sig_atomic_t g_stop = 0;

const uint32_t kCanErrFlag = 0x20000000u;
const uint32_t kCanSffMask = 0x000007FFu;

double round_to(double v, int nd) {
  double scale = std::pow(10.0, nd);
  return std::round(v * scale) / scale;
}

void sleep_s(double s) {
  if (s <= 0.0) return;
  struct timespec ts;
  ts.tv_sec = static_cast<time_t>(s);
  ts.tv_nsec = static_cast<long>((s - static_cast<double>(ts.tv_sec)) * 1e9);
  ::nanosleep(&ts, nullptr);
}

bool mkdir_p(const std::string& path) {
  if (path.empty()) return false;
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur.push_back(path[i]);
    if (path[i] == '/' && cur.size() > 1) {
      ::mkdir(cur.c_str(), 0755);
    }
  }
  ::mkdir(path.c_str(), 0755);
  struct stat sb;
  return ::stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
}

std::string join_path(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/') return a + b;
  return a + "/" + b;
}

}  // namespace

// 数字转文本，整数不带小数点。
std::string num_text(double v) {
  char buf[48];
  if (v == std::floor(v) && std::fabs(v) < 1e15) {
    std::snprintf(buf, sizeof(buf), "%.0f", v);
  } else {
    std::snprintf(buf, sizeof(buf), "%g", v);
  }
  return std::string(buf);
}

std::string iso_now() {
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tmv;
  ::localtime_r(&ts.tv_sec, &tmv);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
                tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min,
                tmv.tm_sec, ts.tv_nsec / 1000000);
  return std::string(buf);
}

std::string hm_now() {
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tmv;
  ::localtime_r(&ts.tv_sec, &tmv);
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return std::string(buf);
}

std::string stamp_now() {
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tmv;
  ::localtime_r(&ts.tv_sec, &tmv);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d-%02d%02d%02d", tmv.tm_year + 1900, tmv.tm_mon + 1,
                tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return std::string(buf);
}

std::string fmt_d(double v, int nd) {
  if (!std::isfinite(v)) return "-";
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.*f", nd, v);
  return std::string(buf);
}

std::string fmt_i(int v) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%d", v);
  return std::string(buf);
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  double k = static_cast<double>(v.size() - 1) * p;
  size_t f = static_cast<size_t>(std::floor(k));
  size_t c = static_cast<size_t>(std::ceil(k));
  if (f == c) return v[static_cast<size_t>(std::floor(k))];
  return v[f] + (v[c] - v[f]) * (k - static_cast<double>(f));
}

// ---------------------------------------------------------------- Config

void Config::derive() {
  turns_mode = (mode == "turns");
  speed_dps = speed_out * kGearRatio;
  if (turns_mode) {
    // 按圈数：目标行程由圈数给出，阶段时长按速度折算。
    double raw_per_s = speed_dps * kRawPerMotorDeg;
    double fwd_s = (raw_per_s > 0.0) ? (turns_fwd * kRawPerTurn / raw_per_s) : 0.0;
    double rev_s = (raw_per_s > 0.0) ? (turns_rev * kRawPerTurn / raw_per_s) : 0.0;
    fwd_ms = static_cast<int>(std::lround(fwd_s * 1000.0));
    rev_ms = static_cast<int>(std::lround(rev_s * 1000.0));
    off_fwd_raw = static_cast<int64_t>(std::llround(turns_fwd * kRawPerTurn));
    off_rev_raw = static_cast<int64_t>(std::llround((turns_fwd - turns_rev) * kRawPerTurn));
  } else {
    off_fwd_raw = static_cast<int64_t>(
        std::llround(speed_dps * (fwd_ms / 1000.0) * kRawPerMotorDeg));
    off_rev_raw = 0;
  }
  poll_s = poll_ms / 1000.0;
  if (read_ms > poll_ms) read_ms = poll_ms;
  read_budget = read_ms / 1000.0;
  write_s = write_ms / 1000.0;
  hb_s = hb_ms / 1000.0;
  cycle_s = (fwd_ms + hold_ms + rev_ms) / 1000.0;
  motors_used = (motors_mode == "auto") ? 0 : std::atoi(motors_mode.c_str());
  if (stamp.empty()) stamp = stamp_now();
}

std::string Config::plan_line() {
  char buf[512];
  if (turns_mode) {
    std::snprintf(buf, sizeof(buf),
                  "位置模式循环 正转 %.2f 圈 → 保持 %dms → 反转 %.2f 圈（速度 %g°/s 输出轴，约 "
                  "%dms + %dms）",
                  turns_fwd, hold_ms, turns_rev, speed_out, fwd_ms, rev_ms);
  } else {
    std::snprintf(buf, sizeof(buf), "位置模式循环 正转 %dms → 保持 %dms → 反转 %dms", fwd_ms,
                  hold_ms, rev_ms);
  }
  return std::string(buf);
}

mj::Value Config::to_json() const {
  mj::Value o = mj::Value::object();
  o.set("iface", mj::Value::string(iface));
  mj::Value ids_v = mj::Value::array();
  for (int id : ids) ids_v.push(mj::Value::number(id));
  o.set("ids", ids_v);
  o.set("minutes", mj::Value::number(minutes));
  o.set("mode", mj::Value::string(mode));
  o.set("speed_out_dps", mj::Value::number(speed_out));
  o.set("speed_motor_dps", mj::Value::number(speed_dps));
  o.set("fwd_ms", mj::Value::number(fwd_ms));
  o.set("hold_ms", mj::Value::number(hold_ms));
  o.set("rev_ms", mj::Value::number(rev_ms));
  if (turns_mode) {
    o.set("turns_fwd", mj::Value::number(turns_fwd));
    o.set("turns_rev", mj::Value::number(turns_rev));
  }
  o.set("poll_ms", mj::Value::number(poll_ms));
  o.set("read_ms", mj::Value::number(read_ms));
  o.set("write_ms", mj::Value::number(write_ms));
  o.set("hb_ms", mj::Value::number(hb_ms));
  o.set("delta_raw", mj::Value::number(static_cast<double>(off_fwd_raw)));
  o.set("off_rev_raw", mj::Value::number(static_cast<double>(off_rev_raw)));
  o.set("motor_lost_sec", mj::Value::number(motor_lost_sec));
  o.set("check_only", mj::Value::boolean(check_only));
  o.set("read_only", mj::Value::boolean(read_only));
  return o;
}

// ---------------------------------------------------------------- Runtime

void Runtime::request_stop() { g_stop = 1; }

Runtime::Runtime(Config cfg) : cfg_(std::move(cfg)) {
  started_at_ = iso_now();
  run_dir_ = join_path(cfg_.log_root,
                       "mvd-" + num_text(cfg_.minutes) + "分钟-" + cfg_.stamp + "-" +
                           (cfg_.motors_used > 0 ? std::to_string(cfg_.motors_used) + "台电机"
                                                 : std::string("待探测")));
  mkdir_p(run_dir_);
  open_logs();
}

Runtime::~Runtime() {
  if (files_open_) {
    f_test_.flush();
    f_frames_.flush();
    f_events_.flush();
    f_health_.flush();
  }
}

void Runtime::open_logs() {
  f_test_.open(join_path(run_dir_, "test.log"), std::ios::out | std::ios::trunc);
  f_frames_.open(join_path(run_dir_, "can-frames.log"), std::ios::out | std::ios::trunc);
  f_events_.open(join_path(run_dir_, "events.jsonl"), std::ios::out | std::ios::trunc);
  f_health_.open(join_path(run_dir_, "bus-health.jsonl"), std::ios::out | std::ios::trunc);
  files_open_ = true;
  f_frames_ << "# CAN_START " << iso_now() << " interface=" << cfg_.iface << "\n";
}

void Runtime::relocate_logs() {
  std::string new_dir =
      join_path(cfg_.log_root, "mvd-" + num_text(cfg_.minutes) + "分钟-" + cfg_.stamp + "-" +
                                   std::to_string(motors_.size()) + "台电机");
  if (new_dir == run_dir_) return;
  if (::rename(run_dir_.c_str(), new_dir.c_str()) == 0) {
    run_dir_ = new_dir;
  }
}

void Runtime::log(const std::string& msg) {
  f_test_ << "[" + hm_now() + "] " + msg << "\n";
  f_test_.flush();
}

void Runtime::event(const std::string& name, const mj::Value& fields) {
  mj::Value rec = mj::Value::object();
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  rec.set("time", mj::Value::number(static_cast<double>(ts.tv_sec) + ts.tv_nsec / 1e9));
  if (t_start_ > 0.0) {
    rec.set("elapsed", mj::Value::number(round_to(now_s() - t_start_, 6)));
  } else {
    rec.set("elapsed", mj::Value());
  }
  rec.set("event", mj::Value::string(name));
  for (const auto& kv : fields.entries()) rec.set(kv.first, kv.second);
  f_events_ << rec.dump(-1) << "\n";
  f_events_.flush();
}

void Runtime::frames(const char* dir, uint32_t can_id, const Frame& data, int len,
                     const std::string& note) {
  if (len <= 0) return;
  ++frame_no_;
  char head[128];
  double el = (t_start_ > 0.0) ? (now_s() - t_start_) : 0.0;
  std::snprintf(head, sizeof(head), "[CAN %s +%.6fs #%lld] %-7s ID=%08X DLC=%d ", cfg_.iface.c_str(),
                el, frame_no_, dir, can_id, len);
  f_frames_ << head << hexs(data, len) << (note.empty() ? "" : (" | " + note)) << "\n";
}

bool Runtime::request(int mid, uint8_t cmd, const Frame& payload, double budget_s, Frame* out_data,
                      double* lat_ms, std::string* err) {
  double t0 = now_s();
  int err_no = 0;
  uint32_t can_id = kIdBase + static_cast<uint32_t>(mid);
  bool ok = bus_.send(can_id, payload, 8, &err_no);
  frames("TX", can_id, payload, 8, cmd_name(cmd));
  MotorStat& st = stats_[mid];
  ++st.tx;
  if (!ok) {
    ++st.tx_fail;
    mj::Value f = mj::Value::object();
    f.set("motor", mj::Value::number(mid));
    char cbuf[16];
    std::snprintf(cbuf, sizeof(cbuf), "0x%02X", cmd);
    f.set("cmd", mj::Value::string(cbuf));
    f.set("errno", mj::Value::number(err_no));
    event("tx_fail", f);
    if (err) *err = std::string("send:") + std::strerror(err_no);
    if (lat_ms) *lat_ms = 0.0;
    return false;
  }
  double deadline = t0 + budget_s;
  while (true) {
    double remain = deadline - now_s();
    if (remain <= 0.0) break;
    double ts = 0.0;
    uint32_t cid = 0;
    int dlc = 0;
    Frame data{};
    if (!bus_.recv(remain, &ts, &cid, &dlc, &data)) break;
    ++st.rx;
    if (cid & kCanErrFlag) {
      ++st.rx_err_frame;
      mj::Value f = mj::Value::object();
      char ibuf[16];
      std::snprintf(ibuf, sizeof(ibuf), "0x%08X", cid);
      f.set("can_id", mj::Value::string(ibuf));
      f.set("data", mj::Value::string(hexs(data, dlc)));
      event("can_error_frame", f);
      frames("RX", cid, data, dlc, "CAN 错误帧");
      continue;
    }
    uint32_t src = cid & kCanSffMask;
    if (src == kIdBase + static_cast<uint32_t>(mid) && dlc >= 1 && data[0] == cmd) {
      double lat = (ts - t0) * 1000.0;
      frames("RX", cid, data, dlc, feedback_note(cmd, data));
      mstate_[mid].last_rx = ts;
      if (out_data) *out_data = data;
      if (lat_ms) *lat_ms = lat;
      return true;
    }
    ++st.rx_other;
    frames("RX", cid, data, dlc, "非本机/非本命令");
  }
  if (lat_ms) *lat_ms = (now_s() - t0) * 1000.0;
  if (err) *err = "timeout";
  return false;
}

bool Runtime::probe(int mid, double budget_s) {
  Frame data{};
  if (!request(mid, 0x9A, cmd_simple(0x9A), budget_s, &data, nullptr, nullptr)) return false;
  State1 p = parse_9a(data);
  MotorState& ms = mstate_[mid];
  ms.has_temp = true;
  ms.temp = p.temp;
  ms.has_volt = true;
  ms.volt = p.volt;
  ms.has_cur = true;
  ms.cur = p.cur;
  ms.has_state = true;
  ms.state = p.state;
  ms.has_err = true;
  ms.err = p.err;
  return true;
}

bool Runtime::read_angle(int mid, int64_t* out) {
  Frame data{};
  if (!request(mid, 0x92, cmd_simple(0x92), cfg_.read_budget, &data, nullptr, nullptr)) {
    return false;
  }
  int64_t ang = parse_92(data);
  mstate_[mid].has_angle = true;
  mstate_[mid].angle = ang;
  if (out) *out = ang;
  return true;
}

void Runtime::setup() {
  t_start_ = now_s();
  std::string err;
  if (!bus_.open(cfg_.iface, &err)) throw std::runtime_error(err);
  bus_open_ = true;

  can_before_ = can_status(cfg_.iface);
  mj::Value f = mj::Value::object();
  f.set("snapshot", can_before_.to_json());
  event("initial_controller", f);

  std::string state_text = can_before_.state.empty() ? "未知" : can_before_.state;
  log("CAN 接口 " + cfg_.iface + "：" + state_text + "，位时序 " +
      (can_before_.valid ? can_before_.bittiming.dump(-1) : std::string("未知")));
  if (can_before_.valid && can_before_.state != "ERROR-ACTIVE") {
    log("警告：控制器状态为 " + state_text + "，通信可能不稳定。");
  }

  for (int mid : cfg_.ids) {
    stats_[mid] = MotorStat();
    mstate_[mid] = MotorState();
  }

  std::vector<int> alive;
  for (int mid : cfg_.ids) {
    if (probe(mid, 0.05)) {
      alive.push_back(mid);
      MotorState& ms = mstate_[mid];
      char buf[256];
      std::snprintf(buf, sizeof(buf), "探测到电机 ID %d（CAN 0x%03X）：温度 %d℃，母线 %s V，错误标志 0x%02X。",
                    mid, kIdBase + mid, ms.temp, fmt_d(ms.volt, 2).c_str(), ms.err);
      log(buf);
      mj::Value e = mj::Value::object();
      e.set("motor", mj::Value::number(mid));
      e.set("ok", mj::Value::boolean(true));
      e.set("temp", mj::Value::number(ms.temp));
      e.set("volt", mj::Value::number(ms.volt));
      e.set("err", mj::Value::number(ms.err));
      event("motor_probe", e);
    } else {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "未收到电机 ID %d（CAN 0x%03X）的回复。", mid, kIdBase + mid);
      log(buf);
      mj::Value e = mj::Value::object();
      e.set("motor", mj::Value::number(mid));
      e.set("ok", mj::Value::boolean(false));
      event("motor_probe", e);
    }
  }

  if (cfg_.motors_mode == "auto") {
    motors_ = alive;
  } else if (cfg_.motors_mode == "1") {
    motors_.clear();
    if (!alive.empty()) motors_.push_back(alive[0]);
  } else {
    motors_.clear();
    for (size_t i = 0; i < cfg_.ids.size() && i < 2; ++i) {
      int mid = cfg_.ids[i];
      if (std::find(alive.begin(), alive.end(), mid) != alive.end()) motors_.push_back(mid);
    }
    if (motors_.size() < 2) {
      throw std::runtime_error("要求双电机运行，但只探测到 " + std::to_string(motors_.size()) +
                               " 台。");
    }
  }
  if (motors_.empty()) {
    throw std::runtime_error("总线上没有可用的 MVD 电机，请检查接线、供电、ID 与 1Mbps 波特率。");
  }

  relocate_logs();
  std::string names;
  for (size_t i = 0; i < motors_.size(); ++i) {
    if (i) names += "、";
    names += std::to_string(motors_[i]);
  }
  log("本次控制 " + std::to_string(motors_.size()) + " 台电机：" + names);
  if (cfg_.check_only) return;

  if (cfg_.read_only) {
    log("只读模式：不清错、不使能、不松抱闸、不写位置，只发 0x9A 心跳与 0x92 读角度。");
    for (int mid : motors_) {
      int64_t ang = 0;
      if (read_angle(mid, &ang)) {
        origin_[mid] = ang;
        target_[mid] = ang;
        char buf[200];
        std::snprintf(buf, sizeof(buf), "电机 %d：当前角度 %.2f°（电机轴），只记录不驱动。", mid,
                      ang / 100.0);
        log(buf);
      } else {
        origin_[mid] = 0;
        target_[mid] = 0;
        log("电机 " + std::to_string(mid) + "：读不到角度，只统计通信。");
      }
    }
    mj::Value e = mj::Value::object();
    e.set("read_only", mj::Value::boolean(true));
    e.set("motors", mj::Value::number(static_cast<double>(motors_.size())));
    event("run_mode", e);
    return;
  }

  for (int mid : motors_) {
    request(mid, 0x9B, cmd_simple(0x9B), cfg_.read_budget, nullptr, nullptr, nullptr);
    bool ok_en = request(mid, 0x88, cmd_simple(0x88), cfg_.read_budget, nullptr, nullptr, nullptr);
    bool ok_br = request(mid, 0x8C, cmd_brake(0x01), cfg_.read_budget, nullptr, nullptr, nullptr);
    char buf[200];
    std::snprintf(buf, sizeof(buf), "电机 %d：清错误、使能 %s、释放抱闸 %s。", mid,
                  ok_en ? "成功" : "无应答", ok_br ? "成功" : "无应答");
    log(buf);
    mj::Value e = mj::Value::object();
    e.set("motor", mj::Value::number(mid));
    e.set("enable", mj::Value::boolean(ok_en));
    e.set("brake_release", mj::Value::boolean(ok_br));
    event("enable", e);
  }
  sleep_s(0.3);

  for (int mid : motors_) {
    int64_t ang = 0;
    if (!read_angle(mid, &ang)) {
      throw std::runtime_error("无法读取电机 " + std::to_string(mid) + " 的初始角度。");
    }
    origin_[mid] = ang;
    target_[mid] = ang;
    char buf[220];
    std::snprintf(buf, sizeof(buf), "电机 %d：初始角度 %.2f°（电机轴），输出轴 %.2f°，作为本次原点。",
                  mid, ang / 100.0, deg_out(ang));
    log(buf);
  }
}

void Runtime::sample(int mid, int kind, int64_t value) {
  if (phase_ < 0) return;
  PhaseStat& ph = ph_stats_[mid][phase_];
  PhaseStat& cy = cy_stats_[mid][phase_];
  if (kind == 0) {
    ph.pos.push_back(value);
    cy.pos.push_back(value);
  } else {
    ph.speed.push_back(static_cast<int>(value));
    cy.speed.push_back(static_cast<int>(value));
  }
}

void Runtime::sample_speed(int mid, int value) {
  if (phase_ < 0) return;
  ph_stats_[mid][phase_].speed.push_back(value);
  cy_stats_[mid][phase_].speed.push_back(value);
}

void Runtime::enter_phase(int phase) {
  phase_ = phase;
  std::string tag = "第" + std::to_string(cycle_of_ + 1) + "组";
  if (cfg_.read_only) {
    if (phase == kFwd) log(tag + "：只读采样中（不写位置，只统计通信）。");
    return;
  }
  char buf[320];
  if (phase == kFwd) {
    for (int mid : motors_) target_[mid] = origin_[mid] + cfg_.off_fwd_raw;
    int mid0 = motors_[0];
    if (cfg_.turns_mode) {
      std::snprintf(buf, sizeof(buf), "%s：正转 → 目标 %+.2f 圈（%+.2f°，输出轴），约 %dms，速度 %g°/s。",
                    tag.c_str(), deg_out(target_[mid0] - origin_[mid0]) / 360.0,
                    deg_out(target_[mid0] - origin_[mid0]), cfg_.fwd_ms, cfg_.speed_out);
    } else {
      std::snprintf(buf, sizeof(buf), "%s：正转 → 目标 %.2f°（输出轴），%dms，速度 %g°/s。",
                    tag.c_str(), deg_out(target_[mid0]), cfg_.fwd_ms, cfg_.speed_out);
    }
  } else if (phase == kHold) {
    for (int mid : motors_) {
      int64_t ang = 0;
      if (read_angle(mid, &ang)) target_[mid] = ang;
    }
    std::snprintf(buf, sizeof(buf), "%s：停止保持 → 锁在当前位置，%dms。", tag.c_str(), cfg_.hold_ms);
  } else {
    for (int mid : motors_) target_[mid] = origin_[mid] + cfg_.off_rev_raw;
    int mid0 = motors_[0];
    if (cfg_.turns_mode) {
      std::snprintf(buf, sizeof(buf), "%s：反转 %+.2f 圈 → 目标 %+.2f°（输出轴），约 %dms。",
                    tag.c_str(), -cfg_.turns_rev,
                    deg_out(target_[mid0] - origin_[mid0]), cfg_.rev_ms);
    } else {
      std::snprintf(buf, sizeof(buf), "%s：反转 → 回原 %.2f°（输出轴），%dms。", tag.c_str(),
                    deg_out(origin_[mid0]), cfg_.rev_ms);
    }
  }
  log(buf);

  mj::Value e = mj::Value::object();
  e.set("phase", mj::Value::string(kPhaseName[phase]));
  e.set("cycle", mj::Value::number(cycle_of_ + 1));
  mj::Value tg = mj::Value::object();
  for (int mid : motors_) tg.set(std::to_string(mid), mj::Value::number(static_cast<double>(target_[mid])));
  e.set("target", tg);
  event("phase_start", e);

  for (int mid : motors_) {
    due_[std::make_pair(mid, static_cast<int>(kWrite))] = now_s() + cfg_.write_s;
    do_task(mid, kWrite);
  }
}

void Runtime::do_task(int mid, int task) {
  if (cfg_.read_only && task == kWrite) return;
  MotorStat& st = stats_[mid];
  Frame data{};
  double lat = 0.0;
  std::string err;

  if (task == kHb) {
    due_[std::make_pair(mid, static_cast<int>(kHb))] = now_s() + cfg_.hb_s;
    bool ok = request(mid, 0x9A, cmd_simple(0x9A), cfg_.read_budget, &data, &lat, &err);
    ReqStat& rec = st.hb;
    ++rec.req;
    if (ok) {
      ++rec.ok;
      rec.lat.push_back(lat);
      State1 p = parse_9a(data);
      MotorState& ms = mstate_[mid];
      if (p.err != 0 && (!ms.has_err || p.err != ms.err)) {
        ++ms.err_seen;
        mj::Value e = mj::Value::object();
        e.set("motor", mj::Value::number(mid));
        char eb[8];
        std::snprintf(eb, sizeof(eb), "0x%02X", p.err);
        e.set("err", mj::Value::string(eb));
        event("motor_error", e);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "电机 %d 报错标志 0x%02X。", mid, p.err);
        log(buf);
      }
      ms.has_temp = true;
      ms.temp = p.temp;
      ms.has_volt = true;
      ms.volt = p.volt;
      ms.has_cur = true;
      ms.cur = p.cur;
      ms.has_state = true;
      ms.state = p.state;
      ms.has_err = true;
      ms.err = p.err;
    } else {
      ++rec.timeout;
      mj::Value e = mj::Value::object();
      e.set("motor", mj::Value::number(mid));
      e.set("budget_ms", mj::Value::number(cfg_.read_ms));
      e.set("err", mj::Value::string(err));
      event("hb_timeout", e);
    }
    return;
  }

  if (task == kPoll) {
    due_[std::make_pair(mid, static_cast<int>(kPoll))] = now_s() + cfg_.poll_s;
    bool ok = request(mid, 0x92, cmd_simple(0x92), cfg_.read_budget, &data, &lat, &err);
    ReqStat& rec = st.poll;
    ++rec.req;
    if (ok) {
      ++rec.ok;
      rec.lat.push_back(lat);
      int64_t ang = parse_92(data);
      mstate_[mid].has_angle = true;
      mstate_[mid].angle = ang;
      sample(mid, 0, ang);
    } else {
      ++rec.timeout;
      mj::Value e = mj::Value::object();
      e.set("motor", mj::Value::number(mid));
      e.set("cmd", mj::Value::string("0x92"));
      e.set("budget_ms", mj::Value::number(cfg_.read_ms));
      event("read_timeout", e);
    }
    return;
  }

  due_[std::make_pair(mid, static_cast<int>(kWrite))] = now_s() + cfg_.write_s;
  bool ok = request(mid, 0xA4, cmd_pos2(cfg_.speed_dps, target_[mid]), cfg_.write_s, &data, &lat,
                    &err);
  ReqStat& rec = st.write;
  ++rec.req;
  if (ok) {
    ++rec.ok;
    rec.lat.push_back(lat);
    Feedback p = parse_fb(data);
    mstate_[mid].speed = p.speed;
    mstate_[mid].iq = p.iq;
    sample_speed(mid, p.speed);
  } else {
    ++rec.timeout;
    mj::Value e = mj::Value::object();
    e.set("motor", mj::Value::number(mid));
    e.set("cmd", mj::Value::string("0xA4"));
    e.set("budget_ms", mj::Value::number(cfg_.write_ms));
    event("write_timeout", e);
  }
}

void Runtime::health() {
  double now = now_s();
  CanSnap snap = can_status(cfg_.iface);
  mj::Value counts = mj::Value::object();
  for (const char* key : {"tx", "rx", "tx_fail", "rx_other", "rx_err_frame"}) {
    long long sum = 0;
    for (int mid : motors_) {
      MotorStat& s = stats_[mid];
      if (std::strcmp(key, "tx") == 0) sum += s.tx;
      else if (std::strcmp(key, "rx") == 0) sum += s.rx;
      else if (std::strcmp(key, "tx_fail") == 0) sum += s.tx_fail;
      else if (std::strcmp(key, "rx_other") == 0) sum += s.rx_other;
      else sum += s.rx_err_frame;
    }
    counts.set(key, mj::Value::number(static_cast<double>(sum)));
  }
  mj::Value ages = mj::Value::object();
  for (int mid : motors_) {
    double lr = mstate_[mid].last_rx;
    if (lr > 0.0) {
      ages.set(std::to_string(mid), mj::Value::number(round_to((now - lr) * 1000.0, 3)));
    } else {
      ages.set(std::to_string(mid), mj::Value());
    }
  }
  mj::Value rec = mj::Value::object();
  rec.set("elapsed", mj::Value::number(round_to(now - t_start_, 6)));
  struct timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  rec.set("wall_time", mj::Value::number(static_cast<double>(ts.tv_sec) + ts.tv_nsec / 1e9));
  rec.set("snapshot", snap.to_json());
  rec.set("counts", counts);
  rec.set("motor_last_rx_age_ms", ages);
  rec.set("phase", phase_ < 0 ? mj::Value() : mj::Value::string(kPhaseName[phase_]));
  rec.set("cycle", mj::Value::number((cycle_of_ < 0 ? 0 : cycle_of_) + 1));
  f_health_ << rec.dump(-1) << "\n";
  f_health_.flush();
  f_frames_.flush();
  last_health_ = now;

  if (now - last_minute_ >= 60.0) {
    minute_summary(false);
    // 主循环偶发长阻塞时只补一条，然后重新对齐，避免连刷多条小结
    last_minute_ = (now - last_minute_ >= 180.0) ? now : (last_minute_ + 60.0);
  }

  if (snap.valid && !snap.state.empty() && snap.state != prev_ctrl_state_) {
    if (!prev_ctrl_state_.empty()) {
      mj::Value e = mj::Value::object();
      e.set("previous", mj::Value::string(prev_ctrl_state_));
      e.set("current", mj::Value::string(snap.state));
      event("controller_state", e);
      log("CAN 控制器状态变化：" + prev_ctrl_state_ + " → " + snap.state + "。");
    }
    prev_ctrl_state_ = snap.state;
  }

  for (int mid : motors_) {
    double lr = mstate_[mid].last_rx;
    if (lr > 0.0 && (now - lr) > cfg_.motor_lost_sec) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "电机 %d 已 %.1f 秒无有效回复，判定掉线。", mid, now - lr);
      throw std::runtime_error(buf);
    }
  }
}

// 累计一行通信小结，写入 test.log 并留一条结构化事件。
void Runtime::minute_summary(bool final_one) {
  ++minute_no_;

  struct Bucket {
    long long tx = 0;
    long long rx = 0;
    long long fail = 0;
    long long errf = 0;
    long long other = 0;
    long long hb[3] = {0, 0, 0};
    long long poll[3] = {0, 0, 0};
    long long wr[3] = {0, 0, 0};
  };

  Bucket total;
  std::map<int, Bucket> per;
  for (int mid : motors_) {
    const MotorStat& s = stats_[mid];
    Bucket b;
    b.tx = s.tx;
    b.rx = s.rx;
    b.fail = s.tx_fail;
    b.errf = s.rx_err_frame;
    b.other = s.rx_other;
    b.hb[0] = s.hb.req;
    b.hb[1] = s.hb.ok;
    b.hb[2] = s.hb.timeout;
    b.poll[0] = s.poll.req;
    b.poll[1] = s.poll.ok;
    b.poll[2] = s.poll.timeout;
    b.wr[0] = s.write.req;
    b.wr[1] = s.write.ok;
    b.wr[2] = s.write.timeout;
    per[mid] = b;

    total.tx += b.tx;
    total.rx += b.rx;
    total.fail += b.fail;
    total.errf += b.errf;
    total.other += b.other;
    for (int i = 0; i < 3; ++i) {
      total.hb[i] += b.hb[i];
      total.poll[i] += b.poll[i];
      total.wr[i] += b.wr[i];
    }
  }

  char buf[700];
  std::snprintf(buf, sizeof(buf),
                "%s通信小结：发送%lld帧、接收%lld帧；心跳请求/应答/超时=%lld/%lld/%lld；"
                "读角度请求/应答/超时=%lld/%lld/%lld；写位置请求/应答/超时=%lld/%lld/%lld；"
                "发送失败%lld次；错误帧%lld、非本机帧%lld。",
                final_one ? "收尾" : "", total.tx, total.rx, total.hb[0], total.hb[1], total.hb[2],
                total.poll[0], total.poll[1], total.poll[2], total.wr[0], total.wr[1], total.wr[2],
                total.fail, total.errf, total.other);
  log(buf);

  if (!final_one) {
    std::printf("已运行 %d 分钟\n", minute_no_);
    std::fflush(stdout);
  }
  // 双机时再给每台一行，避免只看汇总掩盖单台异常
  if (per.size() > 1) {
    std::string line;
    for (const auto& kv : per) {
      char one[200];
      std::snprintf(one, sizeof(one),
                    "%s电机%d：心跳%lld/%lld/%lld、读角度%lld/%lld/%lld、写位置%lld/%lld/%lld",
                    line.empty() ? "" : "；", kv.first, kv.second.hb[0], kv.second.hb[1],
                    kv.second.hb[2], kv.second.poll[0], kv.second.poll[1], kv.second.poll[2],
                    kv.second.wr[0], kv.second.wr[1], kv.second.wr[2]);
      line += one;
    }
    log(line + "。");
  }

  mj::Value e = mj::Value::object();
  e.set("minute", mj::Value::number(minute_no_));
  e.set("final", mj::Value::boolean(final_one));
  e.set("tx", mj::Value::number(static_cast<double>(total.tx)));
  e.set("rx", mj::Value::number(static_cast<double>(total.rx)));
  e.set("tx_fail", mj::Value::number(static_cast<double>(total.fail)));
  e.set("rx_err_frame", mj::Value::number(static_cast<double>(total.errf)));
  e.set("rx_other", mj::Value::number(static_cast<double>(total.other)));
  auto req_json = [](const long long v[3]) {
    mj::Value o = mj::Value::object();
    o.set("req", mj::Value::number(static_cast<double>(v[0])));
    o.set("ok", mj::Value::number(static_cast<double>(v[1])));
    o.set("timeout", mj::Value::number(static_cast<double>(v[2])));
    return o;
  };
  e.set("heartbeat", req_json(total.hb));
  e.set("read_angle", req_json(total.poll));
  e.set("write_pos", req_json(total.wr));
  mj::Value nodes = mj::Value::object();
  for (const auto& kv : per) {
    mj::Value n = mj::Value::object();
    n.set("heartbeat", req_json(kv.second.hb));
    n.set("read_angle", req_json(kv.second.poll));
    n.set("write_pos", req_json(kv.second.wr));
    nodes.set(std::to_string(kv.first), n);
  }
  e.set("motors", nodes);
  event("minute_summary", e);
}

CyclePhase Runtime::stat_for(int mid, int phase) const {
  CyclePhase out;
  auto it = cy_stats_.find(mid);
  if (it == cy_stats_.end()) return out;
  const PhaseStat& d = it->second[phase];
  out.samples_pos = static_cast<int>(d.pos.size());
  if (!d.pos.empty()) {
    out.has_pos = true;
    out.pos_start = d.pos.front();
    out.pos_end = d.pos.back();
  }
  if (!d.speed.empty()) {
    out.has_speed = true;
    out.speed_min = *std::min_element(d.speed.begin(), d.speed.end());
    out.speed_max = *std::max_element(d.speed.begin(), d.speed.end());
  }
  return out;
}

void Runtime::record_cycle(int no) {
  if (cy_stats_.empty()) return;
  CycleRec rec;
  rec.cycle = no;
  for (int mid : motors_) {
    std::array<CyclePhase, 3> arr;
    for (int pi = 0; pi < 3; ++pi) arr[pi] = stat_for(mid, pi);
    rec.per_motor[mid] = arr;
  }
  cycles_.push_back(rec);
  if (no % 10 == 0) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "进度：已完成 %d 组，累计 %.1f 秒。", no, no * cfg_.cycle_s);
    log(buf);
  }
}

void Runtime::run() {
  ph_stats_.clear();
  cy_stats_.clear();
  for (int mid : motors_) {
    ph_stats_[mid] = std::array<PhaseStat, 3>();
    cy_stats_[mid] = std::array<PhaseStat, 3>();
  }
  double base = now_s();
  last_minute_ = base;
  for (int mid : motors_) {
    due_[std::make_pair(mid, static_cast<int>(kHb))] = base;
    due_[std::make_pair(mid, static_cast<int>(kPoll))] = base + 0.01;
    if (!cfg_.read_only) due_[std::make_pair(mid, static_cast<int>(kWrite))] = base + 0.02;
  }

  int total_cycles =
      std::max(1, static_cast<int>(std::ceil(cfg_.minutes * 60.0 / cfg_.cycle_s)));
  {
    char buf[400];
    if (cfg_.read_only) {
      std::snprintf(buf, sizeof(buf), "计划：%d 台电机，只读模式（不使能、不松闸、不写位置），"
                                      "按 %d 组计时（约 %.1f 分钟）。",
                    static_cast<int>(motors_.size()), total_cycles,
                    total_cycles * cfg_.cycle_s / 60.0);
    } else {
      std::snprintf(buf, sizeof(buf), "计划：%d 台电机，%s，共 %d 组（约 %.1f 分钟）。",
                    static_cast<int>(motors_.size()), cfg_.plan_line().c_str(), total_cycles,
                    total_cycles * cfg_.cycle_s / 60.0);
    }
    log(buf);
  }
  if (cfg_.read_only) {
    std::printf("开始只读运行约 %.1f 分钟，%d 台电机（不使能、不松闸、不写位置）；每分钟报一次进度。\n"
                "日志：%s\n",
                total_cycles * cfg_.cycle_s / 60.0, static_cast<int>(motors_.size()),
                run_dir_.c_str());
  } else {
    std::printf("开始运行约 %.1f 分钟，%d 台电机；每分钟报一次进度。\n日志：%s\n",
                total_cycles * cfg_.cycle_s / 60.0, static_cast<int>(motors_.size()),
                run_dir_.c_str());
  }
  std::fflush(stdout);
  {
    char buf[320];
    if (cfg_.read_only) {
      std::snprintf(buf, sizeof(buf), "时序：心跳%dms、读角度%dms（窗口%dms），不发写位置。",
                    cfg_.hb_ms, cfg_.poll_ms, cfg_.read_ms);
    } else {
      std::snprintf(buf, sizeof(buf), "时序：轮询%dms、读超时%dms、写周期%dms（超时%dms）、心跳%dms。",
                    cfg_.poll_ms, cfg_.read_ms, cfg_.write_ms, cfg_.write_ms, cfg_.hb_ms);
    }
    log(buf);
  }

  while (true) {
    if (g_stop) throw Interrupted();
    double now = now_s();
    double elapsed = now - t_start_;
    int cyc = static_cast<int>(std::floor(elapsed / cfg_.cycle_s));
    if (cyc >= total_cycles) break;
    double ms = (elapsed - cyc * cfg_.cycle_s) * 1000.0;
    int ph = (ms < cfg_.fwd_ms) ? kFwd : ((ms < cfg_.fwd_ms + cfg_.hold_ms) ? kHold : kRev);

    if (cyc != cycle_of_) {
      if (cycle_of_ >= 0) record_cycle(cycle_of_ + 1);
      cycle_of_ = cyc;
      for (int mid : motors_) cy_stats_[mid] = std::array<PhaseStat, 3>();
    }
    if (ph != phase_) enter_phase(ph);

    bool found = false;
    double best_t = 0.0;
    int best_mid = 0;
    int best_task = 0;
    for (int mid : motors_) {
      for (int task = 0; task < 3; ++task) {
        auto it = due_.find(std::make_pair(mid, task));
        if (it == due_.end()) continue;
        double t = it->second;
        if (t <= now && (!found || t < best_t)) {
          found = true;
          best_t = t;
          best_mid = mid;
          best_task = task;
        }
      }
    }
    if (found) {
      do_task(best_mid, best_task);
      continue;
    }

    if (now - last_health_ >= 1.0) health();

    double sleep_until = last_health_ + 1.0;
    for (const auto& kv : due_) {
      if (kv.second < sleep_until) sleep_until = kv.second;
    }
    double wait = sleep_until - now_s();
    if (wait < 0.001) wait = 0.001;
    if (wait > 0.05) wait = 0.05;
    sleep_s(wait);
  }

  record_cycle((cycle_of_ < 0 ? 0 : cycle_of_) + 1);
  outcome_ = "completed";
}

std::map<int, StopInfo> Runtime::shutdown_motors() {
  std::map<int, StopInfo> info;
  for (int mid : motors_) {
    StopInfo si;
    int64_t ang = 0;
    bool has_ang = read_angle(mid, &ang);
    int64_t hold = has_ang ? ang : target_[mid];
    si.has_hold = true;
    si.hold_raw = hold;
    auto it = origin_.find(mid);
    int64_t park = (it != origin_.end()) ? it->second : hold;
    if (it != origin_.end()) {
      int64_t origin = it->second;
      request(mid, 0xA4, cmd_pos2(cfg_.speed_dps, origin), cfg_.write_s, nullptr, nullptr, nullptr);
      double t_lim = now_s() + 2.5;
      while (now_s() < t_lim) {
        int64_t a = 0;
        if (read_angle(mid, &a) && std::llabs(a - origin) <= 360) break;
        sleep_s(0.1);
      }
    }
    // 等电机停稳，再用一次同目标的位置指令取真实末次转速。
    double t_settle = now_s() + 1.2;
    bool has_prev = false;
    int64_t prev_ang = 0;
    while (now_s() < t_settle) {
      int64_t a = 0;
      if (read_angle(mid, &a)) {
        if (has_prev && std::llabs(a - prev_ang) <= 30) break;
        prev_ang = a;
        has_prev = true;
      }
      sleep_s(0.1);
    }
    {
      Frame fb{};
      if (request(mid, 0xA4, cmd_pos2(cfg_.speed_dps, park), cfg_.write_s, &fb, nullptr, nullptr)) {
        Feedback p = parse_fb(fb);
        mstate_[mid].speed = p.speed;
        mstate_[mid].iq = p.iq;
      }
    }
    int64_t final_ang = 0;
    bool has_final = read_angle(mid, &final_ang);
    request(mid, 0x81, cmd_simple(0x81), cfg_.read_budget, nullptr, nullptr, nullptr);
    sleep_s(0.15);
    request(mid, 0x8C, cmd_brake(0x00), cfg_.read_budget, nullptr, nullptr, nullptr);
    sleep_s(0.15);
    Frame data{};
    bool ok = request(mid, 0x8C, cmd_brake(0x10), cfg_.read_budget, &data, nullptr, nullptr);
    if (ok) si.brake = (data[1] == 0x00) ? "抱死" : "释放";
    request(mid, 0x9A, cmd_simple(0x9A), cfg_.read_budget, nullptr, nullptr, nullptr);
    si.brake_ok = (si.brake == "抱死");
    si.speed_dps = mstate_[mid].speed;
    if (has_final) {
      si.has_final = true;
      si.final_raw = final_ang;
    }
    if (has_final && it != origin_.end()) {
      si.has_return_err = true;
      si.return_err_raw = final_ang - it->second;
    }

    mj::Value e = mj::Value::object();
    e.set("motor", mj::Value::number(mid));
    e.set("brake", si.brake.empty() ? mj::Value() : mj::Value::string(si.brake));
    e.set("speed_dps", mj::Value::number(si.speed_dps));
    e.set("final_angle_raw", si.has_final ? mj::Value::number((double)si.final_raw) : mj::Value());
    e.set("return_error_raw",
          si.has_return_err ? mj::Value::number((double)si.return_err_raw) : mj::Value());
    event("stop_confirm", e);

    char buf[240];
    std::snprintf(buf, sizeof(buf), "电机 %d：已停止，抱闸 %s，末次反馈转速 %d dps，末角度 %.2f°（输出轴）。",
                  mid, si.brake.empty() ? "未知" : si.brake.c_str(), si.speed_dps,
                  si.has_final ? deg_out(si.final_raw) : 0.0);
    log(buf);
    info[mid] = si;
  }
  return info;
}

}  // namespace mvd
