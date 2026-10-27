// Hobbywing X6 PLUS 推进电调编解码与保活实现。
#include "mvd/fan.h"

#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <utility>

namespace mvd {
namespace {

constexpr int kTxLogEvery = 250;    // 50Hz 下约每 5 秒记一条发送样本
constexpr double kDupWindowS = 2.0; // 完全重复判定窗口

void sleep_ms(int ms) {
  if (ms <= 0) return;
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = static_cast<long>(ms % 1000) * 1000000L;
  ::nanosleep(&ts, nullptr);
}

uint16_t le_u16(const Frame& d, int off) {
  return static_cast<uint16_t>(d[off] | (d[off + 1] << 8));
}

}  // namespace

uint32_t hw_message_id(uint8_t priority, uint16_t data_type_id, uint8_t source_node) {
  return (static_cast<uint32_t>(priority) & 0x1FU) << 24 |
         static_cast<uint32_t>(data_type_id) << 8 |
         (static_cast<uint32_t>(source_node) & 0x7FU);
}

uint32_t hw_service_request_id(uint8_t priority, uint8_t service_type, uint8_t dest,
                               uint8_t source) {
  return (static_cast<uint32_t>(priority) & 0x1FU) << 24 |
         static_cast<uint32_t>(service_type) << 16 |
         (1U << 15) |
         (static_cast<uint32_t>(dest) & 0x7FU) << 8 |
         (1U << 7) |
         (static_cast<uint32_t>(source) & 0x7FU);
}

void hw_pack4_throttle(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3, Frame* out) {
  const uint16_t values[4] = {
      static_cast<uint16_t>(ch0 & 0x3FFFU),
      static_cast<uint16_t>(ch1 & 0x3FFFU),
      static_cast<uint16_t>(ch2 & 0x3FFFU),
      static_cast<uint16_t>(ch3 & 0x3FFFU),
  };
  int bits[56];
  int k = 0;
  for (int ch = 0; ch < 4; ++ch) {
    const uint8_t low = static_cast<uint8_t>(values[ch] & 0xFFU);
    const uint8_t high = static_cast<uint8_t>((values[ch] >> 8U) & 0x3FU);
    for (int b = 7; b >= 0; --b) bits[k++] = (low >> b) & 1;
    for (int b = 5; b >= 0; --b) bits[k++] = (high >> b) & 1;
  }
  *out = Frame{};
  for (int i = 0; i < 56; ++i) {
    (*out)[i / 8] = static_cast<uint8_t>(((*out)[i / 8] << 1) | bits[i]);
  }
}

std::array<uint16_t, 4> hw_unpack4_throttle(const Frame& f) {
  std::array<uint16_t, 4> out{{0, 0, 0, 0}};
  int bits[56];
  int k = 0;
  for (int i = 0; i < 7; ++i) {
    for (int b = 7; b >= 0; --b) bits[k++] = (f[i] >> b) & 1;
  }
  for (int ch = 0; ch < 4; ++ch) {
    int low = 0;
    int high = 0;
    for (int i = 0; i < 8; ++i) low = (low << 1) | bits[ch * 14 + i];
    for (int i = 0; i < 6; ++i) high = (high << 1) | bits[ch * 14 + 8 + i];
    out[static_cast<size_t>(ch)] = static_cast<uint16_t>(low | (high << 8));
  }
  return out;
}

namespace {

uint8_t tail_byte(uint8_t tid) { return static_cast<uint8_t>(0xC0 | (tid & 0x1F)); }

HwFrame make_ready(uint8_t tid, bool on) {
  HwFrame f;
  f.id = hw_message_id(kHwPriorityMedium, kHwMsgReadyControl, kHwSrcNode);
  f.data[0] = static_cast<uint8_t>(on ? kHwReadyOn : kHwReadyOff);
  f.data[1] = tail_byte(tid);
  f.len = 2;
  return f;
}

HwFrame make_select_can(uint8_t tid, uint8_t node) {
  HwFrame f;
  f.id = hw_service_request_id(kHwPriorityMedium, kHwSrvThrottleSelect, node, kHwSrcNode);
  f.data[0] = 0x00;
  f.data[1] = tail_byte(tid);
  f.len = 2;
  return f;
}

HwFrame make_raw(uint8_t tid, uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3) {
  HwFrame f;
  f.id = hw_message_id(kHwPriorityHighest, kHwMsgRawCommand, kHwSrcNode);
  hw_pack4_throttle(ch0, ch1, ch2, ch3, &f.data);
  f.data[7] = tail_byte(tid);
  f.len = 8;
  return f;
}

}  // namespace

FanLink::FanLink(CanBus* bus, FanCfg cfg) : bus_(bus), cfg_(std::move(cfg)) {
  for (size_t i = 0; i < cfg_.nodes.size(); ++i) {
    FanNode n;
    n.node = cfg_.nodes[i];
    n.channel = (i < cfg_.channels.size()) ? cfg_.channels[i] : static_cast<int>(i);
    stats_.nodes[n.node] = n;
  }
}

FanLink::~FanLink() {
  stop();
}

bool FanLink::send(HwFrame& f, int* err) {
  // 单帧传输：同一消息或同一目标服务有各自的 5 位 TID。
  f.data[f.len - 1] = tail_byte(tx_tids_[f.id]++);
  int e = EBADF;
  bool ok = (bus_ != nullptr) && bus_->send(f.id, f.data, f.len, &e, true);
  {
    std::lock_guard<std::mutex> lk(mu_);
    ++stats_.tx;
    if (!ok) ++stats_.tx_fail;
  }
  if (err) *err = e;
  if (!ok) {
    push_log("TX_FAIL", f.id, f.data, f.len,
             "errno=" + std::to_string(e) + " " + std::strerror(e));
  }
  return ok;
}

void FanLink::push_log(const std::string& dir, uint32_t id, const Frame& data, int len,
                       const std::string& note) {
  FanLogItem item;
  item.dir = dir;
  item.id = id;
  item.data = data;
  item.len = len;
  item.note = note;
  std::lock_guard<std::mutex> lk(mu_);
  if (pending_.size() < 4096) pending_.push_back(item);
}

void FanLink::log_control_sample(const std::string& dir, const HwFrame& f,
                                 const std::string& note) {
  push_log(dir, f.id, f.data, f.len, note);
}

std::vector<FanLogItem> FanLink::take_logs() {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<FanLogItem> out;
  out.swap(pending_);
  return out;
}

bool FanLink::init(std::string* err, const Wait& wait) {
  const auto pause = [&](int ms) { if (wait) wait(ms); else sleep_ms(ms); };
  if (!cfg_.control) {
    inited_ = true;
    return true;
  }
  if (bus_ == nullptr || !bus_->is_open()) {
    if (err) *err = "CAN 未打开，无法初始化推进风机";
    return false;
  }

  // 1) 逐台把油门来源切到 CAN 数字油门
  for (size_t i = 0; i < cfg_.nodes.size(); ++i) {
    HwFrame f = make_select_can(0, cfg_.nodes[i]);
    char note[64];
    std::snprintf(note, sizeof(note), "选 CAN 油门 node0x%02X", cfg_.nodes[i]);
    if (!send(f, nullptr)) {
      if (err) *err = "风机选 CAN 油门失败（发送阶段）";
      return false;
    }
    log_control_sample("TX", f, note);
    pause(100);
  }

  // 2) 广播油门使能
  {
    HwFrame f = make_ready(0, true);
    if (!send(f, nullptr)) {
      if (err) *err = "风机油门使能帧发送失败";
      return false;
    }
    log_control_sample("TX", f, "油门使能 0xAA");
    ready_sent_ = true;
    pause(100);
  }

  // 3) 连续零油门，让电调进入可接受油门的待命状态
  int frames = static_cast<int>(std::lround(cfg_.init_zero_sec * 1000.0 / cfg_.period_ms));
  if (frames < 1) frames = 1;
  for (int i = 0; i < frames; ++i) {
    HwFrame f = make_raw(0, 0, 0, 0, 0);
    if (!send(f, nullptr)) {
      if (err) *err = "风机零油门帧发送失败";
      return false;
    }
    if (i == 0 || i == frames - 1) {
      char note[80];
      std::snprintf(note, sizeof(note), "零油门 %d/%d", i + 1, frames);
      log_control_sample("TX", f, note);
    }
    pause(cfg_.period_ms);
  }

  inited_ = true;
  return true;
}

void FanLink::start() {
  if (!cfg_.control || run_) return;
  run_ = true;
  th_ = std::thread(&FanLink::worker, this);
}

void FanLink::stop() {
  if (!run_) return;
  run_ = false;
  if (th_.joinable()) th_.join();
}

bool FanLink::shutdown(const Wait& wait) {
  if (!cfg_.control || bus_ == nullptr || !bus_->is_open()) return false;
  const auto pause = [&](int ms) { if (wait) wait(ms); else sleep_ms(ms); };
  for (int i = 0; i < cfg_.stop_zero_frames; ++i) {
    HwFrame f = make_raw(0, 0, 0, 0, 0);
    bool sent = send(f, nullptr);
    if (sent && (i == 0 || i == cfg_.stop_zero_frames - 1)) {
      char note[80];
      std::snprintf(note, sizeof(note), "收尾零油门 %d/%d", i + 1, cfg_.stop_zero_frames);
      log_control_sample("TX", f, note);
    }
    pause(cfg_.period_ms);
  }
  HwFrame off = make_ready(0, false);
  bool sent = send(off, nullptr);
  if (sent) {
    ready_sent_ = false;
    log_control_sample("TX", off, "油门失能 0x55");
  }
  return sent;
}

void FanLink::worker() {
  const double period = cfg_.period_ms / 1000.0;
  double next = now_s();
  long long skip = 0;
  while (run_) {
    HwFrame f = make_raw(0, cfg_.throttle, cfg_.throttle, cfg_.throttle, cfg_.throttle);
    bool sent = send(f, nullptr);
    const double t1 = now_s();
    bool log_now = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (has_last_tx_) {
        const double gap = (t1 - last_tx_) * 1000.0;
        if (stats_.period_ms.size() < 400000) stats_.period_ms.push_back(gap);
        if (gap > cfg_.period_ms * 1.5) ++stats_.late;
      }
      last_tx_ = t1;
      has_last_tx_ = true;
      if (++skip >= kTxLogEvery) {
        skip = 0;
        log_now = true;
      }
    }
    if (log_now && sent) {
      char note[96];
      std::snprintf(note, sizeof(note), "RawCommand 油门 %u 每 %dms 保活",
                    static_cast<unsigned>(cfg_.throttle), cfg_.period_ms);
      push_log("TX", f.id, f.data, f.len, note);
    }

    next += period;
    double wait = next - now_s();
    if (wait <= 0.0) {
      // 落后了就重新对齐，不再补发历史帧
      next = now_s();
      wait = 0.0;
    }
    if (wait > 0.0) {
      struct timespec ts;
      ts.tv_sec = static_cast<time_t>(wait);
      ts.tv_nsec = static_cast<long>((wait - static_cast<double>(ts.tv_sec)) * 1e9);
      if (ts.tv_nsec < 1) ts.tv_nsec = 1;
      ::nanosleep(&ts, nullptr);
    }
  }
}

bool FanLink::on_frame(uint32_t can_id, const Frame& data, int dlc, double ts) {
  // 仅解析广播数据帧；服务帧和 RTR 不应当更新遥测在线状态。
  if (!(can_id & kCanEffFlag) || (can_id & (kCanErrFlag | 0x40000000U | 0x80U))) return false;
  const uint16_t type = static_cast<uint16_t>((can_id >> 8) & 0xFFFFU);
  int kind = -1;
  int want = 0;
  if (type == kHwMsgTelemetry1) {
    kind = 0;
    want = 7;
  } else if (type == kHwMsgTelemetry2) {
    kind = 1;
    want = 6;
  } else if (type == kHwMsgTelemetry3) {
    kind = 2;
    want = 8;
  } else {
    return false;
  }
  if (dlc != want) return false;
  if ((data[want - 1] & 0xE0U) != 0xC0U) return false;

  const uint8_t node = static_cast<uint8_t>(can_id & 0x7FU);
  const uint8_t tid = static_cast<uint8_t>(data[want - 1] & 0x1FU);

  std::lock_guard<std::mutex> lk(mu_);
  auto it = stats_.nodes.find(node);
  if (it == stats_.nodes.end()) {
    FanNode n;
    n.node = node;
    n.channel = -1;
    it = stats_.nodes.emplace(node, n).first;
  }
  FanNode& n = it->second;
  n.last_rx = ts;  // 重复帧也证明仍有报文到达；新数据新鲜度单独记录。

  const auto key = std::make_pair(node, kind);
  DupState& st = dup_[key];
  const bool same = st.has && st.tid == tid && st.data == data && (ts - st.ts) < kDupWindowS;
  if (same) {
    ++n.dup;
    st.ts = ts;
    ++stats_.telemetry;
    return true;
  }
  if (st.has) {
    const double gap = (ts - st.ts) * 1000.0;
    if (gap > n.max_gap_ms) n.max_gap_ms = gap;
  }
  st.has = true;
  st.tid = tid;
  st.data = data;
  st.ts = ts;

  ++stats_.telemetry;
  if (kind == 0) {
    ++n.msg1;
    n.has_rpm = true;
    n.rpm = le_u16(data, 0);
    n.has_thr = true;
    n.thr_raw = le_u16(data, 2);
    n.has_status = true;
    n.status = le_u16(data, 4);
  } else if (kind == 1) {
    ++n.msg2;
    n.has_volt = true;
    n.volt = le_u16(data, 0) / 10.0;
    n.has_cur = true;
    n.cur = le_u16(data, 2) / 10.0;
    n.has_temp = true;
    n.temp = static_cast<double>(data[4]);
  } else {
    ++n.msg3;
    n.has_mos = true;
    n.mos_temp = data[0];
    n.has_cap = true;
    n.cap_temp = data[1];
    n.has_motor_temp = true;
    n.motor_temp = data[2];
  }
  n.last_unique_rx = ts;

  // 遥测按 1/200 抽样写帧记录，统计仍按全部帧
  ++rx_seen_;
  if (rx_seen_ <= 6 || rx_seen_ % 200 == 0) {
    if (pending_.size() < 4096) {
      char note[128];
      if (kind == 0) {
        std::snprintf(note, sizeof(note), "遥测1 node0x%02X 转速%d rpm 油门%d 状态0x%04X", node,
                      n.rpm, n.thr_raw, n.status);
      } else if (kind == 1) {
        std::snprintf(note, sizeof(note), "遥测2 node0x%02X 母线%.1fV %.1fA 温度%.0f℃", node,
                      n.volt, n.cur, n.temp);
      } else {
        std::snprintf(note, sizeof(note), "遥测3 node0x%02X MOS%d℃ 电容%d℃ 电机%d℃", node,
                      n.mos_temp, n.cap_temp, n.motor_temp);
      }
      FanLogItem item;
      item.dir = "RX";
      item.id = can_id & 0x1FFFFFFFU;
      item.data = data;
      item.len = dlc;
      item.note = note;
      pending_.push_back(item);
    }
  }
  return true;
}

FanStats FanLink::stats() const {
  std::lock_guard<std::mutex> lk(mu_);
  FanStats copy = stats_;
  return copy;
}

double FanLink::last_rx_age() const {
  std::lock_guard<std::mutex> lk(mu_);
  double newest = 0.0;
  for (const auto& kv : stats_.nodes) {
    if (kv.second.last_rx > newest) newest = kv.second.last_rx;
  }
  if (newest <= 0.0) return -1.0;
  return now_s() - newest;
}

}  // namespace mvd
