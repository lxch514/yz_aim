// SocketCAN 收发封装与 CAN 控制器状态采集。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mvd/json_mini.h"
#include "mvd/protocol.h"

namespace mvd {

// 一次 `ip -j -d -s link show` 的快照。
struct CanSnap {
  bool valid = false;
  std::string error;
  std::string state;
  mj::Value berr;       // {tx, rx}
  mj::Value bittiming;  // {bitrate, sample_point, ...}
  mj::Value xstats;     // 驱动私有计数
  mj::Value net_rx;     // stats64.rx
  mj::Value net_tx;     // stats64.tx
  std::vector<std::string> flags;
  std::string qdisc;
  std::string operstate;
  long long txqlen = -1;

  mj::Value to_json() const;
  // 取 stats64 里的一项计数，缺失返回 -1。
  long long counter(const char* dir, const char* field) const;
};

CanSnap can_status(const std::string& iface);

// can0 原始套接字。
class CanBus {
 public:
  ~CanBus();

  bool open(const std::string& iface, std::string* err);
  bool send(uint32_t can_id, const Frame& data, int len, int* err_no);
  bool recv(double timeout_s, double* ts, uint32_t* can_id, int* dlc, Frame* data);
  void close();
  bool is_open() const { return fd_ >= 0; }

 private:
  int fd_ = -1;
};

// 单调时钟秒数。
double now_s();

}  // namespace mvd
