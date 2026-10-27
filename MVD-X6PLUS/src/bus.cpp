// SocketCAN 收发与控制器状态采集实现。
#include "mvd/bus.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>

namespace mvd {
namespace {

// 读取一条 shell 命令的全部标准输出。
std::string run_cmd(const std::string& cmd) {
  std::string out;
  FILE* fp = ::popen((cmd + " 2>/dev/null").c_str(), "r");
  if (fp == nullptr) return out;
  char buf[4096];
  while (std::fgets(buf, sizeof(buf), fp) != nullptr) out += buf;
  ::pclose(fp);
  return out;
}

long long num_field(const mj::Value& obj, const char* key) {
  const mj::Value* v = obj.find(key);
  if (v == nullptr || v->type() != mj::Value::NUM) return -1;
  return v->as_int(-1);
}

}  // namespace

double now_s() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;
}

mj::Value CanSnap::to_json() const {
  mj::Value o = mj::Value::object();
  if (!valid) {
    o.set("error", mj::Value::string(error));
    return o;
  }
  o.set("state", state.empty() ? mj::Value() : mj::Value::string(state));
  o.set("berr", berr);
  o.set("bittiming", bittiming);
  o.set("xstats", xstats);
  o.set("flags", [&] {
    mj::Value a = mj::Value::array();
    for (const auto& f : flags) a.push(mj::Value::string(f));
    return a;
  }());
  o.set("qdisc", qdisc.empty() ? mj::Value() : mj::Value::string(qdisc));
  o.set("txqlen", txqlen < 0 ? mj::Value() : mj::Value::number(static_cast<double>(txqlen)));
  o.set("operstate", operstate.empty() ? mj::Value() : mj::Value::string(operstate));
  mj::Value net = mj::Value::object();
  net.set("rx", net_rx);
  net.set("tx", net_tx);
  o.set("net", net);
  return o;
}

long long CanSnap::counter(const char* dir, const char* field) const {
  const mj::Value& obj = (strcmp(dir, "rx") == 0) ? net_rx : net_tx;
  return num_field(obj, field);
}

CanSnap can_status(const std::string& iface) {
  CanSnap s;
  std::string text = run_cmd("ip -j -d -s link show " + iface);
  if (text.empty()) {
    s.error = "ip 命令无输出";
    return s;
  }
  bool ok = false;
  mj::Value root = mj::parse(text, &ok);
  if (!ok || !root.is_arr() || root.size() == 0) {
    s.error = "ip 输出无法解析";
    return s;
  }
  const mj::Value& d = root.items()[0];
  s.valid = true;

  const mj::Value* linkinfo = d.find("linkinfo");
  if (linkinfo != nullptr) {
    const mj::Value* data = linkinfo->find("info_data");
    if (data != nullptr) {
      if (const mj::Value* st = data->find("state")) s.state = st->as_str();
      if (const mj::Value* b = data->find("berr_counter")) s.berr = *b;
      if (const mj::Value* bt = data->find("bittiming")) s.bittiming = *bt;
    }
    if (const mj::Value* xs = linkinfo->find("info_xstats")) s.xstats = *xs;
  }
  if (const mj::Value* fl = d.find("flags")) {
    for (const auto& f : fl->items()) s.flags.push_back(f.as_str());
  }
  if (const mj::Value* q = d.find("qdisc")) s.qdisc = q->as_str();
  if (const mj::Value* op = d.find("operstate")) s.operstate = op->as_str();
  s.txqlen = num_field(d, "txqlen");

  const mj::Value* stats = d.find("stats64");
  if (stats != nullptr) {
    if (const mj::Value* rx = stats->find("rx")) s.net_rx = *rx;
    if (const mj::Value* tx = stats->find("tx")) s.net_tx = *tx;
  }
  return s;
}

CanBus::~CanBus() { close(); }

bool CanBus::open(const std::string& iface, std::string* err) {
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    if (err) *err = std::string("创建 CAN 套接字失败：") + strerror(errno);
    return false;
  }
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface.c_str());
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    if (err) *err = "找不到接口 " + iface + "：" + strerror(errno);
    close();
    return false;
  }
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    if (err) *err = "绑定接口 " + iface + " 失败：" + strerror(errno);
    close();
    return false;
  }
  // 打开总线错误帧上报，便于统计错误帧。
  can_err_mask_t mask = CAN_ERR_MASK;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof(mask));
  // 内核收帧时间用于排除旧应答；丢包计数反映本 socket 接收队列溢出。
  const int on = 1;
  if (::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) < 0 ||
      ::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL, &on, sizeof(on)) < 0) {
    if (err) *err = std::string("启用 CAN 接收诊断失败：") + strerror(errno);
    close();
    return false;
  }
  rx_dropped_ = 0;
  rx_queue_ms_ = 0.0;
  return true;
}

bool CanBus::send(uint32_t can_id, const Frame& data, int len, int* err_no, bool extended) {
  if (fd_ < 0) {
    if (err_no) *err_no = EBADF;
    return false;
  }
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = extended ? ((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                          : (can_id & CAN_SFF_MASK);
  frame.can_dlc = static_cast<__u8>(len < 0 ? 0 : (len > 8 ? 8 : len));
  for (int i = 0; i < 8; ++i) frame.data[i] = data[i];
  // 队列拥塞时立即记录 errno，避免阻塞电机调度或风机 50Hz 保活线程。
  ssize_t n = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    if (err_no) *err_no = (n < 0) ? errno : EIO;
    return false;
  }
  if (err_no) *err_no = 0;
  return true;
}

bool CanBus::recv(double timeout_s, double* ts, uint32_t* can_id, int* dlc, Frame* data) {
  if (fd_ < 0) return false;
  int ms = static_cast<int>(std::ceil(timeout_s * 1000.0));
  if (timeout_s > 0.0 && ms < 1) ms = 1;
  if (ms < 0) ms = 0;
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  int rc = ::poll(&pfd, 1, ms);
  if (rc <= 0 || !(pfd.revents & POLLIN)) return false;
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  struct iovec iov{&frame, sizeof(frame)};
  alignas(struct cmsghdr) char control[CMSG_SPACE(sizeof(struct timespec)) +
                                     CMSG_SPACE(sizeof(uint32_t))]{};
  struct msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);
  ssize_t n = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
  if (n != static_cast<ssize_t>(sizeof(frame))) return false;
  double received = now_s();
  rx_queue_ms_ = 0.0;
  for (struct cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level != SOL_SOCKET) continue;
    if (c->cmsg_type == SO_TIMESTAMPNS && c->cmsg_len >= CMSG_LEN(sizeof(struct timespec))) {
      struct timespec stamp{}, wall{};
      ::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
      ::clock_gettime(CLOCK_REALTIME, &wall);
      // 时间戳来自 CLOCK_REALTIME，按当前时钟差换算到调度所用的单调时钟。
      double age = static_cast<double>(wall.tv_sec - stamp.tv_sec) +
                   static_cast<double>(wall.tv_nsec - stamp.tv_nsec) / 1e9;
      if (age < 0.0) age = 0.0;
      rx_queue_ms_ = age * 1000.0;
      received = now_s() - age;
    } else if (c->cmsg_type == SO_RXQ_OVFL && c->cmsg_len >= CMSG_LEN(sizeof(uint32_t))) {
      ::memcpy(&rx_dropped_, CMSG_DATA(c), sizeof(rx_dropped_));
    }
  }
  if (ts) *ts = received;
  if (can_id) *can_id = frame.can_id;
  if (dlc) *dlc = frame.can_dlc;
  if (data) {
    for (int i = 0; i < 8; ++i) (*data)[i] = frame.data[i];
  }
  return true;
}

void CanBus::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace mvd
