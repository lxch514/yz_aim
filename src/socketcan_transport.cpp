#include "socketcan_transport.h"
#include "can_link_query.h"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace yz_aim {
namespace {

constexpr std::size_t kClassicFrameBytes = sizeof(struct can_frame);
// 内核在 CAN FD 网卡上按 canfd_frame 交付 FD 帧，这里统一用足够大的缓冲读取再按长度判定。
constexpr std::size_t kReceiveBufferBytes = sizeof(struct canfd_frame);

} // namespace

SocketCanTransport::SocketCanTransport(std::string interface_name, uint8_t node)
    : interface_(std::move(interface_name)), node_(node) {}

SocketCanTransport::~SocketCanTransport() {
    if (descriptor_ >= 0) ::close(descriptor_);
}

void SocketCanTransport::mark_fatal(const char* operation) {
    if (!fatal_)
        std::fprintf(stderr, "CAN 接口 %s 的 %s 操作失败：%s\n",
                     interface_.c_str(), operation, std::strerror(errno));
    fatal_ = true;
}

bool SocketCanTransport::open(std::string& error) {
    error.clear();
    CanLinkInfo link;
    std::string query_error;
    if (query_can_link(interface_, link, query_error)) {
        if (!link.found) {
            error = "CAN 接口不存在：" + interface_ + "（用 ip link 查看实际接口名）";
            return false;
        }
        interface_up_ = link.up;
        reported_bitrate_ = link.bitrate;
        controller_state_ = link.state;
    } else {
        std::fprintf(stderr, "提示：无法读取 %s 的链路状态（%s），继续尝试打开。\n",
                     interface_.c_str(), query_error.c_str());
    }
    if (!interface_up_) {
        error = "CAN 接口未启动：" + interface_ +
                "（先执行 ip link set " + interface_ + " up）";
        return false;
    }

    const unsigned index = ::if_nametoindex(interface_.c_str());
    if (index == 0) { error = "无法解析接口索引：" + interface_; return false; }

    descriptor_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (descriptor_ < 0) { error = std::string("创建 SocketCAN 套接字失败：") + std::strerror(errno); return false; }

    const int flags = ::fcntl(descriptor_, F_GETFL, 0);
    if (flags >= 0) (void)::fcntl(descriptor_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(descriptor_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        error = std::string("绑定 ") + interface_ + " 失败：" + std::strerror(errno);
        ::close(descriptor_);
        descriptor_ = -1;
        return false;
    }

    // 只接收本节点的 SDO 应答、TPDO 与心跳，其它节点的周期报文不进本会话队列。
    const can_filter filters[] = {
        {static_cast<canid_t>(0x180u + node_), CAN_SFF_MASK},
        {static_cast<canid_t>(0x280u + node_), CAN_SFF_MASK},
        {static_cast<canid_t>(0x380u + node_), CAN_SFF_MASK},
        {static_cast<canid_t>(0x480u + node_), CAN_SFF_MASK},
        {static_cast<canid_t>(0x580u + node_), CAN_SFF_MASK},
        {static_cast<canid_t>(0x700u + node_), CAN_SFF_MASK},
    };
    if (::setsockopt(descriptor_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
        error = std::string("设置 CAN 接收过滤失败：") + std::strerror(errno);
        ::close(descriptor_);
        descriptor_ = -1;
        return false;
    }
    // 不接收自己发出的帧，与 USB-CAN 适配器的行为一致。
    const int own_messages = 0;
    (void)::setsockopt(descriptor_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own_messages, sizeof(own_messages));
    return true;
}

bool SocketCanTransport::send(const Frame& frame) {
    if (descriptor_ < 0 || fatal_) return false;
    if (frame.id > 0x7ff || frame.dlc > 8) return false;
    struct can_frame classic {};
    classic.can_id = frame.id;
    classic.len = frame.dlc;
    for (unsigned i = 0; i < frame.dlc; ++i) classic.data[i] = frame.data[i];
    for (;;) {
        const ssize_t written = ::write(descriptor_, &classic, kClassicFrameBytes);
        if (written == static_cast<ssize_t>(kClassicFrameBytes)) return true;
        if (written < 0 && errno == EINTR) continue;
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)) {
            std::fprintf(stderr, "CAN %s 发送队列已满，本帧未发出。\n", interface_.c_str());
            fatal_ = true;
            return false;
        }
        mark_fatal("发送");
        return false;
    }
}

bool SocketCanTransport::receive(Frame& frame, Deadline deadline) {
    if (descriptor_ < 0 || fatal_) return false;
    for (;;) {
        const auto now = Clock::now();
        if (now >= deadline) return false;
        // 向下取整，保证本调用不会越过调用方给的截止时间。
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        int timeout_ms = static_cast<int>(remaining.count());
        if (timeout_ms < 0) timeout_ms = 0;

        pollfd descriptor{descriptor_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, timeout_ms);
        if (ready < 0) {
            if (errno == EINTR) return false;
            mark_fatal("等待接收");
            return false;
        }
        if (ready == 0) return false;
        if ((descriptor.revents & POLLNVAL) != 0) {
            std::fprintf(stderr, "CAN 接口 %s 的套接字已失效。\n", interface_.c_str());
            fatal_ = true;
            return false;
        }
        if ((descriptor.revents & (POLLIN | POLLERR | POLLHUP)) == 0) continue;

        std::array<uint8_t, kReceiveBufferBytes> buffer{};
        const ssize_t received = ::read(descriptor_, buffer.data(), buffer.size());
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return false;
            mark_fatal("接收");
            return false;
        }
        // 长度不等于经典帧即为 CAN FD 帧或空读，本工具只处理经典 CAN 2.0B。
        if (received != static_cast<ssize_t>(kClassicFrameBytes)) return false;

        struct can_frame classic {};
        std::memcpy(&classic, buffer.data(), kClassicFrameBytes);
        if ((classic.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) return false;
        if (classic.len > 8) return false;
        frame = {};
        frame.id = static_cast<uint16_t>(classic.can_id & CAN_SFF_MASK);
        frame.dlc = classic.len;
        for (unsigned i = 0; i < classic.len; ++i) frame.data[i] = classic.data[i];
        return true;
    }
}

} // namespace yz_aim
