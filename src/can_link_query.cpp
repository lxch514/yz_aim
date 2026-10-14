#include "can_link_query.h"

#include <cerrno>
#include <cstring>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace yz_aim {
namespace {

// 内核 ifi_flags 的位定义，与 <linux/if.h> 的 IFF_UP / IFF_RUNNING 一致。
constexpr unsigned kIffUp = 0x1u;
constexpr unsigned kIffRunning = 0x40u;

std::string errno_text(const char* prefix) {
    return std::string(prefix) + std::strerror(errno);
}

// 属性数据区首地址；不依赖 RTA_DATA 的返回类型，避免不同头文件版本差异。
const char* attribute_payload(const rtattr* attribute) {
    return reinterpret_cast<const char*>(attribute) + RTA_LENGTH(0);
}

// 取出 NUL 结尾的属性字符串，长度以属性自身长度为准。
bool attribute_string(const rtattr* attribute, std::string& out) {
    const int length = RTA_PAYLOAD(attribute);
    if (length <= 0) return false;
    const char* text = attribute_payload(attribute);
    const std::size_t bounded = strnlen(text, static_cast<std::size_t>(length));
    if (bounded == 0) return false;
    out.assign(text, bounded);
    return true;
}

// 解析 IFLA_LINKINFO：取出接口类型、IFLA_CAN_BITTIMING 与 IFLA_CAN_STATE。
void parse_link_info(const rtattr* attribute, CanLinkInfo& info) {
    int outer_length = RTA_PAYLOAD(attribute);
    for (const rtattr* outer = reinterpret_cast<const rtattr*>(attribute_payload(attribute));
         RTA_OK(outer, outer_length); outer = RTA_NEXT(outer, outer_length)) {
        if (outer->rta_type == IFLA_INFO_KIND) {
            attribute_string(outer, info.kind);
        } else if (outer->rta_type == IFLA_INFO_DATA) {
            int inner_length = RTA_PAYLOAD(outer);
            for (const rtattr* inner = reinterpret_cast<const rtattr*>(attribute_payload(outer));
                 RTA_OK(inner, inner_length); inner = RTA_NEXT(inner, inner_length)) {
                if (RTA_PAYLOAD(inner) < static_cast<int>(sizeof(uint32_t))) continue;
                uint32_t value = 0;
                std::memcpy(&value, attribute_payload(inner), sizeof(value));
                if (inner->rta_type == IFLA_CAN_BITTIMING) {
                    info.bitrate = value;
                } else if (inner->rta_type == IFLA_CAN_STATE) {
                    info.state = static_cast<int>(value);
                }
            }
        }
    }
}

// 解析一条 RTM_NEWLINK；命中目标名字时写入 info 并返回 true。
bool parse_new_link(nlmsghdr* message, const std::string& interface_name, CanLinkInfo& info) {
    const auto* link = static_cast<const ifinfomsg*>(NLMSG_DATA(message));
    int length = static_cast<int>(message->nlmsg_len) - NLMSG_LENGTH(sizeof(ifinfomsg));
    if (length <= 0) return false;

    CanLinkInfo candidate;
    candidate.up = (link->ifi_flags & kIffUp) != 0;
    candidate.running = (link->ifi_flags & kIffRunning) != 0;

    for (const rtattr* attribute = IFLA_RTA(link); RTA_OK(attribute, length);
         attribute = RTA_NEXT(attribute, length)) {
        if (attribute->rta_type == IFLA_IFNAME) {
            std::string name;
            if (attribute_string(attribute, name) && name == interface_name) candidate.found = true;
        } else if (attribute->rta_type == IFLA_LINKINFO) {
            parse_link_info(attribute, candidate);
        }
    }
    if (!candidate.found) return false;
    info = candidate;
    return true;
}

} // namespace

const char* can_state_text(int state) {
    switch (state) {
        case 0: return "ERROR-ACTIVE（正常）";
        case 1: return "ERROR-WARNING（错误计数偏高）";
        case 2: return "ERROR-PASSIVE（被动错误，收发不可信）";
        case 3: return "BUS-OFF（已离线，需 down/up 复位）";
        case 4: return "STOPPED（已停止）";
        case 5: return "SLEEPING（休眠）";
        default: return "未知";
    }
}

bool query_can_link(const std::string& interface_name, CanLinkInfo& info, std::string& error) {
    info = CanLinkInfo{};
    error.clear();
    if (interface_name.empty()) { error = "接口名为空"; return false; }

    const int descriptor = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (descriptor < 0) { error = errno_text("创建 netlink 套接字失败："); return false; }

    // 只读查询，不改任何配置；加超时避免链路异常时挂住。
    timeval receive_timeout{};
    receive_timeout.tv_sec = 2;
    (void)::setsockopt(descriptor, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));
    const int receive_buffer = 1 << 20;
    (void)::setsockopt(descriptor, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

    sockaddr_nl local{};
    local.nl_family = AF_NETLINK;
    if (::bind(descriptor, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        error = errno_text("绑定 netlink 套接字失败：");
        ::close(descriptor);
        return false;
    }

    struct {
        nlmsghdr header;
        ifinfomsg link;
    } request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
    request.header.nlmsg_seq = 1;
    request.link.ifi_family = AF_UNSPEC;

    if (::send(descriptor, &request, request.header.nlmsg_len, 0) < 0) {
        error = errno_text("发送 netlink 请求失败：");
        ::close(descriptor);
        return false;
    }

    bool complete = false;
    while (!complete) {
        char buffer[32768];
        const ssize_t received = ::recv(descriptor, buffer, sizeof(buffer), 0);
        if (received < 0) {
            if (errno == EINTR) continue;
            error = (errno == EAGAIN || errno == EWOULDBLOCK)
                ? std::string("netlink 查询超时")
                : errno_text("读取 netlink 应答失败：");
            ::close(descriptor);
            return false;
        }
        if (received == 0) break;
        int remaining = static_cast<int>(received);
        for (nlmsghdr* message = reinterpret_cast<nlmsghdr*>(buffer); NLMSG_OK(message, remaining);
             message = NLMSG_NEXT(message, remaining)) {
            if (message->nlmsg_type == NLMSG_DONE) { complete = true; break; }
            if (message->nlmsg_type == NLMSG_ERROR) {
                error = "netlink 返回错误应答";
                ::close(descriptor);
                return false;
            }
            if (message->nlmsg_type != RTM_NEWLINK) continue;
            if (parse_new_link(message, interface_name, info)) complete = true;
        }
    }
    ::close(descriptor);
    return true;
}

} // namespace yz_aim
