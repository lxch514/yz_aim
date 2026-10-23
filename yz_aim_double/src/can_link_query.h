#pragma once
// 通过 rtnetlink 只读查询本机 CAN 网卡的运行状态与内核报告的波特率。

#include <cstdint>
#include <string>

namespace yz_aim {

struct CanLinkInfo {
    bool found = false;    // 该名字的网卡存在
    bool up = false;       // IFF_UP，配合 can 驱动表示控制器已启动
    bool running = false;  // IFF_RUNNING，载波/总线监听正常
    uint32_t bitrate = 0;  // 内核报告的仲裁波特率；0 表示内核未提供（例如由设备树固定）
    std::string kind;      // 网卡类型，正常应为 "can"
    // 内核 CAN 控制器状态 IFLA_CAN_STATE：0=ERROR-ACTIVE、1=ERROR-WARNING、
    // 2=ERROR-PASSIVE、3=BUS-OFF、4=STOPPED、5=SLEEPING；-1 表示内核未提供。
    // 非 0 表示控制器已经因为错误计数进入异常状态，此时收发都不可信，必须先 down/up 复位。
    int state = -1;
};

// 把 controller_state 数值翻成中文短语；未知值返回 "未知"。
const char* can_state_text(int state);

// 只读查询接口信息；返回 false 表示查询本身失败（不是"网卡不存在"）。
bool query_can_link(const std::string& interface_name, CanLinkInfo& info, std::string& error);

} // namespace yz_aim
