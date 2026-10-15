#pragma once
// 本机 SocketCAN（can0/can1…）上的 CANopen 传输层，替换原 USB-CAN 适配器。

#include "yz_aim.h"

#include <cstdint>
#include <string>
#include <vector>

namespace yz_aim {

// 直接用内核 CAN 控制器收发经典 11 位帧；打开时只做只读自检，不发任何帧。
class SocketCanTransport final : public Transport {
public:
    SocketCanTransport(std::string interface_name, uint8_t node);
    ~SocketCanTransport() override;

    SocketCanTransport(const SocketCanTransport&) = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;

    // 打开接口并只接收本节点相关报文（SDO 应答、TPDO、心跳）。
    bool open(std::string& error);

    bool send(const Frame& frame) override;
    bool receive(Frame& frame, Deadline deadline) override;
    bool failed() const override { return fatal_; }
    bool accept_sdo_replies(uint8_t primary, uint8_t alternate) override;
    // Exposed for offline verification of the exact IDs installed into CAN_RAW_FILTER.
    static std::vector<uint16_t> receive_ids(uint8_t primary, uint8_t alternate = 0);

    bool interface_up() const { return interface_up_; }
    uint32_t reported_bitrate() const { return reported_bitrate_; }
    int controller_state() const { return controller_state_; }
    const std::string& interface_name() const { return interface_; }

private:
    void mark_fatal(const char* operation);

    std::string interface_;
    uint8_t node_ = 1;
    int descriptor_ = -1;
    bool fatal_ = false;
    bool interface_up_ = false;
    uint32_t reported_bitrate_ = 0;
    int controller_state_ = -1;
};

} // namespace yz_aim
