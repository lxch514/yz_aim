#include "shared_can_bus.h"
#include <algorithm>

namespace yz_aim {
bool SharedCanBus::Port::accept_sdo_replies(uint8_t primary, uint8_t alternate) {
    if (primary != node_ || primary < 1 || primary > 127 || alternate > 127) return false;
    const auto& other = this == &bus_.first_ ? bus_.second_ : bus_.first_;
    if (alternate && alternate == other.node_) return false;
    alternate_ = alternate;
    return true;
}
bool SharedCanBus::route(const Frame& f) {
    for (Port* port : {&first_, &second_}) {
        if (!port->node_) continue;
        const bool sdo = f.id == 0x580 + port->node_ ||
            (port->alternate_ && f.id == 0x580 + port->alternate_);
        if (sdo) {
            // Never discard another axis's ACK, including B-before-A paired writes.
            if (port->sdo_.size() >= 4096) { overflow_ = true; return false; }
            port->sdo_.push_back(f);
            return true;
        }
        bool telemetry = false;
        for (uint16_t base : {0x180,0x280,0x380,0x480,0x700})
            telemetry = telemetry || f.id == base + port->node_;
        if (!telemetry) continue;
        // At most one latest PDO/heartbeat per COB-ID. They never obscure SDO ACKs.
        auto& queue = port->telemetry_;
        const auto it = std::find_if(queue.begin(),queue.end(),[&](const Frame& old){return old.id==f.id;});
        if (it != queue.end()) { *it = f; ++telemetry_coalesced_; }
        else queue.push_back(f);
        return true;
    }
    return true;
}
bool SharedCanBus::Port::receive(Frame& frame, Deadline deadline) {
    while (!failed()) {
        // SDO traffic is ordered per node and has priority over unsolicited PDOs.
        if (Clock::now() >= deadline) return false;
        if (!sdo_.empty()) { frame=sdo_.front(); sdo_.pop_front(); return true; }
        if (!telemetry_.empty()) { frame=telemetry_.front(); telemetry_.pop_front(); return true; }
        Frame incoming;
        if (!bus_.physical_.receive(incoming, deadline)) return false;
        if (!bus_.route(incoming)) return false;
    }
    return false;
}
} // namespace yz_aim
