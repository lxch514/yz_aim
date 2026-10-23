#pragma once
#include "yz_aim.h"
namespace yz_aim {
class FrameTrace;
std::unique_ptr<Transport> open_lys_can(const std::string& library_path,
    uint32_t bitrate, std::string& error, uint32_t device=0, uint32_t channel=0,
    uint32_t nominal_timing=0, FrameTrace* trace=nullptr);
}
