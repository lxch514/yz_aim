#pragma once
// Internal ABI subset from /home/zakiu/lys-sdk/include/lyscan.h
// Source SHA-256 5ef84725b37dad7781f952fef33d74f5185d9dd59565e9ecf09b72c3339a791d
// Vendor SDK license unspecified; no binary or full SDK vendored here.
#include <cstddef>
#include <cstdint>

namespace yz_aim { namespace lys_abi {
using UINT = unsigned int;
using INT = int;
using UCHAR = unsigned char;
struct CanObject {
    UINT id, timestamp;
    UCHAR timestamp_flag, send_type, remote_flag, extended_flag, length;
    UCHAR reserved[3];
    UCHAR data[8];
};
struct BusParams {
    UINT clock_hz, bitrate_hz;
    UCHAR tseg1, tseg2, sjw, no_samp;
};
struct Params {
    BusParams nominal;
    UCHAR reserved0[4];
    BusParams data;
    UCHAR reserved1[8];
};
struct InitFdConfig {
    UINT arb_baud_hz, data_baud_hz, arb_timing, data_timing;
    UCHAR mode, fd_enable, iso_enable, reserved2;
    Params params;
};
static_assert(sizeof(UINT) == 4 && sizeof(CanObject) == 24, "LYS CAN ABI mismatch");
static_assert(offsetof(CanObject, data) == 16, "LYS CAN ABI offset mismatch");
static_assert(sizeof(InitFdConfig) == 56, "LYS FD configuration ABI mismatch");
constexpr UINT usbcANfdMini = 43;
constexpr UINT status_ok = 1;
} } // namespace yz_aim::lys_abi
