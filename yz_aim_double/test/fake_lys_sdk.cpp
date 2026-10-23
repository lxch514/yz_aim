#include "fake_lys_sdk.h"
#include "../sdk/lyscan_abi.h"
#include <cstring>

using yz_aim::lys_abi::UINT;
using yz_aim::lys_abi::INT;
using yz_aim::lys_abi::CanObject;
using yz_aim::lys_abi::InitFdConfig;

namespace {
FakeLysStats stats;
int failure = 0, receive_mode = 0;
CanObject queued{};
bool pending = false;
}
extern "C" FakeLysStats FakeGetStats() { return stats; }
extern "C" void FakeReset() { stats = {}; failure = 0; receive_mode = 0; pending = false; }
extern "C" void FakeSetFailure(int mode) { failure = mode; }
extern "C" void FakeSetReceiveMode(int mode) { receive_mode = mode; }
extern "C" UINT LCAN_OpenDevice(UINT dev_type, UINT device) {
    if (dev_type != 43) return 0;
    ++stats.opens;
    stats.device = device;
    return failure == 1 ? 0 : 1;
}
extern "C" UINT LCAN_CloseDevice(UINT, UINT) { ++stats.closes; return 1; }
extern "C" UINT LCAN_InitCANFD(UINT, UINT, UINT channel, InitFdConfig* config) {
    ++stats.inits;
    stats.bitrate = config->arb_baud_hz;
    stats.timing = config->arb_timing;
    stats.fd_enabled = config->fd_enable;
    stats.mode = config->mode;
    stats.channel = channel;
    return failure == 2 ? 0 : 1;
}
extern "C" UINT LCAN_Start(UINT, UINT, UINT) { ++stats.starts; return failure == 3 ? 0 : 1; }
extern "C" UINT LCAN_Stop(UINT, UINT, UINT) { ++stats.stops; return 1; }
extern "C" UINT LCAN_Transmit(UINT, UINT, UINT, CanObject* frame, UINT count) {
    ++stats.transmits;
    if (failure == 5 || count != 1) return 0;
    stats.transmit_length = frame->length;
    stats.transmit_extended = frame->extended_flag;
    stats.transmit_remote = frame->remote_flag;
    stats.transmit_send_type = frame->send_type;
    queued = {};
    queued.id = 0x581;
    queued.length = 8;
    queued.data[0] = 0x4b;
    queued.data[1] = 0x6c;
    queued.data[2] = 0x60;
    queued.data[3] = 0;
    queued.data[4] = 0x9c;
    queued.data[5] = 0xff;
    pending = true;
    return 1;
}
extern "C" UINT LCAN_GetReceiveNum(UINT, UINT, UINT, yz_aim::lys_abi::UCHAR type) {
    ++stats.receive_count_polls;
    if (failure==7) return UINT32_MAX;
    return type == 0 && pending ? 1 : 0;
}
extern "C" UINT LCAN_Receive(UINT, UINT, UINT, CanObject* frame, UINT len, INT wait_ms) {
    ++stats.receives;
    stats.receive_wait_ms = wait_ms;
    if (failure == 4) return 3; // cannot fit the requested buffer; adapter must report fault
    if (!pending || len != 1) return 0;
    pending = false;
    *frame = queued;
    if (receive_mode == 1) frame->extended_flag = 1;
    if (receive_mode == 2) frame->remote_flag = 1;
    if (receive_mode == 3) frame->send_type = 2;
    if (receive_mode == 5) frame->send_type = 3;
    if (receive_mode == 6) frame->send_type = 154;
    if (receive_mode == 4) frame->length = 9;
    return 1;
}
