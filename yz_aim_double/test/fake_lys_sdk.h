#pragma once
#include <cstdint>

struct FakeLysStats {
    int opens = 0, inits = 0, starts = 0, stops = 0, closes = 0, transmits = 0, receives = 0;
    int receive_count_polls = 0, receive_wait_ms = 0;
    uint32_t bitrate = 0, timing = 0, device = 0, channel = 0;
    uint8_t fd_enabled = 0, mode = 0, transmit_length = 0;
    uint8_t transmit_extended = 0, transmit_remote = 0, transmit_send_type = 0;
};
extern "C" FakeLysStats FakeGetStats();
extern "C" void FakeReset();
extern "C" void FakeSetFailure(int mode); // 1=open 2=init 3=start 4=receive 5=send
extern "C" void FakeSetReceiveMode(int mode); // 0 normal 1 extended 2 remote 3 self-echo 4 oversized
