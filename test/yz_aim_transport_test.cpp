#include "yz_aim.h"
#include "fake_lys_sdk.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <memory>

namespace {
constexpr uint32_t kFakeDefaultDevice = 250;
constexpr uint32_t kFakeCustomTimingDevice = 251;
int checks = 0;
void require(bool value, const char* description) {
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", description); std::exit(1); }
}
}
int main(int argc, char** argv) {
    require(argc == 2, "fake SDK argument");
    void* sdk = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    require(sdk != nullptr, "load fake SDK without USB hardware");
    auto fn = [&](const char* name) { return dlsym(sdk, name); };
    auto reset = reinterpret_cast<void (*)()>(fn("FakeReset"));
    auto fail = reinterpret_cast<void (*)(int)>(fn("FakeSetFailure"));
    auto receive_mode = reinterpret_cast<void (*)(int)>(fn("FakeSetReceiveMode"));
    auto stats = reinterpret_cast<FakeLysStats (*)()>(fn("FakeGetStats"));
    require(reset && fail && receive_mode && stats, "fake control symbols available");
    std::string reason;
    reset();
    require(!yz_aim::open_lys_can(argv[1], 500000, reason, kFakeDefaultDevice) &&
            reason.find("timing") != std::string::npos && stats().opens == 0,
            "no invented timing recipe for 500K");
    {
        auto device = yz_aim::open_lys_can(argv[1], 1000000, reason, kFakeDefaultDevice);
        if (!device) std::fprintf(stderr, "classic CAN adapter open failed: %s\n", reason.c_str());
        require(device != nullptr && reason.empty(), "classic CAN adapter opened");
        const auto initial = stats();
        require(initial.opens == 1 && initial.inits == 1 && initial.starts == 1 &&
                initial.fd_enabled == 0 && initial.mode == 0 && initial.bitrate == 1000000 &&
                initial.timing == 0x01400106 && initial.device == kFakeDefaultDevice && initial.channel == 0,
                "actual open/init/start and classic CAN configuration");
        std::string ownership_error;
        require(!yz_aim::open_lys_can(argv[1], 1000000, ownership_error, kFakeDefaultDevice) &&
                ownership_error.find("owned") != std::string::npos && stats().opens == 1,
                "single adapter owner enforced before second SDK open");
        yz_aim::Frame invalid{0x801, 8, {}};
        require(!device->send(invalid) && stats().transmits == 0, "reject extended CAN ID before SDK");
        yz_aim::Frame request{0x601, 8, {0x40, 0x6c, 0x60}};
        yz_aim::Frame reply;
        const int receives_before_empty = stats().receives;
        require(!device->receive(reply, yz_aim::Clock::now() + std::chrono::milliseconds(4)) &&
                stats().receive_count_polls > 0 && stats().receives == receives_before_empty &&
                !device->failed(), "empty SDK queue does not call blocking Receive");
        require(device->send(request), "transmit classic CAN");
        auto actual = stats();
        require(actual.transmits == 1 && actual.transmit_length == 8 &&
                !actual.transmit_extended && !actual.transmit_remote && !actual.transmit_send_type,
                "SDK sees standard data frame with normal send mode");
        require(device->receive(reply, yz_aim::Clock::now() + std::chrono::milliseconds(40)) &&
                reply.id == 0x581 && reply.dlc == 8 && reply.data[4] == 0x9c &&
                stats().receive_wait_ms == 0,
                "classic SDK receive maps to CAN frame");
        for (int mode : {1, 2, 4}) {
            receive_mode(mode);
            require(device->send(request), "send before invalid frame");
            require(!device->receive(reply, yz_aim::Clock::now() + std::chrono::milliseconds(4)) &&
                    !device->failed(), "reject extended/rtr/oversize frame");
        }
        for (int mode : {3, 5, 6}) {
            receive_mode(mode);
            require(device->send(request), "send before SDK RX send_type frame");
            require(device->receive(reply, yz_aim::Clock::now() + std::chrono::milliseconds(40)) &&
                    reply.id == 0x581 && reply.dlc == 8,
                    "RX send_type is not a documented echo flag; accept 2/3/154");
        }
        receive_mode(0);
        fail(4);
        require(device->send(request), "queue fake frame for invalid SDK receive count");
        require(!device->receive(reply, yz_aim::Clock::now() + std::chrono::milliseconds(40)) &&
                device->failed(), "unexpected SDK receive count is fatal, not timeout");
    }
    require(stats().stops == 1 && stats().closes == 1, "normal Stop then Close");
    for (int phase = 1; phase <= 3; ++phase) {
        reset(); fail(phase);
        require(!yz_aim::open_lys_can(argv[1], 1000000, reason, kFakeDefaultDevice), "failed lifecycle returns no transport");
        const auto observed = stats();
        require(observed.opens == 1 && observed.closes == (phase == 1 ? 0 : 1) &&
                observed.stops == (phase == 3 ? 1 : 0), "lifecycle cleanup tracks actual phase");
    }
    reset();
    {
        auto adapter = yz_aim::open_lys_can(argv[1], 500000, reason, kFakeCustomTimingDevice, 1, 0x12345678);
        require(adapter != nullptr && stats().bitrate == 500000 && stats().timing == 0x12345678 &&
                stats().device == kFakeCustomTimingDevice && stats().channel == 1, "explicit verified custom timing passes through");
    }
    dlclose(sdk);
    std::printf("yz_aim_transport_test: %d checks passed (fake SDK, no USB)\n", checks);
}
