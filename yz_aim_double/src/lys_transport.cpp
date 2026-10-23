#include "lys_transport.h"
#include "frame_trace.h"
#include <filesystem>
#include <cerrno>
#include "../sdk/lyscan_abi.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dlfcn.h>
#include <string>
#include <thread>

namespace yz_aim {
namespace {
using namespace lys_abi;

template<class Fn> bool symbol(void* handle, const char* name, Fn& dest) {
    static_assert(sizeof(Fn) == sizeof(void*), "function pointer cannot represent dlsym result");
    void* location = dlsym(handle, name);
    std::memcpy(&dest, &location, sizeof(location));
    return location != nullptr;
}

class LysTransport final : public Transport {
public:
    using Open = UINT (*)(UINT, UINT);
    using Close = UINT (*)(UINT, UINT);
    using Init = UINT (*)(UINT, UINT, UINT, InitFdConfig*);
    using StartStop = UINT (*)(UINT, UINT, UINT);
    using Send = UINT (*)(UINT, UINT, UINT, CanObject*, UINT);
    using Receive = UINT (*)(UINT, UINT, UINT, CanObject*, UINT, INT);
    using GetReceiveNum = UINT (*)(UINT, UINT, UINT, UCHAR);
    explicit LysTransport(FrameTrace* trace) : trace_(trace) {}
    ~LysTransport() override {
        if (opened && initialized && stop && stop(usbcANfdMini, device, channel) != status_ok)
            std::fprintf(stderr, "YZ-AIM LYS: LCAN_Stop cleanup failed\n");
        if (opened && close && close(usbcANfdMini, device) != status_ok)
            std::fprintf(stderr, "YZ-AIM LYS: LCAN_CloseDevice cleanup failed\n");
        if (library) dlclose(library);
        if (libusb) dlclose(libusb);
        if (lock_fd >= 0) ::close(lock_fd);
        if (legacy_lock_fd >= 0) ::close(legacy_lock_fd);
    }
    bool init(const std::string& path, uint32_t bitrate, uint32_t timing,
              uint32_t device_index, uint32_t channel_index, std::string& error) {
        device = device_index;
        channel = channel_index;
        // Own the USB adapter once per device across processes and channels.
        // New files stay inside this project. Honor an existing legacy lock read-only.
        const std::string legacy="/tmp/yz-aim-lys-device-"+std::to_string(device)+".lock";
        legacy_lock_fd=::open(legacy.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
        if ((legacy_lock_fd<0 && errno!=ENOENT) ||
            (legacy_lock_fd>=0 && flock(legacy_lock_fd,LOCK_EX|LOCK_NB)!=0)) {
            error="LYS USB device already owned or legacy lock inaccessible"; return false;
        }
        std::error_code ec;
        auto root=std::filesystem::read_symlink("/proc/self/exe",ec).parent_path();
        for(unsigned i=0;i<5 && !root.empty();++i) {
            if(std::filesystem::is_regular_file(root/"src/lys_transport.cpp") &&
               std::filesystem::is_regular_file(root/"build.sh")) break;
            root=root.parent_path();
        }
        if(ec || !std::filesystem::is_regular_file(root/"src/lys_transport.cpp")) {
            error="cannot locate USB project runtime directory"; return false;
        }
        std::filesystem::create_directories(root/".runtime",ec);
        if(ec) { error="cannot create project runtime directory: "+ec.message();return false; }
        const std::string lock_path=(root/".runtime"/("lys-device-"+std::to_string(device)+".lock")).string();
        lock_fd=::open(lock_path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
        if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0) {
            error="LYS USB device already owned or lock inaccessible: "+lock_path;return false;
        }
        // Vendor liblyscan.so has unresolved libusb symbols on target; resolve them first.
        libusb = dlopen("libusb-1.0.so.0", RTLD_NOW | RTLD_GLOBAL);
        if (!libusb) { error = std::string("load target libusb: ") + dlerror(); return false; }
        library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!library) { error = std::string("load liblyscan: ") + dlerror(); return false; }
        if (!symbol(library, "LCAN_OpenDevice", open) || !symbol(library, "LCAN_CloseDevice", close) ||
            !symbol(library, "LCAN_InitCANFD", init_can) || !symbol(library, "LCAN_Start", start) ||
            !symbol(library, "LCAN_Stop", stop) || !symbol(library, "LCAN_Transmit", transmit) ||
            !symbol(library, "LCAN_GetReceiveNum", get_receive_num) ||
            !symbol(library, "LCAN_Receive", receive_can)) {
            error = "liblyscan missing required classic CAN API";
            return false;
        }
        if (open(usbcANfdMini, device) != status_ok) { error = "LCAN_OpenDevice failed"; return false; }
        opened = true;
        InitFdConfig config{};
        config.arb_baud_hz = bitrate;
        config.arb_timing = timing;
        config.fd_enable = 0; // classic CAN; no CAN FD/BRS
        config.mode = 0;
        if (init_can(usbcANfdMini, device, channel, &config) != status_ok) {
            error = "LCAN_InitCANFD(fdEn=0) failed";
            return false;
        }
        initialized = true;
        if (start(usbcANfdMini, device, channel) != status_ok) { error = "LCAN_Start failed"; return false; }
        started = true;
        return true;
    }
    bool send(const Frame& frame) override {
        if (frame.id > 0x7ff || frame.dlc > 8 || fatal || !started) {
            if(trace_) trace_->frame("TX_ERR",frame);
            return false;
        }
        CanObject obj{};
        obj.id = frame.id;
        obj.length = frame.dlc;
        std::copy_n(frame.data.begin(), frame.dlc, obj.data);
        if (fatal || !started) return false;
        if (transmit(usbcANfdMini, device, channel, &obj, 1) != status_ok) {
            fatal = true;
            if(trace_) trace_->frame("TX_ERR",frame);
            return false;
        }
        if(trace_) trace_->frame("TX",frame);
        return true;
    }
    bool failed() const override { return fatal; }
    bool receive(Frame& frame, Deadline deadline) override {
        if (fatal || !started) return false;
        while (Clock::now() < deadline) {
            // Follow the vendor demo: check the classic-CAN queue before Receive.
            // Handle an empty queue with our own short, deadline-bounded wait.
            const UINT available=get_receive_num(usbcANfdMini, device, channel, UCHAR{0});
            if (available==UINT32_MAX) { fatal=true;return false; }
            if (available == 0) {
                std::this_thread::sleep_until(std::min(deadline,
                    Clock::now() + std::chrono::milliseconds(1)));
                continue;
            }
            CanObject obj{};
            const UINT received = receive_can(usbcANfdMini, device, channel, &obj, 1, 0);
            if (received == 0) continue;
            if (received != 1) { fatal = true; return false; }
            if (obj.extended_flag || obj.remote_flag || obj.id > 0x7ff || obj.length > 8)
                continue;
            frame = {};
            frame.id = static_cast<uint16_t>(obj.id);
            frame.dlc = obj.length;
            std::copy_n(obj.data, obj.length, frame.data.begin());
            if(trace_) trace_->frame("RX",frame);
            return true;
        }
        return false;
    }
private:
    FrameTrace* trace_ = nullptr;
    void* libusb = nullptr;
    void* library = nullptr;
    UINT device = 0, channel = 0;
    int lock_fd = -1, legacy_lock_fd = -1;
    bool opened = false, initialized = false, started = false, fatal = false;
    Open open = nullptr;
    Close close = nullptr;
    Init init_can = nullptr;
    StartStop start = nullptr, stop = nullptr;
    Send transmit = nullptr;
    Receive receive_can = nullptr;
    GetReceiveNum get_receive_num = nullptr;
};
} // namespace

std::unique_ptr<Transport> open_lys_can(const std::string& library_path,
                                        uint32_t bitrate, std::string& error,
                                        uint32_t device, uint32_t channel,
                                        uint32_t nominal_timing, FrameTrace* trace) {
    error.clear();
    if (library_path.empty()) { error = "explicit liblyscan.so path is required"; return nullptr; }
    if (bitrate != 125000 && bitrate != 250000 && bitrate != 500000 && bitrate != 1000000) {
        error = "select CAN bitrate 125/250/500/1000 kbit/s"; return nullptr;
    }
    if (!nominal_timing) {
        if (bitrate != 1000000) {
            error = "LYS timing for this CAN bitrate is undocumented; supply verified nominal_timing";
            return nullptr;
        }
        nominal_timing = 0x01400106; // vendor demo 1 Mbit/s nominal timing
    }
    auto candidate = std::make_unique<LysTransport>(trace);
    if (!candidate->init(library_path, bitrate, nominal_timing, device, channel, error)) return nullptr;
    return candidate;
}
} // namespace yz_aim
