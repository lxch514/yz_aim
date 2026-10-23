#pragma once
#include "yz_aim.h"
#include <atomic>
#include <array>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace yz_aim {
// Diagnostic output never waits for the terminal on the CAN/control thread.
class FrameTrace {
public:
    struct Counts { uint64_t tx, rx, tx_error, trace_drop, output_drop, kernel_drop; };
    explicit FrameTrace(std::string label, bool enabled = true, int output_fd = 2);
    ~FrameTrace();
    void frame(const char* direction, const Frame& frame);
    void raw(const char* direction, uint32_t id, uint8_t dlc, const uint8_t* data);
    void kernel_dropped(uint64_t count);
    Counts counts() const;
private:
    struct Entry {
        uint64_t sequence = 0;
        Clock::time_point at{};
        uint32_t id = 0;
        uint8_t dlc = 0;
        char direction[12]{};
        std::array<uint8_t, 8> data{};
    };
    void run();
    void output(const char* data, std::size_t size);
    void summary();
    std::string label_;
    bool enabled_ = false;
    int descriptor_ = -1;
    Clock::time_point epoch_ = Clock::now();
    std::atomic<uint64_t> sequence_{0}, tx_{0}, rx_{0}, tx_error_{0}, trace_drop_{0}, output_drop_{0}, kernel_drop_{0};
    std::atomic<bool> closing_{false};
    std::vector<Entry> queue_;
    // Single CAN worker produces; the diagnostic worker consumes. No mutex contention loss.
    std::atomic<std::size_t> head_{0}, tail_{0};
    std::mutex mutex_;
    std::condition_variable ready_;
    std::thread worker_;
};
} // namespace yz_aim
