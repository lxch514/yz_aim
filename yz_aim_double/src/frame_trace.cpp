#include "frame_trace.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <csignal>
#include <pthread.h>
#include <unistd.h>

namespace yz_aim {
FrameTrace::FrameTrace(std::string label, bool enabled, int output_fd)
    : label_(std::move(label)), enabled_(enabled), queue_(enabled ? 16384 : 0) {
    if (!enabled_) return;
    // Reopen instead of dup: setting O_NONBLOCK on a dup would alter stderr itself.
    const std::string path = "/proc/self/fd/" + std::to_string(output_fd);
    descriptor_ = ::open(path.c_str(), O_WRONLY | O_APPEND | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
    if (descriptor_ < 0) {
        enabled_ = false;
        std::fprintf(stderr, "[CAN TRACE] cannot open nonblocking diagnostic output: %s\n", std::strerror(errno));
        return;
    }
    worker_ = std::thread(&FrameTrace::run, this);
}
FrameTrace::~FrameTrace() {
    closing_ = true;
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
    if (descriptor_ >= 0) ::close(descriptor_);
}
void FrameTrace::frame(const char* direction, const Frame& f) {
    raw(direction, f.id, f.dlc, f.data.data());
}
void FrameTrace::raw(const char* direction, uint32_t id, uint8_t dlc, const uint8_t* data) {
    if (!enabled_) return;
    const auto sequence = ++sequence_;
    if (std::strcmp(direction, "TX") == 0) ++tx_;
    else if (std::strcmp(direction, "RX") == 0) ++rx_;
    else if (std::strcmp(direction, "TX_ERR") == 0) ++tx_error_;
    // Spin briefly, then wait: the record is never discarded just because the worker holds the lock.
    constexpr unsigned kSpinAttempts = 64;
    bool queued = false;
    {
        std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
        unsigned spins = 0;
        while (!lock.try_lock()) {
            if (++spins >= kSpinAttempts) { lock.lock(); break; }
            std::this_thread::yield();
        }
        if (size_ == queue_.size()) {
            ++trace_drop_;
        } else {
            Entry& entry = queue_[tail_];
            entry = {};
            entry.sequence = sequence;
            entry.at = Clock::now();
            entry.id = id;
            entry.dlc = std::min<uint8_t>(dlc, 8);
            std::snprintf(entry.direction, sizeof(entry.direction), "%s", direction);
            if (data) std::copy_n(data, entry.dlc, entry.data.begin());
            tail_ = (tail_ + 1) % queue_.size();
            ++size_;
            queued = true;
        }
    }
    if (queued) ready_.notify_one();
}
void FrameTrace::kernel_dropped(uint64_t count) { kernel_drop_ += count; }
FrameTrace::Counts FrameTrace::counts() const {
    return {tx_.load(), rx_.load(), tx_error_.load(), trace_drop_.load(), output_drop_.load(), kernel_drop_.load()};
}
void FrameTrace::output(const char* data, std::size_t size) {
    // One bounded nonblocking write; terminal/pipe congestion only loses diagnostics.
    const ssize_t written = ::write(descriptor_, data, size);
    if (written != static_cast<ssize_t>(size)) ++output_drop_;
}
void FrameTrace::summary() {
    const auto c = counts();
    char line[384];
    const int n = std::snprintf(line, sizeof(line),
        "[CAN STAT %s] TX=%llu RX=%llu TX_ERR=%llu KERNEL_RX_DROP=%llu TRACE_DROP=%llu OUTPUT_DROP=%llu\n",
        label_.c_str(), (unsigned long long)c.tx, (unsigned long long)c.rx,
        (unsigned long long)c.tx_error, (unsigned long long)c.kernel_drop,
        (unsigned long long)c.trace_drop, (unsigned long long)c.output_drop);
    output(line, std::min<std::size_t>(n > 0 ? n : 0, sizeof(line)-1));
}
void FrameTrace::run() {
    sigset_t blocked; sigemptyset(&blocked); sigaddset(&blocked,SIGPIPE);
    (void)pthread_sigmask(SIG_BLOCK,&blocked,nullptr);
    // Take a batch per lock so formatting and output never extend the critical section.
    constexpr std::size_t kBatch = 32;
    std::array<Entry, kBatch> batch{};
    auto next_summary = Clock::now() + std::chrono::seconds(1);
    for (;;) {
        std::size_t count = 0;
        bool stop = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait_until(lock, next_summary, [&] { return closing_ || size_ != 0; });
            while (size_ != 0 && count < kBatch) {
                batch[count++] = queue_[head_];
                head_ = (head_ + 1) % queue_.size();
                --size_;
            }
            stop = (count == 0 && closing_);
        }
        for (std::size_t i = 0; i < count; ++i) {
            const Entry& entry = batch[i];
            char line[256];
            const double elapsed = std::chrono::duration<double>(entry.at - epoch_).count();
            int n = std::snprintf(line, sizeof(line), "[CAN %s +%.6fs #%llu] %-8s ID=%03X DLC=%u",
                label_.c_str(), elapsed, (unsigned long long)entry.sequence,
                entry.direction, entry.id, entry.dlc);
            for (unsigned j=0; j<entry.dlc && n+4 < static_cast<int>(sizeof(line)); ++j)
                n += std::snprintf(line+n, sizeof(line)-n, " %02X", entry.data[j]);
            if (n > 0 && n+1 < static_cast<int>(sizeof(line))) { line[n++]='\n'; output(line,n); }
        }
        if (stop) break;
        if (Clock::now() >= next_summary) { summary(); next_summary = Clock::now()+std::chrono::seconds(1); }
    }
    summary();
}
} // namespace yz_aim
