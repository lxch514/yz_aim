#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace yz_aim {
// The CAN worker never waits for disk writes. Only join after the motors have stopped.
class MotionLog {
public:
    MotionLog() = default;
    ~MotionLog();
    MotionLog(const MotionLog&) = delete;
    MotionLog& operator=(const MotionLog&) = delete;
    bool open(const std::filesystem::path& directory, const std::string& metadata, std::string& error);
    bool append(std::string row);
    void finish(const std::string& reason);
    bool failed() const { return failed_.load(); }
    const std::string& path() const { return path_; }
private:
    void run();
    std::string path_;
    std::ofstream stream_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> queue_;
    bool closing_ = false;
    std::atomic<bool> failed_{false};
};
} // namespace yz_aim
