#include "motion_log.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace yz_aim {
MotionLog::~MotionLog() { finish("logger closed"); }
bool MotionLog::open(const std::filesystem::path& directory, const std::string& metadata, std::string& error) {
    try {
        if (std::filesystem::weakly_canonical(directory) != std::filesystem::absolute(directory).lexically_normal()) {
            error = "refusing a redirected/symlink motion log directory";
            return false;
        }
        std::filesystem::create_directories(directory);
        const auto now = std::chrono::system_clock::now();
        const auto seconds = std::chrono::system_clock::to_time_t(now);
        std::tm local{};
        localtime_r(&seconds, &local);
        std::ostringstream filename;
        filename << "motion-" << std::put_time(&local, "%Y%m%d-%H%M%S") << "-" << ::getpid()
                 << "-" << std::chrono::steady_clock::now().time_since_epoch().count() << ".csv";
        path_ = (directory / filename.str()).string();
        stream_.open(path_, std::ios::out | std::ios::trunc);
        if (!stream_) { error = "cannot create motion CSV: " + path_; return false; }
        stream_ << metadata << '\n'
            << "elapsed_ms,event,node_a,node_b,target_a_rpm,target_b_rpm,actual_a_rpm,actual_b_rpm,"
               "raw_a,raw_b,aligned_a,aligned_b,travel_a,travel_b,error_pulses,error_output_deg,"
               "sample_skew_ms,age_a_ms,age_b_ms,age_speed_a_ms,age_speed_b_ms,"
               "estimated_accel_a,estimated_accel_b,quality,status_a,status_b,alarm_a,alarm_b\n";
        stream_.flush();
        if (!stream_) { error = "cannot write motion CSV header"; return false; }
        worker_ = std::thread(&MotionLog::run, this);
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}
bool MotionLog::append(std::string row) {
    if (failed_) return false;
    std::lock_guard<std::mutex> guard(mutex_);
    if (closing_ || queue_.size() >= 8192) { failed_ = true; return false; }
    queue_.push_back(std::move(row));
    ready_.notify_one();
    return true;
}
void MotionLog::run() {
    auto next_flush = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    for (;;) {
        std::string row;
        {
            std::unique_lock<std::mutex> guard(mutex_);
            ready_.wait_until(guard, next_flush, [&] { return closing_ || !queue_.empty(); });
            if (queue_.empty() && closing_) break;
            if (!queue_.empty()) { row = std::move(queue_.front()); queue_.pop_front(); }
        }
        if (!row.empty()) stream_ << row << '\n';
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_flush) { stream_.flush(); next_flush = now + std::chrono::seconds(1); }
        if (!stream_) failed_ = true;
    }
    stream_.flush();
    if (!stream_) failed_ = true;
}
void MotionLog::finish(const std::string& reason) {
    if (!worker_.joinable()) return;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        queue_.push_back("# end_reason=" + reason);
        closing_ = true;
        ready_.notify_one();
    }
    worker_.join();
    stream_.close();
}
} // namespace yz_aim
