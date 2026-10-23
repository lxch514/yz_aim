#pragma once
#include "yz_aim.h"
#include <algorithm>
#include <cmath>

namespace yz_aim {

struct SyncFeedback {
    bool usable = false;
    int64_t position_a = 0, position_b = 0;
    double sample_skew_ms = 0;
};

inline SyncFeedback aligned_feedback(const Snapshot& a, const Snapshot& b, Deadline now,
                                     double acceleration_a = 0, double acceleration_b = 0,
                                     double speed_limit = kMaxMotorRpm) {
    SyncFeedback f;
    f.position_a = a.position_pulses;
    f.position_b = b.position_pulses;
    if (a.position_sample.valid && b.position_sample.valid)
        f.sample_skew_ms = std::abs(std::chrono::duration<double, std::milli>(
            a.position_sample.sampled_at-b.position_sample.sampled_at).count());
    if (!a.position_sample.valid || !b.position_sample.valid ||
        !a.speed_sample.valid || !b.speed_sample.valid ||
        a.position_sample.retried || b.position_sample.retried ||
        a.speed_sample.retried || b.speed_sample.retried ||
        !std::isfinite(a.speed_rpm) || !std::isfinite(b.speed_rpm)) return f;
    const auto ta = a.position_sample.sampled_at, tb = b.position_sample.sampled_at;
    const auto latest = std::max(ta, tb);
    f.sample_skew_ms = std::abs(std::chrono::duration<double, std::milli>(ta - tb).count());
    for (auto t : {ta, tb, a.speed_sample.sampled_at, b.speed_sample.sampled_at})
        if (t > now || now - t > std::chrono::milliseconds(500)) return f;
    if (f.sample_skew_ms > 200) return f;
    const double k = static_cast<double>(kMotorPulsesPerRevolution) / 60.0;
    auto displacement = [&](const Snapshot& sample, double acceleration) {
        acceleration = std::clamp(acceleration, -500.0, 500.0); // configured ramp, not an arbitrary jump
        const double lag = std::chrono::duration<double>(sample.position_sample.sampled_at-
                                                       sample.speed_sample.sampled_at).count();
        const double seconds = std::chrono::duration<double>(latest-sample.position_sample.sampled_at).count();
        const double v = std::clamp(sample.speed_rpm+acceleration*lag, -speed_limit, speed_limit);
        const double end = v+acceleration*seconds;
        if (std::abs(end)>speed_limit && std::abs(acceleration)>0.01) {
            const double cap = end>0 ? speed_limit : -speed_limit;
            const double ramp = std::clamp((cap-v)/acceleration, 0.0, seconds);
            return k*(v*ramp+0.5*acceleration*ramp*ramp+cap*(seconds-ramp));
        }
        return k*(v*seconds+0.5*acceleration*seconds*seconds);
    };
    f.position_a += std::llround(displacement(a, acceleration_a));
    f.position_b += std::llround(displacement(b, acceleration_b));
    f.usable = true;
    return f;
}

struct SyncTrim {
    int32_t rpm_a = 0, rpm_b = 0;
    int64_t error_pulses = 0; // normalized extension coordinate B minus A
    double error_deg = 0;
};

// Proportional cross-axis trim, bounded to slowing the leading axis only.
// Gains are deliberately explicit; hardware tuning/acceptance is still required.
class VelocitySynchronizer {
public:
    void reset(int32_t zero_a, int32_t zero_b, int sign_a, int32_t nominal,
               double maximum_trim_rpm) {
        zero_a_ = zero_a; zero_b_ = zero_b; sign_a_ = sign_a;
        nominal_ = nominal;
        cap_ = std::min(maximum_trim_rpm, std::abs(nominal) * 0.10);
        trim_a_ = trim_b_ = 0;
        max_error_ = 0;
    }
    SyncTrim update(const SyncFeedback& f, double dt_seconds, bool settled) {
        SyncTrim out;
        const int64_t xa = sign_a_ * (f.position_a - zero_a_);
        const int64_t xb = -sign_a_ * (f.position_b - zero_b_);
        out.error_pulses = xb - xa;
        out.error_deg = static_cast<double>(out.error_pulses) * 360.0 / kOutputPulsesPerRevolution;
        max_error_ = std::max(max_error_, std::abs(out.error_deg));
        const int direction = nominal_ > 0 ? 1 : -1;
        const double lead = direction * out.error_deg;
        const double deadband = 0.02; // output degrees, below the unchanged 0.5-degree guard
        const double desired_a = settled ? std::clamp((-lead - deadband) * 10.0, 0.0, cap_) : 0.0;
        const double desired_b = settled ? std::clamp(( lead - deadband) * 10.0, 0.0, cap_) : 0.0;
        const double step = 20.0 * std::clamp(dt_seconds, 0.0, 0.25); // RPM/s slew bound
        auto slew = [step](double current, double target) {
            return current + std::clamp(target - current, -step, step);
        };
        trim_a_ = slew(trim_a_, desired_a);
        trim_b_ = slew(trim_b_, desired_b);
        const int magnitude = std::abs(nominal_);
        const int ma = std::clamp(static_cast<int>(std::lround(magnitude - trim_a_)), 1, magnitude);
        const int mb = std::clamp(static_cast<int>(std::lround(magnitude - trim_b_)), 1, magnitude);
        out.rpm_a = sign_a_ * direction * ma;
        out.rpm_b = -sign_a_ * direction * mb;
        return out;
    }
    double max_error_degrees() const { return max_error_; }
private:
    int64_t zero_a_ = 0, zero_b_ = 0;
    int sign_a_ = -1;
    int32_t nominal_ = 1;
    double cap_ = 0, trim_a_ = 0, trim_b_ = 0, max_error_ = 0;
};
} // namespace yz_aim
