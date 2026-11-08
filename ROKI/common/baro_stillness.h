#pragma once

#include <math.h>
#include <stdint.h>

// Long-window barometric trend used only to authorize a zero-velocity update.
// A single pressure sample cannot distinguish rest from slow uniform motion.
// Keep the short-term ZUPT gates too: this is an additional, conservative check.
class BaroStillness {
 public:
  static constexpr uint8_t kSamples = 80;  // ~4 s at the 20 Hz barometer rate.
  static constexpr float kMaximumDriftMps = 0.035f;

  void reset() {
    next_ = count_ = 0;
    ready_ = false;
    slope_mps_ = 0.f;
  }

  void add(float height_m, uint32_t now_us) {
    if (!isfinite(height_m)) {
      reset();
      return;
    }
    height_[next_] = height_m;
    stamp_us_[next_] = now_us;
    next_ = (uint8_t)((next_ + 1) % kSamples);
    if (count_ < kSamples) ++count_;
    if (count_ < kSamples) {
      ready_ = false;
      return;
    }

    // For a full ring, next_ addresses its oldest entry.
    const uint32_t origin_us = stamp_us_[next_];
    if ((uint32_t)(now_us - origin_us) < 3500000UL) {
      ready_ = false;
      return;
    }
    double sum_t = 0, sum_y = 0, sum_tt = 0, sum_ty = 0;
    for (uint8_t i = 0; i < kSamples; ++i) {
      const uint8_t index = (uint8_t)((next_ + i) % kSamples);
      const double t = (uint32_t)(stamp_us_[index] - origin_us) * 1e-6;
      const double y = height_[index];
      sum_t += t;
      sum_y += y;
      sum_tt += t * t;
      sum_ty += t * y;
    }
    const double denominator = kSamples * sum_tt - sum_t * sum_t;
    if (denominator < 1e-6) {
      ready_ = false;
      return;
    }
    slope_mps_ = (float)((kSamples * sum_ty - sum_t * sum_y) / denominator);
    ready_ = isfinite(slope_mps_);
  }

  bool ready() const { return ready_; }
  bool stable() const { return ready_ && fabsf(slope_mps_) < kMaximumDriftMps; }
  float slopeMps() const { return slope_mps_; }

 private:
  float height_[kSamples]{};
  uint32_t stamp_us_[kSamples]{};
  uint8_t next_ = 0;
  uint8_t count_ = 0;
  bool ready_ = false;
  float slope_mps_ = 0.f;
};
