// SimpleFOC sensor backed by the MA732/MA730 absolute angle over SPI, with a
// windowed velocity estimate (same idea as windowed_encoder.h): velocity is
// the angle change over the shortest window >= window_s holding min_counts
// LSBs, capped at max_window_s, window length taken from a slowly filtered
// speed so the sample choice does not bias the estimate.
//
// Used on the EPC91120 because the MA732's A/B outputs pick up spurious
// counts from the 100 kHz-class GaN switching edges while the SPI angle stays
// clean.
#pragma once
#include <SimpleFOC.h>
#include "MA730GQ.h"

class SpiAngleSensor : public Sensor {
 public:
  explicit SpiAngleSensor(MA730GQ* enc) : enc_(enc) {}

  float window_s     = 0.002f;
  float max_window_s = 0.04f;
  int   min_counts   = 8;                       // in 14-bit LSBs (0.38 mrad each)

  void init() override { Sensor::init(); }

  float getSensorAngle() override { return enc_->getAngleRadians(); }   // [0, 2pi)

  float getVelocity() override {
    const uint32_t now = _micros();
    const float a = getAngle();
    if (n_ == 0 || (uint32_t)(now - t_[head_]) >= kMinStepUs) push(a, now);
    const float quantum = _2PI / 16384.0f;
    float want = (fabsf(v_slow_) > 1e-3f) ? (float)min_counts * quantum / fabsf(v_slow_) : max_window_s;
    if (want < window_s) want = window_s;
    if (want > max_window_s) want = max_window_s;
    int best = -1;
    for (int k = 1; k < n_; k++) {
      const int i = (head_ - k + kCap) % kCap;
      const float age = (float)(uint32_t)(now - t_[i]) * 1e-6f;
      if (age < want) continue;
      best = i; break;
    }
    if (best < 0) {
      const int i = (head_ - (n_ - 1) + kCap) % kCap;
      const float age = (float)(uint32_t)(now - t_[i]) * 1e-6f;
      if (n_ < 2 || age < window_s) return v_slow_;
      best = i;
    }
    const float age = (float)(uint32_t)(now - t_[best]) * 1e-6f;
    const float v = (a - a_[best]) / age;
    v_slow_ += 0.002f * (v - v_slow_);
    return v;
  }

 private:
  MA730GQ* enc_;
  static const int      kCap = 256;
  static const uint32_t kMinStepUs = 250;
  float    a_[kCap];
  uint32_t t_[kCap];
  int head_ = 0, n_ = 0;
  float v_slow_ = 0.0f;
  void push(float a, uint32_t t) { head_ = (head_ + 1) % kCap; a_[head_] = a; t_[head_] = t; if (n_ < kCap) n_++; }
};
