// Quadrature encoder with a windowed velocity estimate.
//
// SimpleFOC's Encoder::getVelocity() times individual pulses, so at a few rpm
// the estimate carries every bit of edge jitter and the unequal spacing of the
// four quadrature edges, and the velocity loop turns that into torque noise.
// This variant returns (angle now - angle at the start of a window) / elapsed,
// where the window is the shortest span >= window_s that contains at least
// min_counts pulses (capped at max_window_s). At speed that is ~2 ms, so the
// estimate adds almost no delay; at a few rpm it stretches to tens of ms so the
// quantisation noise stays bounded. window_s = 0 falls back to the stock
// estimator (useful for A/B tests).
#pragma once
#include <SimpleFOC.h>

class WindowedEncoder : public Encoder {
 public:
  using Encoder::Encoder;

  float window_s     = 0.002f;  // minimum estimation window; 0 = stock SimpleFOC estimator
  float max_window_s = 0.04f;   // stretch limit at very low speed; longer windows add lag and the loop hunts (bench: 40 ms best at 1 rpm)
  int   min_counts   = 8;       // stretch until at least this many counts are inside the window

  float getVelocity() override {
    if (window_s <= 0.0f) return Encoder::getVelocity();
    const uint32_t now = _micros();
    const float a = getAngle();                       // valid: update() runs every loop before move()
    if (n_ == 0 || (uint32_t)(now - t_[head_]) >= kMinStepUs) push(a, now);

    // Window length from a slowly filtered speed (~0.1 s), so the choice of
    // sample depends neither on the counts in this window nor on the current
    // jerk: both selection effects bias the time-average high at low speed.
    const float quantum = _2PI / cpr;
    float want = (fabsf(v_slow_) > 1e-3f) ? (float)min_counts * quantum / fabsf(v_slow_) : max_window_s;
    if (want < window_s) want = window_s;
    if (want > max_window_s) want = max_window_s;
    int best = -1;
    for (int k = 1; k < n_; k++) {
      const int i = (head_ - k + kCap) % kCap;
      const float age = (float)(uint32_t)(now - t_[i]) * 1e-6f;
      if (age < want) continue;
      best = i;
      break;
    }
    if (best < 0) {                                    // buffer does not span the window yet
      const int i = (head_ - (n_ - 1) + kCap) % kCap;
      const float age = (float)(uint32_t)(now - t_[i]) * 1e-6f;
      if (n_ < 2 || age < window_s) return Encoder::getVelocity();
      best = i;
    }
    const float age = (float)(uint32_t)(now - t_[best]) * 1e-6f;
    const float v = (a - a_[best]) / age;
    v_slow_ += 0.002f * (v - v_slow_);
    return v;
  }

 private:
  static const int      kCap = 512;
  static const uint32_t kMinStepUs = 250;             // spacing of stored samples
  float    a_[kCap];
  uint32_t t_[kCap];
  int head_ = 0, n_ = 0;
  float v_slow_ = 0.0f;
  void push(float a, uint32_t t) {
    head_ = (head_ + 1) % kCap;
    a_[head_] = a; t_[head_] = t;
    if (n_ < kCap) n_++;
  }
};
