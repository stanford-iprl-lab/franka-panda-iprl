/**
 * timer.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 2, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIALDYN_UTILS_TIMER_H_
#define SPATIALDYN_UTILS_TIMER_H_

#include <chrono>  // std::chrono
#include <thread>  // std::this_thread

namespace SpatialDyn {

class Timer {

 public:

  Timer() {}

  Timer(double frequency)
      : ns_interval_(std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / frequency))) {}

  /**
   * Timer loop frequency in Hz.
   */
  void set_frequency(double frequency);
  double frequency() const;

  /**
   * Timer loop time interval in seconds (1 / frequency).
   */
  void set_dt(double dt);
  double dt() const;

  /**
   * Current CPU time since epoch in seconds.
   */
  double time() const;

  /**
   * CPU time since last timer reset in seconds.
   */
  double time_elapsed() const;

  /**
   * Simulation time since last timer reset in seconds.
   */
  double time_sim() const;

  /**
   * Number of loop iterations since last timer reset.
   */
  unsigned long long num_iterations() const;

  /**
   * Reset timer.
   */
  void Reset();

  /**
   * Wait for next timer loop.
   */
  void Sleep();

 private:

  std::chrono::steady_clock::time_point t_start_;
  std::chrono::steady_clock::time_point t_next_;
  unsigned long long num_iters_ = 0;

  bool initialized_ = false;
  std::chrono::nanoseconds ns_interval_ = std::chrono::milliseconds(1);

};

void Timer::set_frequency(double frequency) {
  ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / frequency));
}

double Timer::frequency() const {
  return 1e9 / ns_interval_.count();
}

void Timer::set_dt(double dt) {
  ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(dt / 1e9));
}

double Timer::dt() const {
  return ns_interval_.count() / 1e9;
}


double Timer::time() const {
  auto now = std::chrono::steady_clock::now();
  auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto ns_since_epoch = now_ns.time_since_epoch();
  return ns_since_epoch.count() / 1e9;
}

double Timer::time_elapsed() const {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - t_start_;
  auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
  return ns_elapsed.count() / 1e9;
}

double Timer::time_sim() const {
  auto elapsed = t_next_ - t_start_;
  auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
  return ns_elapsed.count() / 1e9;
}

unsigned long long Timer::num_iterations() const {
  return num_iters_;
}

void Timer::Reset() {
  t_start_ = std::chrono::steady_clock::now();
  t_next_  = t_start_ + ns_interval_;
  num_iters_ = 0;
}

void Timer::Sleep() {
  num_iters_++;
  if (!initialized_) {
    initialized_ = true;
    t_start_ = std::chrono::steady_clock::now();
    t_next_ = t_start_ + ns_interval_;
    return;
  }

  auto t_curr = std::chrono::steady_clock::now();
  if (t_curr < t_next_) {
    std::this_thread::sleep_for(t_next_ - t_curr);
  }
  t_next_ += ns_interval_;
}

}  // namespace SpatialDyn

#endif  // SPATIALDYN_UTILS_TIMER_H_
