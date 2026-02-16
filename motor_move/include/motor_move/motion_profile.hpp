// Copyright (c) 2026 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP

#include <algorithm>
#include <cmath>

/**
 * Time-based trapezoidal motion profile for trajectory generation.
 *
 * Given a total distance, max velocity, and max acceleration, generates
 * a trapezoidal (or triangular) velocity profile over time.
 *
 * At each time step, returns the reference position (distance traveled)
 * and reference velocity. This enables the PID to track intermediate
 * setpoints rather than the final goal, keeping tracking errors small.
 *
 * Phases:
 *   1. Acceleration: 0 -> peak_vel
 *   2. Cruise: peak_vel (only if distance is large enough)
 *   3. Deceleration: peak_vel -> 0
 */
class TrajectoryProfile {
public:
  struct State {
    double position; // distance traveled along profile [m or rad]
    double velocity; // profile velocity at this time [m/s or rad/s]
  };

  TrajectoryProfile() = default;

  void configure(double total_distance, double max_vel, double max_accel) {
    distance_ = std::fabs(total_distance);
    max_vel_ = max_vel;
    max_accel_ = max_accel;

    if (distance_ < 1e-6) {
      accel_time_ = 0.0;
      cruise_time_ = 0.0;
      total_time_ = 0.0;
      peak_vel_ = 0.0;
      return;
    }

    // Distance needed to accelerate to max_vel and decelerate back to 0
    double d_to_max = max_vel_ * max_vel_ / (2.0 * max_accel_);

    if (distance_ < 2.0 * d_to_max) {
      // Triangular profile: never reaches max_vel
      peak_vel_ = std::sqrt(max_accel_ * distance_);
      accel_time_ = peak_vel_ / max_accel_;
      cruise_time_ = 0.0;
    } else {
      // Trapezoidal profile: reaches max_vel
      peak_vel_ = max_vel_;
      accel_time_ = max_vel_ / max_accel_;
      double d_accel = 0.5 * max_accel_ * accel_time_ * accel_time_;
      cruise_time_ = (distance_ - 2.0 * d_accel) / max_vel_;
    }
    total_time_ = 2.0 * accel_time_ + cruise_time_;
  }

  State compute(double t) const {
    if (t <= 0.0 || total_time_ <= 0.0) {
      return {0.0, 0.0};
    }

    if (t >= total_time_) {
      return {distance_, 0.0};
    }

    double pos, vel;

    if (t < accel_time_) {
      // Acceleration phase
      vel = max_accel_ * t;
      pos = 0.5 * max_accel_ * t * t;
    } else if (t < accel_time_ + cruise_time_) {
      // Cruise phase
      double t_cruise = t - accel_time_;
      vel = peak_vel_;
      double d_accel = 0.5 * max_accel_ * accel_time_ * accel_time_;
      pos = d_accel + peak_vel_ * t_cruise;
    } else {
      // Deceleration phase
      double t_decel = t - accel_time_ - cruise_time_;
      vel = peak_vel_ - max_accel_ * t_decel;
      if (vel < 0.0)
        vel = 0.0;
      double d_accel = 0.5 * max_accel_ * accel_time_ * accel_time_;
      double d_cruise = peak_vel_ * cruise_time_;
      pos = d_accel + d_cruise + peak_vel_ * t_decel -
            0.5 * max_accel_ * t_decel * t_decel;
    }

    return {pos, vel};
  }

  double total_time() const { return total_time_; }
  bool is_finished(double t) const { return t >= total_time_; }

private:
  double distance_ = 0.0;
  double max_vel_ = 0.5;
  double max_accel_ = 0.5;
  double accel_time_ = 0.0;
  double cruise_time_ = 0.0;
  double total_time_ = 0.0;
  double peak_vel_ = 0.0;
};

#endif // MOTION_PROFILE_HPP
