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

// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/mimo.hpp"
#include <algorithm>
#include <cmath>

MIMO_PID::MIMO_PID(const Eigen::MatrixXd Kp, const Eigen::MatrixXd Ki,
                   const Eigen::MatrixXd Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd) {
  int n = Kp.cols();
  this->integral = Eigen::MatrixXd::Zero(n, 1);
  this->position_prev = Eigen::MatrixXd::Zero(n, 1);
  this->first_run = true;

  this->integral_min = Eigen::VectorXd::Constant(n, -10.0);
  this->integral_max = Eigen::VectorXd::Constant(n, 10.0);
  this->use_anti_windup = true;
}

MIMO_PID::~MIMO_PID() {}

MIMO_PID::MIMO_PID() {
  this->integral = Eigen::MatrixXd::Zero(3, 1);
  this->position_prev = Eigen::MatrixXd::Zero(3, 1);
  this->first_run = true;

  this->integral_min = Eigen::VectorXd::Constant(3, -10.0);
  this->integral_max = Eigen::VectorXd::Constant(3, 10.0);
  this->use_anti_windup = true;
}

void MIMO_PID::set_Kp(const Eigen::MatrixXd Kp) {
  this->Kp = Kp;
  if (this->integral.rows() != Kp.cols() || this->integral.cols() != 1) {
    int n = Kp.cols();
    this->integral = Eigen::MatrixXd::Zero(n, 1);
    this->position_prev = Eigen::MatrixXd::Zero(n, 1);
    this->integral_min = Eigen::VectorXd::Constant(n, -10.0);
    this->integral_max = Eigen::VectorXd::Constant(n, 10.0);
  }
}

void MIMO_PID::set_Ki(const Eigen::MatrixXd Ki) {
  this->Ki = Ki;
  if (this->integral.rows() != Ki.cols() || this->integral.cols() != 1) {
    int n = Ki.cols();
    this->integral = Eigen::MatrixXd::Zero(n, 1);
    this->position_prev = Eigen::MatrixXd::Zero(n, 1);
    this->integral_min = Eigen::VectorXd::Constant(n, -10.0);
    this->integral_max = Eigen::VectorXd::Constant(n, 10.0);
  }
}

void MIMO_PID::set_Kd(const Eigen::MatrixXd Kd) {
  this->Kd = Kd;
  if (this->integral.rows() != Kd.cols() || this->integral.cols() != 1) {
    int n = Kd.cols();
    this->integral = Eigen::MatrixXd::Zero(n, 1);
    this->position_prev = Eigen::MatrixXd::Zero(n, 1);
    this->integral_min = Eigen::VectorXd::Constant(n, -10.0);
    this->integral_max = Eigen::VectorXd::Constant(n, 10.0);
  }
}

void MIMO_PID::set_integral_limits(const Eigen::VectorXd &min_limits,
                                   const Eigen::VectorXd &max_limits) {
  this->integral_min = min_limits;
  this->integral_max = max_limits;
  this->use_anti_windup = true;
}

void MIMO_PID::set_integral_limits(double min_val, double max_val) {
  int n = this->integral.rows();
  this->integral_min = Eigen::VectorXd::Constant(n, min_val);
  this->integral_max = Eigen::VectorXd::Constant(n, max_val);
  this->use_anti_windup = true;
}

void MIMO_PID::set_p_term_limits(double max_linear, double max_angular) {
  this->p_max_linear = max_linear;
  this->p_max_angular = max_angular;
  this->use_p_limits = true;
}

void MIMO_PID::reset_integral() { this->integral.setZero(); }

void MIMO_PID::reset() {
  this->integral.setZero();
  this->position_prev.setZero();
  this->first_run = true;
}

void MIMO_PID::clamp_integral() {
  if (!use_anti_windup)
    return;

  for (int i = 0; i < integral.rows(); ++i) {
    if (integral(i, 0) < integral_min(i)) {
      integral(i, 0) = integral_min(i);
    } else if (integral(i, 0) > integral_max(i)) {
      integral(i, 0) = integral_max(i);
    }
  }
}

Eigen::MatrixXd MIMO_PID::compute(const Eigen::MatrixXd error,
                                  const Eigen::MatrixXd position,
                                  const double dt) {
  // Update integral
  this->integral += error * dt;
  clamp_integral();

  // Derivative on Measurement - no spike on setpoint change!
  Eigen::MatrixXd derivative;
  if (first_run) {
    // First run: no derivative (avoids spike)
    derivative = Eigen::MatrixXd::Zero(position.rows(), 1);
    first_run = false;
  } else {
    // Derivative of POSITION (negative because we want to slow down)
    derivative = -(position - this->position_prev) / dt;
  }
  this->position_prev = position;

  // Calculate each term separately
  Eigen::MatrixXd p_term = this->Kp * error;
  Eigen::MatrixXd i_term = this->Ki * this->integral;
  Eigen::MatrixXd d_term = this->Kd * derivative;

  // Limit P-term only (allows D-term to brake freely)
  if (use_p_limits) {
    double px = p_term(0, 0);
    double py = p_term(1, 0);
    double p_linear = std::sqrt(px * px + py * py);

    if (p_linear > p_max_linear && p_linear > 0.001) {
      double scale = p_max_linear / p_linear;
      p_term(0, 0) = px * scale;
      p_term(1, 0) = py * scale;
    }

    p_term(2, 0) = std::clamp(p_term(2, 0), -p_max_angular, p_max_angular);
  }

  return p_term + i_term + d_term;
}
