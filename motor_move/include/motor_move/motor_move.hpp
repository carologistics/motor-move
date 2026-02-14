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

#pragma once

#include "motor_move/mimo.hpp"
#include "motor_move/motion_profile.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_move_msgs/action/motor_move.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace motor_move {
using MotorMoveAction = motor_move_msgs::action::MotorMove;
using GoalHandleMotorMove = rclcpp_action::ServerGoalHandle<MotorMoveAction>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose = geometry_msgs::msg::Pose;
class MotorMove : public rclcpp::Node {
public:
  explicit MotorMove(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MotorMove();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
  rclcpp_action::Server<MotorMoveAction>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex target_pose_mutex_;
  PoseStamped target_pose_;

  PoseStamped
  to_frame(const geometry_msgs::msg::PoseStamped::SharedPtr point_ptr,
           std::string frame_id);
  void set_matrix_parameter(const std::string &name,
                            const Eigen::MatrixXd &matrix);

  Eigen::MatrixXd get_matrix_parameter(const std::string &name, int rows,
                                       int cols);
  std::string matrix_to_string(const Eigen::MatrixXd &matrix);
  MIMO_PID mimo_;
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const MotorMoveAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMotorMove> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
  inline float calculate_distance(const PoseStamped pose);

  std::string namespace_;
  std::string base_frame_;
  std::string odom_frame_;

  // Live Tuning
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  // =========================================================================
  // PID TUNING LOGGING
  // =========================================================================
  bool enable_tuning_log_;
  std::string tuning_log_path_;
  std::ofstream csv_file_;
  std::string experiment_timestamp_;
  bool logging_active_;
  std::string tuning_remote_target_;

  // =========================================================================
  // DECOUPLING (Entkopplung)
  // =========================================================================
  bool enable_decoupling_;
  Eigen::MatrixXd decoupling_matrix_;

  // =========================================================================
  // FEEDFORWARD (Motion Profile)
  // =========================================================================
  bool enable_feedforward_;
  double max_linear_velocity_;
  double max_linear_acceleration_;
  double max_angular_velocity_;
  double max_angular_acceleration_;

  void init_tuning_logging();
  void log_pid_data(double timestamp, double error_x, double error_y,
                    double error_yaw, double cmd_vel_x, double cmd_vel_y,
                    double cmd_vel_yaw, double target_x, double target_y,
                    double target_yaw, double ff_vel_x, double ff_vel_y,
                    double ff_vel_yaw, double pid_vel_x, double pid_vel_y,
                    double pid_vel_yaw);
  void finalize_tuning_logging();
  void generate_plot();
  void transfer_to_remote();
};
} // namespace motor_move
