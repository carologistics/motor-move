// Copyright (c) 2026 Carologistics
//
// Licensed under the Apache License, Version 2.0

#pragma once

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_move_msgs/action/motor_move.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace motor_move {

using MotorMoveAction = motor_move_msgs::action::MotorMove;
using GoalHandleMotorMove = rclcpp_action::ServerGoalHandle<MotorMoveAction>;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class MotorMove : public rclcpp::Node {
public:
  explicit MotorMove(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
  rclcpp_action::Server<MotorMoveAction>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  std::string namespace_;
  std::string base_frame_;
  std::string odom_frame_;

  std::atomic<double> max_speed_;
  std::atomic<double> acceleration_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const MotorMoveAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle,
               PoseStamped target_pose);

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  PoseStamped transform_to_odom(const PoseStamped &pose);
  void publish_stop();
};

} // namespace motor_move
