// Copyright (c) 2026 Carologistics
//
// Licensed under the Apache License, Version 2.0

#pragma once

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_move_msgs/action/motor_move.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<MotorMoveAction>::SharedPtr action_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string namespace_;
  std::string base_frame_;
  std::string odom_frame_;

  std::atomic<double> max_linear_speed_;
  std::atomic<double> linear_acceleration_;

  std::atomic<double> max_angular_speed_;
  std::atomic<double> angular_acceleration_;
  std::atomic<double> linear_kp_;
  std::atomic<double> angular_kp_;

  std::mutex state_mutex_;
  bool have_odom_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;

  std::shared_ptr<GoalHandleMotorMove> active_goal_;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_yaw_ = 0.0;
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  rclcpp::Time goal_start_time_;
  rclcpp::Time last_control_time_;


  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const MotorMoveAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle);

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool goal_to_odom(const PoseStamped &goal, double &x, double &y,
                    double &yaw);
  bool frame_is(const std::string &frame, const std::string &expected) const;
  void publish_stop();
  void clear_active_goal();
};

} // namespace motor_move
