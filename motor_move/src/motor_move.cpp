// Copyright (c) 2026 Carologistics
//
// Licensed under the Apache License, Version 2.0

#include "motor_move/motor_move.hpp"

#include "tf2/utils.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace motor_move {
namespace {

constexpr double kLoopRateHz = 20.0;
constexpr double kTimeoutSeconds = 10.0;
constexpr double kDistanceTolerance = 0.02;
constexpr double kYawToleranceRadians = 2.0 * M_PI / 180.0;

double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double ramp_toward(double current, double target, double max_delta) {
  if (target > current) {
    return std::min(target, current + max_delta);
  }
  return std::max(target, current - max_delta);
}

double braking_speed(double error, double max_speed, double acceleration) {
  if (error <= 0.0 || max_speed <= 0.0 || acceleration <= 0.0) {
    return 0.0;
  }
  return std::min(max_speed, std::sqrt(2.0 * acceleration * error));
}

std::string frame_with_namespace(const std::string &ns,
                                 const std::string &frame) {
  if (ns == "/" || ns.empty()) {
    return frame;
  }
  return ns.substr(1) + "/" + frame;
}

} // namespace

MotorMove::MotorMove(const rclcpp::NodeOptions &options)
    : Node("motor_move", options) {
  namespace_ = this->get_namespace();
  odom_frame_ = frame_with_namespace(namespace_, "odom");
  base_frame_ = frame_with_namespace(namespace_, "base_link");

  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("acceleration", 0.5);

  max_speed_ = this->get_parameter("max_speed").as_double();
  acceleration_ = this->get_parameter("acceleration").as_double();

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotorMove::on_parameter_change, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&MotorMove::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotorMove::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "motor_move ready: base_frame=%s odom_frame=%s max_speed=%.3f "
              "acceleration=%.3f",
              base_frame_.c_str(), odom_frame_.c_str(), max_speed_.load(),
              acceleration_.load());
}

rcl_interfaces::msg::SetParametersResult MotorMove::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const auto &name = param.get_name();
    if (name != "max_speed" && name != "acceleration") {
      continue;
    }

    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = name + " must be a double";
      return result;
    }
    if (param.as_double() <= 0.0) {
      result.successful = false;
      result.reason = name + " must be greater than 0";
      return result;
    }
  }

  for (const auto &param : parameters) {
    if (param.get_name() == "max_speed") {
      max_speed_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "max_speed set to %.3f",
                  max_speed_.load());
    } else if (param.get_name() == "acceleration") {
      acceleration_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "acceleration set to %.3f",
                  acceleration_.load());
    }
  }

  return result;
}

PoseStamped MotorMove::transform_to_odom(const PoseStamped &pose) {
  PoseStamped target = pose;
  if (target.header.frame_id.empty()) {
    target.header.frame_id = base_frame_;
  }
  target.header.stamp = rclcpp::Time(0);

  PoseStamped transformed;
  tf_buffer_->transform(target, transformed, odom_frame_);
  transformed.header.stamp = rclcpp::Time(0);
  return transformed;
}

PoseStamped MotorMove::transform_to_base(const PoseStamped &pose) {
  PoseStamped target = pose;
  target.header.stamp = rclcpp::Time(0);

  PoseStamped transformed;
  tf_buffer_->transform(target, transformed, base_frame_);
  return transformed;
}

void MotorMove::publish_stop() {
  cmd_vel_->publish(geometry_msgs::msg::Twist{});
}

rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid;

  const auto &pose = goal->motor_goal;
  const std::string frame =
      pose.header.frame_id.empty() ? base_frame_ : pose.header.frame_id;
  RCLCPP_INFO(this->get_logger(),
              "Received motor move goal in frame '%s': x=%.3f y=%.3f yaw=%.3f",
              frame.c_str(), pose.pose.position.x, pose.pose.position.y,
              tf2::getYaw(pose.pose.orientation));

  try {
    (void)transform_to_odom(pose);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Rejecting goal: %s", ex.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  PoseStamped target_pose;
  try {
    target_pose = transform_to_odom(goal_handle->get_goal()->motor_goal);
  } catch (const tf2::TransformException &ex) {
    auto result = std::make_shared<MotorMoveAction::Result>();
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Could not start goal: %s", ex.what());
    return;
  }

  std::thread{std::bind(&MotorMove::execute, this, std::placeholders::_1,
                        target_pose),
              goal_handle}
      .detach();
}

void MotorMove::execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle,
                        PoseStamped target_pose) {
  auto feedback = std::make_shared<MotorMoveAction::Feedback>();
  auto result = std::make_shared<MotorMoveAction::Result>();

  rclcpp::Rate rate(kLoopRateHz);
  rclcpp::Time start_time = this->now();
  rclcpp::Time last_time = start_time;
  double linear_speed = 0.0;
  double angular_speed = 0.0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      publish_stop();
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    const rclcpp::Time now = this->now();
    if ((now - start_time).seconds() > kTimeoutSeconds) {
      publish_stop();
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Goal timed out");
      return;
    }

    PoseStamped error;
    try {
      error = transform_to_base(target_pose);
    } catch (const tf2::TransformException &ex) {
      publish_stop();
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    const double dx = error.pose.position.x;
    const double dy = error.pose.position.y;
    const double distance = std::hypot(dx, dy);
    const double yaw_error = normalize_angle(tf2::getYaw(error.pose.orientation));
    const double abs_yaw_error = std::fabs(yaw_error);

    feedback->distance_to_target = static_cast<float>(distance);
    feedback->yaw_error = static_cast<float>(abs_yaw_error);
    feedback->elapsed_time = static_cast<float>((now - start_time).seconds());

    if (distance <= kDistanceTolerance &&
        abs_yaw_error <= kYawToleranceRadians) {
      publish_stop();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal reached");
      return;
    }

    double dt = (now - last_time).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / kLoopRateHz;
    }
    last_time = now;

    const double max_speed = max_speed_.load();
    const double acceleration = acceleration_.load();
    const double max_delta = acceleration * dt;

    const double target_linear_speed =
        braking_speed(distance, max_speed, acceleration);
    linear_speed = ramp_toward(linear_speed, target_linear_speed, max_delta);

    const double target_angular_speed =
        std::copysign(braking_speed(abs_yaw_error, max_speed, acceleration),
                      yaw_error);
    angular_speed = ramp_toward(angular_speed, target_angular_speed, max_delta);

    geometry_msgs::msg::Twist cmd;
    if (distance > kDistanceTolerance) {
      cmd.linear.x = linear_speed * dx / distance;
      cmd.linear.y = linear_speed * dy / distance;
    }
    if (abs_yaw_error > kYawToleranceRadians) {
      cmd.angular.z = angular_speed;
    }

    feedback->linear_speed = static_cast<float>(
        std::hypot(cmd.linear.x, cmd.linear.y));
    feedback->angular_speed = static_cast<float>(std::fabs(cmd.angular.z));
    goal_handle->publish_feedback(feedback);

    cmd_vel_->publish(cmd);
    rate.sleep();
  }

  publish_stop();
  result->success = false;
  goal_handle->abort(result);
}

} // namespace motor_move

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_move::MotorMove>());
  rclcpp::shutdown();
  return 0;
}
