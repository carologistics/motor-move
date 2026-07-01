// Copyright (c) 2026 Carologistics
//
// Licensed under the Apache License, Version 2.0

#include "motor_move/motor_move.hpp"

#include <algorithm>
#include <cmath>

namespace motor_move {
namespace {

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

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&MotorMove::odom_callback, this, std::placeholders::_1));

  this->declare_parameter("max_linear_speed", 0.5);
  this->declare_parameter("linear_acceleration", 0.5);
  this->declare_parameter("max_angular_speed", 0.5);
  this->declare_parameter("angular_acceleration", 0.5);
  this->declare_parameter("linear_kp", 1.0);
  this->declare_parameter("angular_kp", 1.5);

  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  linear_acceleration_ = this->get_parameter("linear_acceleration").as_double();

  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  angular_acceleration_ = this->get_parameter("angular_acceleration").as_double();
  linear_kp_ = this->get_parameter("linear_kp").as_double();
  angular_kp_ = this->get_parameter("angular_kp").as_double();

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotorMove::on_parameter_change, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&MotorMove::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotorMove::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "motor_move ready: odom_topic=odom base_frame=%s odom_frame=%s "
              "max_linear_speed=%.3f linear_acceleration=%.3f "
              "max_angular_speed=%.3f angular_acceleration=%.3f "
              "linear_kp=%.3f angular_kp=%.3f",
              base_frame_.c_str(), odom_frame_.c_str(),
              max_linear_speed_.load(), linear_acceleration_.load(),
              max_angular_speed_.load(), angular_acceleration_.load(),
              linear_kp_.load(), angular_kp_.load());
}

rcl_interfaces::msg::SetParametersResult MotorMove::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const auto &name = param.get_name();
    if (name != "max_linear_speed" && name != "linear_acceleration" &&
        name != "max_angular_speed" && name != "angular_acceleration" &&
        name != "linear_kp" && name != "angular_kp") {
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
    if (param.get_name() == "max_linear_speed") {
      max_linear_speed_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "max_linear_speed set to %.3f",
                  max_linear_speed_.load());
    } else if (param.get_name() == "linear_acceleration") {
      linear_acceleration_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "linear_acceleration set to %.3f",
                  linear_acceleration_.load());
    }
    if (param.get_name() == "max_angular_speed") {
      max_angular_speed_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "max_angular_speed set to %.3f",
                  max_angular_speed_.load());
    } else if (param.get_name() == "angular_acceleration") {
      angular_acceleration_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "angular_acceleration set to %.3f",
                  angular_acceleration_.load());
    } else if (param.get_name() == "linear_kp") {
      linear_kp_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "linear_kp set to %.3f",
                  linear_kp_.load());
    } else if (param.get_name() == "angular_kp") {
      angular_kp_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "angular_kp set to %.3f",
                  angular_kp_.load());
    }
  }

  return result;
}

bool MotorMove::frame_is(const std::string &frame,
                         const std::string &expected) const {
  return frame == expected || frame == "/" + expected ||
         frame == expected.substr(expected.find_last_of('/') + 1);
}

bool MotorMove::goal_to_odom(const PoseStamped &goal, double &x, double &y,
                             double &yaw) const {
  const std::string frame =
      goal.header.frame_id.empty() ? base_frame_ : goal.header.frame_id;
  const double goal_yaw = yaw_from_quaternion(goal.pose.orientation);

  if (frame_is(frame, odom_frame_)) {
    x = goal.pose.position.x;
    y = goal.pose.position.y;
    yaw = goal_yaw;
    return true;
  }

  if (frame_is(frame, base_frame_)) {
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    x = current_x_ + cos_yaw * goal.pose.position.x -
        sin_yaw * goal.pose.position.y;
    y = current_y_ + sin_yaw * goal.pose.position.x +
        cos_yaw * goal.pose.position.y;
    yaw = normalize_angle(current_yaw_ + goal_yaw);
    return true;
  }

  return false;
}

void MotorMove::publish_stop() {
  cmd_vel_->publish(geometry_msgs::msg::Twist{});
}

void MotorMove::clear_active_goal() {
  active_goal_.reset();
  linear_speed_ = 0.0;
  angular_speed_ = 0.0;
}

rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: no odom received yet");
    return rclcpp_action::GoalResponse::REJECT;
  }

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (!goal_to_odom(goal->motor_goal, x, y, yaw)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Rejecting goal: frame '%s' is not supported. Use '%s' or "
                 "'%s'.",
                 goal->motor_goal.header.frame_id.c_str(), odom_frame_.c_str(),
                 base_frame_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(),
              "Accepted goal request in frame '%s': odom target=(%.3f, %.3f, "
              "%.3f)",
              goal->motor_goal.header.frame_id.c_str(), x, y, yaw);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (active_goal_ == goal_handle) {
    publish_stop();
  }
  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  auto result = std::make_shared<MotorMoveAction::Result>();
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!have_odom_) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Could not start goal: no odom");
    return;
  }

  if (active_goal_) {
    auto previous_result = std::make_shared<MotorMoveAction::Result>();
    previous_result->success = false;
    active_goal_->abort(previous_result);
  }

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (!goal_to_odom(goal_handle->get_goal()->motor_goal, x, y, yaw)) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  active_goal_ = goal_handle;
  target_x_ = x;
  target_y_ = y;
  target_yaw_ = yaw;
  linear_speed_ = 0.0;
  angular_speed_ = 0.0;
  goal_start_time_ = this->now();
  last_control_time_ = goal_start_time_;

  RCLCPP_INFO(this->get_logger(),
              "Stored odom target: x=%.3f y=%.3f yaw=%.3f", target_x_,
              target_y_, target_yaw_);
}

void MotorMove::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
  have_odom_ = true;

  if (!active_goal_) {
    return;
  }

  auto feedback = std::make_shared<MotorMoveAction::Feedback>();
  auto result = std::make_shared<MotorMoveAction::Result>();

  if (active_goal_->is_canceling()) {
    publish_stop();
    result->success = false;
    active_goal_->canceled(result);
    clear_active_goal();
    return;
  }

  const rclcpp::Time now = this->now();
  if ((now - goal_start_time_).seconds() > kTimeoutSeconds) {
    publish_stop();
    result->success = false;
    active_goal_->abort(result);
    clear_active_goal();
    RCLCPP_WARN(this->get_logger(), "Goal timed out");
    return;
  }

  const double dx = target_x_ - current_x_;
  const double dy = target_y_ - current_y_;
  const double distance = std::hypot(dx, dy);
  const double yaw_error = normalize_angle(target_yaw_ - current_yaw_);
  const double abs_yaw_error = std::fabs(yaw_error);

  feedback->distance_to_target = static_cast<float>(distance);
  feedback->yaw_error = static_cast<float>(abs_yaw_error);
  feedback->elapsed_time = static_cast<float>((now - goal_start_time_).seconds());

  double dt = (now - last_control_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.05;
  }
  last_control_time_ = now;


  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "odom target=(%.3f, %.3f, %.3f) current=(%.3f, %.3f, %.3f) "
      "error=(%.3f, %.3f, %.3f) distance=%.3f",
      target_x_, target_y_, target_yaw_, current_x_, current_y_, current_yaw_,
      dx, dy, yaw_error, distance);

  if (distance <= kDistanceTolerance && abs_yaw_error <= kYawToleranceRadians) {
    publish_stop();
    result->success = true;
    active_goal_->succeed(result);
    clear_active_goal();
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    return;
  }

  const double max_linear_speed = max_linear_speed_.load();
  const double linear_acceleration = linear_acceleration_.load();

  const double max_angular_speed = max_angular_speed_.load();
  const double angular_acceleration = angular_acceleration_.load();
  const double linear_kp = linear_kp_.load();
  const double angular_kp = angular_kp_.load();

  const bool rotate_first = abs_yaw_error > kYawToleranceRadians;

  const double braking_angle =
      max_angular_speed * max_angular_speed / (2.0 * angular_acceleration);
  const double braking_distance =
      max_linear_speed * max_linear_speed / (2.0 * linear_acceleration);

  geometry_msgs::msg::Twist cmd;

  if (rotate_first) {
    linear_speed_ = 0.0;

    if (abs_yaw_error > braking_angle) {
      angular_speed_ += std::copysign(angular_acceleration * dt, yaw_error);

      if (std::abs(angular_speed_) > max_angular_speed) {
        angular_speed_ = std::copysign(max_angular_speed, yaw_error);
      }
    } else {
      const double profile_speed =
          std::sqrt(2.0 * angular_acceleration * abs_yaw_error);
      const double damping_speed = angular_kp * abs_yaw_error;
      angular_speed_ =
          std::copysign(std::min(profile_speed, damping_speed), yaw_error);
    }
    angular_speed_ = std::copysign(
        std::min(std::fabs(angular_speed_), abs_yaw_error / dt),
        angular_speed_);

    cmd.angular.z = angular_speed_;
  } else if (distance > kDistanceTolerance) {
    angular_speed_ = 0.0;

    if (distance > braking_distance) {
      linear_speed_ += linear_acceleration * dt;

      if (linear_speed_ > max_linear_speed) {
        linear_speed_ = max_linear_speed;
      }
    } else {
      const double profile_speed =
          std::sqrt(2.0 * linear_acceleration * distance);
      const double damping_speed = linear_kp * distance;
      linear_speed_ = std::min(profile_speed, damping_speed);
    }
    linear_speed_ = std::min(linear_speed_, distance / dt);

    const double dir_x = dx / distance;
    const double dir_y = dy / distance;

    // odom/map direction -> robot/base_link direction
    const double c = std::cos(current_yaw_);
    const double s = std::sin(current_yaw_);

    cmd.linear.x = linear_speed_ * ( c * dir_x + s * dir_y);
    cmd.linear.y = linear_speed_ * (-s * dir_x + c * dir_y);
  } else {
    linear_speed_ = 0.0;
    angular_speed_ = 0.0;
  }

  feedback->linear_speed =
      static_cast<float>(std::hypot(cmd.linear.x, cmd.linear.y));
  feedback->angular_speed = static_cast<float>(std::fabs(cmd.angular.z));
  active_goal_->publish_feedback(feedback);

  cmd_vel_->publish(cmd);
}

} // namespace motor_move

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_move::MotorMove>());
  rclcpp::shutdown();
  return 0;
}
