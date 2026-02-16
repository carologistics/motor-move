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

#include <algorithm>
// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/motor_move.hpp"
#include "tf2/utils.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <filesystem>
#include <iomanip>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace motor_move {

// =============================================================================
// HILFSFUNKTIONEN FÜR MATRIX-PARAMETER
// =============================================================================

void MotorMove::set_matrix_parameter(const std::string &name,
                                     const Eigen::MatrixXd &matrix) {
  std::vector<double> flat_matrix(matrix.data(), matrix.data() + matrix.size());
  this->declare_parameter(name, rclcpp::ParameterValue(flat_matrix));
}

Eigen::MatrixXd MotorMove::get_matrix_parameter(const std::string &name,
                                                int rows, int cols) {
  std::vector<double> flat_matrix;
  this->get_parameter(name, flat_matrix);
  Eigen::MatrixXd matrix = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      flat_matrix.data(), rows, cols);
  return matrix;
}

std::string MotorMove::matrix_to_string(const Eigen::MatrixXd &matrix) {
  std::ostringstream ss;
  ss << matrix;
  return ss.str();
}

// =============================================================================
// PID TUNING LOGGING
// =============================================================================

void MotorMove::init_tuning_logging() {
  if (!enable_tuning_log_) {
    logging_active_ = false;
    return;
  }

  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream oss;
  oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
  experiment_timestamp_ = oss.str();

  std::string dir_path = tuning_log_path_ + "/" + experiment_timestamp_;
  try {
    std::filesystem::create_directories(dir_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to create tuning log directory: %s", e.what());
    logging_active_ = false;
    return;
  }

  std::string csv_path = dir_path + "/pid_data.csv";
  csv_file_.open(csv_path);

  if (!csv_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s",
                 csv_path.c_str());
    logging_active_ = false;
    return;
  }

  // CSV-Header: original columns + feedforward/pid breakdown
  csv_file_ << "timestamp,error_x,error_y,error_yaw,"
            << "cmd_vel_x,cmd_vel_y,cmd_vel_yaw,"
            << "target_x,target_y,target_yaw,"
            << "ff_vel_x,ff_vel_y,ff_vel_yaw,"
            << "pid_vel_x,pid_vel_y,pid_vel_yaw\n";
  csv_file_.flush();

  logging_active_ = true;
  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Logging started: %s",
              csv_path.c_str());
}

void MotorMove::log_pid_data(
    double timestamp, double error_x, double error_y, double error_yaw,
    double cmd_vel_x, double cmd_vel_y, double cmd_vel_yaw, double target_x,
    double target_y, double target_yaw, double ff_vel_x, double ff_vel_y,
    double ff_vel_yaw, double pid_vel_x, double pid_vel_y, double pid_vel_yaw) {
  if (!logging_active_ || !csv_file_.is_open()) {
    return;
  }

  csv_file_ << std::fixed << std::setprecision(6) << timestamp << "," << error_x
            << "," << error_y << "," << error_yaw << "," << cmd_vel_x << ","
            << cmd_vel_y << "," << cmd_vel_yaw << "," << target_x << ","
            << target_y << "," << target_yaw << "," << ff_vel_x << ","
            << ff_vel_y << "," << ff_vel_yaw << "," << pid_vel_x << ","
            << pid_vel_y << "," << pid_vel_yaw << "\n";
}

void MotorMove::generate_plot() {
  std::string local_dir = tuning_log_path_ + "/" + experiment_timestamp_;
  std::string csv_path = local_dir + "/pid_data.csv";
  std::string png_path = local_dir + "/pid_analysis.png";

  std::string script_path = "/home/robotino/ros2/robotino_navigation_ws/src/"
                            "motor-move/motor_move/scripts/plot_pid_data.py";

  std::string plot_cmd = "python3 " + script_path + " " + csv_path + " " +
                         png_path + " 2>/dev/null";

  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Generating plot...");

  int result = std::system(plot_cmd.c_str());

  if (result == 0) {
    RCLCPP_INFO(this->get_logger(), "[PID TUNING] Plot saved: %s",
                png_path.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
                "[PID TUNING] Plot generation failed (exit code: %d)", result);
  }
}

void MotorMove::transfer_to_remote() {
  if (tuning_remote_target_.empty()) {
    return;
  }

  std::string local_dir = tuning_log_path_ + "/" + experiment_timestamp_;

  size_t colon_pos = tuning_remote_target_.find(':');
  if (colon_pos == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(),
                 "[PID TUNING] Invalid remote target format: %s (expected "
                 "user@host:/path)",
                 tuning_remote_target_.c_str());
    return;
  }

  std::string user_host = tuning_remote_target_.substr(0, colon_pos);
  std::string remote_base_path = tuning_remote_target_.substr(colon_pos + 1);
  std::string remote_full_path = remote_base_path + "/" + experiment_timestamp_;

  std::string mkdir_cmd = "ssh -o ConnectTimeout=5 -o BatchMode=yes " +
                          user_host + " 'mkdir -p " + remote_full_path +
                          "' 2>/dev/null";
  int mkdir_result = std::system(mkdir_cmd.c_str());

  if (mkdir_result != 0) {
    RCLCPP_WARN(this->get_logger(),
                "[PID TUNING] Failed to create remote directory (exit code: "
                "%d). Trying scp anyway...",
                mkdir_result);
  }

  std::string scp_cmd = "scp -o ConnectTimeout=5 -o BatchMode=yes -r " +
                        local_dir + "/* " + user_host + ":" + remote_full_path +
                        "/ 2>/dev/null &";

  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Transferring to remote: %s:%s",
              user_host.c_str(), remote_full_path.c_str());

  int result = std::system(scp_cmd.c_str());

  if (result == 0) {
    RCLCPP_INFO(this->get_logger(),
                "[PID TUNING] Transfer initiated (CSV + PNG)");
  } else {
    RCLCPP_WARN(this->get_logger(), "[PID TUNING] SCP command returned "
                                    "non-zero. Check SSH key authentication.");
  }
}

void MotorMove::finalize_tuning_logging() {
  if (!logging_active_) {
    return;
  }

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
    RCLCPP_INFO(this->get_logger(),
                "[PID TUNING] Logging finished: %s/%s/pid_data.csv",
                tuning_log_path_.c_str(), experiment_timestamp_.c_str());
  }

  generate_plot();
  transfer_to_remote();

  logging_active_ = false;
}

// =============================================================================
// KONSTRUKTOR
// =============================================================================

MotorMove::MotorMove(const rclcpp::NodeOptions &options)
    : Node("motor_move", options), mimo_(), logging_active_(false) {

  namespace_ = this->get_namespace();
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  if (namespace_ != "/") {
    if (!namespace_.empty() && namespace_[0] == '/') {
      namespace_ = namespace_.substr(1);
    }
    odom_frame = namespace_ + "/" + odom_frame;
    base_frame = namespace_ + "/" + base_frame;
  }

  base_frame_ = base_frame;
  odom_frame_ = odom_frame;
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_link frame id: %s",
              base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom frame id: %s", odom_frame_.c_str());

  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, _1, _2),
      std::bind(&MotorMove::handle_cancel, this, _1),
      std::bind(&MotorMove::handle_accepted, this, _1));

  // --- PID Gains ---
  std::vector<double> default_Kp = {1.8, 0.0, 0.0, 0.0, 1.8,
                                    0.0, 0.0, 0.0, 1.8};
  std::vector<double> default_Ki = {0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0};
  std::vector<double> default_Kd = {0.2, 0.0, 0.0, 0.0, 0.2,
                                    0.0, 0.0, 0.0, 0.2};

  this->declare_parameter("Kp", default_Kp);
  this->declare_parameter("Ki", default_Ki);
  this->declare_parameter("Kd", default_Kd);

  Eigen::MatrixXd Kp_matrix = get_matrix_parameter("Kp", 3, 3);
  Eigen::MatrixXd Ki_matrix = get_matrix_parameter("Ki", 3, 3);
  Eigen::MatrixXd Kd_matrix = get_matrix_parameter("Kd", 3, 3);

  RCLCPP_INFO(this->get_logger(), "Kp:\n%s",
              matrix_to_string(Kp_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Ki:\n%s",
              matrix_to_string(Ki_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Kd:\n%s",
              matrix_to_string(Kd_matrix).c_str());

  mimo_.set_Kp(Kp_matrix);
  mimo_.set_Ki(Ki_matrix);
  mimo_.set_Kd(Kd_matrix);

  // --- P-Term Limits (live tunable) ---
  this->declare_parameter("p_max_linear", 0.4);
  this->declare_parameter("p_max_angular", 1.0);

  double p_max_lin, p_max_ang;
  this->get_parameter("p_max_linear", p_max_lin);
  this->get_parameter("p_max_angular", p_max_ang);
  mimo_.set_p_term_limits(p_max_lin, p_max_ang);

  RCLCPP_INFO(this->get_logger(),
              "P-term limits: linear=%.2f m/s, angular=%.2f rad/s", p_max_lin,
              p_max_ang);

  // --- TF2 ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // --- Steuerungsparameter ---
  this->declare_parameter("loop_rate", 15.0);
  this->declare_parameter("timeout_seconds", 10.0);
  this->declare_parameter("yaw_tolerance_degrees", 5.0);
  this->declare_parameter("distance_tolerance", 0.05);

  double loop_rate_val, timeout_val, yaw_tol_val, dist_tol_val;
  this->get_parameter("loop_rate", loop_rate_val);
  this->get_parameter("timeout_seconds", timeout_val);
  this->get_parameter("yaw_tolerance_degrees", yaw_tol_val);
  this->get_parameter("distance_tolerance", dist_tol_val);

  RCLCPP_INFO(this->get_logger(), "Control parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Loop rate: %f Hz", loop_rate_val);
  RCLCPP_INFO(this->get_logger(), "  Timeout: %f seconds", timeout_val);
  RCLCPP_INFO(this->get_logger(), "  Yaw tolerance: %f degrees", yaw_tol_val);
  RCLCPP_INFO(this->get_logger(), "  Distance tolerance: %f meters",
              dist_tol_val);

  // =========================================================================
  // PID TUNING LOGGING PARAMETER
  // =========================================================================
  this->declare_parameter("enable_tuning_log", false);
  this->declare_parameter("tuning_log_path", std::string("/tmp/pid_tuning"));
  this->declare_parameter("tuning_remote_target", std::string(""));

  this->get_parameter("enable_tuning_log", enable_tuning_log_);
  this->get_parameter("tuning_log_path", tuning_log_path_);
  this->get_parameter("tuning_remote_target", tuning_remote_target_);

  if (enable_tuning_log_) {
    RCLCPP_WARN(this->get_logger(), "=== PID TUNING LOGGING ENABLED ===");
    RCLCPP_WARN(this->get_logger(),
                "CSV logs will be saved to: %s/<timestamp>/pid_data.csv",
                tuning_log_path_.c_str());

    if (!tuning_remote_target_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Remote transfer enabled: %s/<timestamp>/pid_data.csv",
                  tuning_remote_target_.c_str());
    }
  }

  // =========================================================================
  // DECOUPLING (Entkopplung)
  // =========================================================================
  std::vector<double> default_decoupling = {1.0, 0.0, 0.0, 0.0, 1.0,
                                            0.0, 0.0, 0.0, 1.0};
  this->declare_parameter("decoupling_matrix", default_decoupling);
  this->declare_parameter("enable_decoupling", false);

  this->get_parameter("enable_decoupling", enable_decoupling_);
  decoupling_matrix_ = get_matrix_parameter("decoupling_matrix", 3, 3);

  if (enable_decoupling_) {
    RCLCPP_WARN(this->get_logger(), "=== DECOUPLING ENABLED ===");
    RCLCPP_WARN(this->get_logger(), "Decoupling matrix D:\n%s",
                matrix_to_string(decoupling_matrix_).c_str());
  }

  // =========================================================================
  // FEEDFORWARD (Motion Profile)
  // =========================================================================
  this->declare_parameter("enable_feedforward", false);
  this->declare_parameter("max_linear_velocity", 0.5);
  this->declare_parameter("max_linear_acceleration", 0.5);
  this->declare_parameter("max_angular_velocity", 1.0);
  this->declare_parameter("max_angular_acceleration", 1.0);

  this->get_parameter("enable_feedforward", enable_feedforward_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_angular_acceleration", max_angular_acceleration_);

  if (enable_feedforward_) {
    RCLCPP_WARN(this->get_logger(), "=== FEEDFORWARD ENABLED ===");
    RCLCPP_WARN(
        this->get_logger(),
        "Motion profile: max_lin_vel=%.2f m/s, max_lin_accel=%.2f m/s^2",
        max_linear_velocity_, max_linear_acceleration_);
    RCLCPP_WARN(
        this->get_logger(),
        "                max_ang_vel=%.2f rad/s, max_ang_accel=%.2f rad/s^2",
        max_angular_velocity_, max_angular_acceleration_);
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Feedforward disabled. Use enable_feedforward:=true to enable.");
  }

  // =========================================================================
  // LIVE TUNING FEATURE
  // =========================================================================
  this->declare_parameter("enable_live_tuning", false);
  bool live_tuning_enabled = false;
  this->get_parameter("enable_live_tuning", live_tuning_enabled);

  if (live_tuning_enabled) {
    RCLCPP_WARN(this->get_logger(), "=== LIVE TUNING MODE ENABLED ===");
    RCLCPP_WARN(
        this->get_logger(),
        "PID gains, P-limits, feedforward params can be changed at runtime.");
    RCLCPP_WARN(
        this->get_logger(),
        "Use: ros2 param set <node> Kp \"[1.8, 0, 0, 0, 1.8, 0, 0, 0, 1.8]\"");

    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &MotorMove::on_parameter_change, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(this->get_logger(), "Live tuning disabled. Launch with "
                                    "enable_live_tuning:=true to enable.");
  }
}

// =============================================================================
// PARAMETER CHANGE CALLBACK (Live Tuning)
// =============================================================================

rcl_interfaces::msg::SetParametersResult MotorMove::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    // --- PID Gains ---
    if (name == "Kp" || name == "Ki" || name == "Kd") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        result.successful = false;
        result.reason = name + " must be a double array";
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid type for %s: expected double array",
                     name.c_str());
        return result;
      }

      auto values = param.as_double_array();
      if (values.size() != 9) {
        result.successful = false;
        result.reason = name + " must have exactly 9 elements (3x3 matrix)";
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid size for %s: got %zu, expected 9", name.c_str(),
                     values.size());
        return result;
      }

      Eigen::MatrixXd matrix =
          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::RowMajor>>(values.data(), 3, 3);

      {
        std::lock_guard lock{target_pose_mutex_};
        if (name == "Kp") {
          mimo_.set_Kp(matrix);
        } else if (name == "Ki") {
          mimo_.set_Ki(matrix);
        } else if (name == "Kd") {
          mimo_.set_Kd(matrix);
        }
      }

      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] %s updated to:\n%s",
                  name.c_str(), matrix_to_string(matrix).c_str());
    }

    // --- P-Term Limits ---
    if (name == "p_max_linear" || name == "p_max_angular") {
      double val = param.as_double();
      if (val < 0.0) {
        result.successful = false;
        result.reason = name + " must be non-negative";
        return result;
      }
      {
        std::lock_guard lock{target_pose_mutex_};
        double p_lin, p_ang;
        this->get_parameter("p_max_linear", p_lin);
        this->get_parameter("p_max_angular", p_ang);
        if (name == "p_max_linear")
          p_lin = val;
        if (name == "p_max_angular")
          p_ang = val;
        mimo_.set_p_term_limits(p_lin, p_ang);
      }
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] %s updated to: %f",
                  name.c_str(), val);
    }

    // --- Control parameters ---
    if (name == "loop_rate" || name == "timeout_seconds" ||
        name == "yaw_tolerance_degrees" || name == "distance_tolerance") {
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] %s updated to: %f",
                  name.c_str(), param.as_double());
    }

    // --- Decoupling ---
    if (name == "enable_decoupling") {
      enable_decoupling_ = param.as_bool();
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] Decoupling %s",
                  enable_decoupling_ ? "ENABLED" : "DISABLED");
    }

    if (name == "decoupling_matrix") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        result.successful = false;
        result.reason = "decoupling_matrix must be a double array";
        return result;
      }
      auto values = param.as_double_array();
      if (values.size() != 9) {
        result.successful = false;
        result.reason = "decoupling_matrix must have exactly 9 elements (3x3)";
        return result;
      }
      Eigen::MatrixXd matrix =
          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::RowMajor>>(values.data(), 3, 3);
      {
        std::lock_guard lock{target_pose_mutex_};
        decoupling_matrix_ = matrix;
      }
      RCLCPP_WARN(this->get_logger(),
                  "[LIVE TUNING] Decoupling matrix updated to:\n%s",
                  matrix_to_string(matrix).c_str());
    }

    // --- Feedforward ---
    if (name == "enable_feedforward") {
      enable_feedforward_ = param.as_bool();
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] Feedforward %s",
                  enable_feedforward_ ? "ENABLED" : "DISABLED");
    }

    if (name == "max_linear_velocity") {
      max_linear_velocity_ = param.as_double();
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] max_linear_velocity = %f",
                  max_linear_velocity_);
    }
    if (name == "max_linear_acceleration") {
      max_linear_acceleration_ = param.as_double();
      RCLCPP_WARN(this->get_logger(),
                  "[LIVE TUNING] max_linear_acceleration = %f",
                  max_linear_acceleration_);
    }
    if (name == "max_angular_velocity") {
      max_angular_velocity_ = param.as_double();
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] max_angular_velocity = %f",
                  max_angular_velocity_);
    }
    if (name == "max_angular_acceleration") {
      max_angular_acceleration_ = param.as_double();
      RCLCPP_WARN(this->get_logger(),
                  "[LIVE TUNING] max_angular_acceleration = %f",
                  max_angular_acceleration_);
    }

    // --- enable_live_tuning cannot be changed at runtime ---
    if (name == "enable_live_tuning") {
      result.successful = false;
      result.reason = "enable_live_tuning can only be set at launch time";
      RCLCPP_WARN(
          this->get_logger(),
          "Cannot change enable_live_tuning at runtime. Restart the node.");
      return result;
    }
  }

  return result;
}

// Destruktor
MotorMove::~MotorMove() { finalize_tuning_logging(); }

// =============================================================================
// KOORDINATENTRANSFORMATION
// =============================================================================

PoseStamped MotorMove::to_frame(const PoseStamped::SharedPtr point_ptr,
                                std::string frame_id) {
  PoseStamped point_out;
  try {
    tf_buffer_->transform(*point_ptr, point_out, frame_id);
    return point_out;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    throw ex;
  }
}

// =============================================================================
// ACTION-SERVER CALLBACKS
// =============================================================================

rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal requst with order x: %f y: %f",
              goal->motor_goal.pose.position.x,
              goal->motor_goal.pose.position.y);
  try {
    std::lock_guard lock{target_pose_mutex_};
    target_pose_ =
        to_frame(std::make_shared<PoseStamped>(goal->motor_goal), odom_frame_);
    target_pose_.header.stamp = rclcpp::Time(0);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&MotorMove::execute, this, _1), goal_handle}.detach();
}

// =============================================================================
// HILFSFUNKTION
// =============================================================================

inline float MotorMove::calculate_distance(const PoseStamped pose) {
  return std::sqrt(pose.pose.position.x * pose.pose.position.x +
                   pose.pose.position.y * pose.pose.position.y);
}

// =============================================================================
// REGELSCHLEIFE
// =============================================================================

void MotorMove::execute(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  MotorMoveAction::Feedback::SharedPtr feedback =
      std::make_shared<MotorMoveAction::Feedback>();
  MotorMoveAction::Result::SharedPtr result =
      std::make_shared<MotorMoveAction::Result>();
  float &distance = feedback->distance_to_target;

  double timeout_seconds;
  double loop_rate_hz;
  double yaw_tolerance_degrees;
  double distance_tolerance;

  this->get_parameter("timeout_seconds", timeout_seconds);
  this->get_parameter("loop_rate", loop_rate_hz);
  this->get_parameter("yaw_tolerance_degrees", yaw_tolerance_degrees);
  this->get_parameter("distance_tolerance", distance_tolerance);

  // Read feedforward params fresh from parameter server (works without live
  // tuning)
  this->get_parameter("enable_feedforward", enable_feedforward_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_angular_acceleration", max_angular_acceleration_);

  // Read decoupling params fresh
  this->get_parameter("enable_decoupling", enable_decoupling_);

  // Read PID gains fresh (works without live tuning callback)
  {
    Eigen::MatrixXd Kp_matrix = get_matrix_parameter("Kp", 3, 3);
    Eigen::MatrixXd Ki_matrix = get_matrix_parameter("Ki", 3, 3);
    Eigen::MatrixXd Kd_matrix = get_matrix_parameter("Kd", 3, 3);
    mimo_.set_Kp(Kp_matrix);
    mimo_.set_Ki(Ki_matrix);
    mimo_.set_Kd(Kd_matrix);

    double p_max_lin, p_max_ang;
    this->get_parameter("p_max_linear", p_max_lin);
    this->get_parameter("p_max_angular", p_max_ang);
    mimo_.set_p_term_limits(p_max_lin, p_max_ang);
  }

  rclcpp::Duration timeout_duration =
      rclcpp::Duration::from_seconds(timeout_seconds);
  const double YAW_TOLERANCE = yaw_tolerance_degrees * M_PI / 180.0;
  const double DISTANCE_TOLERANCE = distance_tolerance;

  rclcpp::Rate loop_rate(loop_rate_hz);
  rclcpp::Time start_time = this->now();
  rclcpp::Time current_time = this->now();
  rclcpp::Time previous_time = current_time;

  // PID Tuning Logging starten
  init_tuning_logging();

  // Reset PID state for new goal (prevents derivative spike)
  mimo_.reset();

  // Target-Pose für Logging (im odom-Frame)
  double target_x, target_y, target_yaw;
  {
    std::lock_guard lock{target_pose_mutex_};
    target_x = target_pose_.pose.position.x;
    target_y = target_pose_.pose.position.y;
    target_yaw = tf2::getYaw(target_pose_.pose.orientation);
  }

  // =========================================================================
  // TRAJECTORY SETUP (for feedforward mode)
  // =========================================================================
  double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;
  double direction_angle = 0.0;
  double linear_distance = 0.0;
  double angular_distance = 0.0;
  TrajectoryProfile linear_profile, angular_profile;

  if (enable_feedforward_) {
    // Get start position in odom frame
    geometry_msgs::msg::TransformStamped start_tf;
    try {
      start_tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                             tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get start position: %s",
                   ex.what());
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    start_x = start_tf.transform.translation.x;
    start_y = start_tf.transform.translation.y;
    start_yaw = tf2::getYaw(start_tf.transform.rotation);

    // Compute trajectory in odom frame
    double dx = target_x - start_x;
    double dy = target_y - start_y;
    angular_distance = target_yaw - start_yaw;

    // Normalize angular distance to [-pi, pi]
    while (angular_distance > M_PI)
      angular_distance -= 2.0 * M_PI;
    while (angular_distance < -M_PI)
      angular_distance += 2.0 * M_PI;

    linear_distance = std::sqrt(dx * dx + dy * dy);
    direction_angle = std::atan2(dy, dx);

    // Configure time-based profiles
    linear_profile.configure(linear_distance, max_linear_velocity_,
                             max_linear_acceleration_);
    angular_profile.configure(std::fabs(angular_distance),
                              max_angular_velocity_, max_angular_acceleration_);

    RCLCPP_INFO(this->get_logger(),
                "[TRAJECTORY] start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
                start_x, start_y, start_yaw, target_x, target_y, target_yaw);
    RCLCPP_INFO(this->get_logger(),
                "[TRAJECTORY] Linear: %.3fm in %.2fs (peak %.2f m/s), "
                "Angular: %.3frad in %.2fs",
                linear_distance, linear_profile.total_time(),
                max_linear_velocity_, angular_distance,
                angular_profile.total_time());
  }

  while (rclcpp::ok()) {
    // Cancel-Check
    if (goal_handle->is_canceling()) {
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      finalize_tuning_logging();
      return;
    }

    // Timeout-Check
    current_time = this->now();
    rclcpp::Duration elapsed = current_time - start_time;
    if (elapsed > timeout_duration) {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_->publish(stop_cmd);

      result->success = false;
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Goal timed out after %f seconds",
                  timeout_seconds);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      finalize_tuning_logging();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Execute goal");
    PoseStamped error =
        to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_);
    distance = calculate_distance(error);
    goal_handle->publish_feedback(feedback);

    float yaw = std::abs(tf2::getYaw(error.pose.orientation));

    if (yaw > YAW_TOLERANCE || distance > DISTANCE_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(),
                  "Distance to target: %f, Yaw error: %f (tolerance: %f)",
                  distance, yaw, YAW_TOLERANCE);

      rclcpp::Duration delta_t = current_time - previous_time;
      double dt = delta_t.seconds();
      if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / loop_rate_hz;
        RCLCPP_WARN(this->get_logger(),
                    "Invalid delta_t, using expected loop time: %f", dt);
      }

      RCLCPP_INFO(this->get_logger(), "Time delta %f", dt);
      Eigen::MatrixXd error_matrix(3, 1);
      error_matrix << error.pose.position.x, error.pose.position.y,
          tf2::getYaw(error.pose.orientation);

      // =====================================================================
      // Get current robot position in odom frame
      // =====================================================================
      geometry_msgs::msg::TransformStamped current_tf;
      try {
        current_tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                                 tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get current position: %s",
                    ex.what());
        loop_rate.sleep();
        continue;
      }

      double cur_x = current_tf.transform.translation.x;
      double cur_y = current_tf.transform.translation.y;
      double cur_yaw = tf2::getYaw(current_tf.transform.rotation);

      Eigen::MatrixXd position_matrix(3, 1);
      position_matrix << cur_x, cur_y, cur_yaw;

      double v_ff_x = 0.0, v_ff_y = 0.0, v_ff_yaw = 0.0;
      double pid_out_x = 0.0, pid_out_y = 0.0, pid_out_yaw = 0.0;
      double log_err_x, log_err_y, log_err_yaw;
      geometry_msgs::msg::Twist cmd_vel;

      if (enable_feedforward_) {
        // ===================================================================
        // TRAJECTORY-BASED FEEDFORWARD + TRACKING ERROR PID
        // ===================================================================
        double t = elapsed.seconds();
        double yaw_sign = (angular_distance >= 0.0) ? 1.0 : -1.0;

        // Reference from trajectory profile
        auto lin_state = linear_profile.compute(t);
        auto ang_state = angular_profile.compute(t);

        // Reference position in odom frame
        double ref_x = start_x + std::cos(direction_angle) * lin_state.position;
        double ref_y = start_y + std::sin(direction_angle) * lin_state.position;
        double ref_yaw = start_yaw + yaw_sign * ang_state.position;

        // Reference velocity in odom frame (feedforward)
        double ref_vx_odom = std::cos(direction_angle) * lin_state.velocity;
        double ref_vy_odom = std::sin(direction_angle) * lin_state.velocity;
        double ref_vyaw = yaw_sign * ang_state.velocity;

        // Tracking error in odom frame (should be SMALL - cm, not m)
        double track_err_x = ref_x - cur_x;
        double track_err_y = ref_y - cur_y;
        double track_err_yaw = ref_yaw - cur_yaw;
        while (track_err_yaw > M_PI)
          track_err_yaw -= 2.0 * M_PI;
        while (track_err_yaw < -M_PI)
          track_err_yaw += 2.0 * M_PI;

        // PID on tracking error (all in odom frame - consistent with D-term)
        Eigen::MatrixXd tracking_error(3, 1);
        tracking_error << track_err_x, track_err_y, track_err_yaw;

        // Apply decoupling if enabled
        if (enable_decoupling_) {
          tracking_error = decoupling_matrix_ * tracking_error;
        }

        Eigen::MatrixXd pid_output_odom =
            mimo_.compute(tracking_error, position_matrix, dt);

        // Total velocity in odom frame = feedforward + PID correction
        double total_vx_odom = ref_vx_odom + pid_output_odom(0, 0);
        double total_vy_odom = ref_vy_odom + pid_output_odom(1, 0);
        double total_vyaw = ref_vyaw + pid_output_odom(2, 0);

        // Transform from odom frame to base_link frame for cmd_vel
        double cos_yaw = std::cos(cur_yaw);
        double sin_yaw = std::sin(cur_yaw);

        cmd_vel.linear.x = cos_yaw * total_vx_odom + sin_yaw * total_vy_odom;
        cmd_vel.linear.y = -sin_yaw * total_vx_odom + cos_yaw * total_vy_odom;
        cmd_vel.angular.z = total_vyaw;

        // Output clamping: never exceed physical robot limits
        double lin_mag = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x +
                                   cmd_vel.linear.y * cmd_vel.linear.y);
        if (lin_mag > max_linear_velocity_ && lin_mag > 0.001) {
          double scale = max_linear_velocity_ / lin_mag;
          cmd_vel.linear.x *= scale;
          cmd_vel.linear.y *= scale;
        }
        cmd_vel.angular.z = std::clamp(
            cmd_vel.angular.z, -max_angular_velocity_, max_angular_velocity_);

        // Compute FF and PID in base_link for logging
        v_ff_x = cos_yaw * ref_vx_odom + sin_yaw * ref_vy_odom;
        v_ff_y = -sin_yaw * ref_vx_odom + cos_yaw * ref_vy_odom;
        v_ff_yaw = ref_vyaw;
        pid_out_x =
            cos_yaw * pid_output_odom(0, 0) + sin_yaw * pid_output_odom(1, 0);
        pid_out_y =
            -sin_yaw * pid_output_odom(0, 0) + cos_yaw * pid_output_odom(1, 0);
        pid_out_yaw = pid_output_odom(2, 0);

        // Log tracking error (what PID actually sees)
        log_err_x = track_err_x;
        log_err_y = track_err_y;
        log_err_yaw = track_err_yaw;

        RCLCPP_INFO(this->get_logger(),
                    "[TRAJ] t=%.2f ref=(%.3f,%.3f) cur=(%.3f,%.3f) "
                    "track_err=(%.4f,%.4f,%.4f)",
                    t, ref_x, ref_y, cur_x, cur_y, track_err_x, track_err_y,
                    track_err_yaw);
        RCLCPP_INFO(this->get_logger(),
                    "[TRAJ] ff=(%.3f,%.3f,%.3f) pid=(%.3f,%.3f,%.3f) "
                    "cmd=(%.3f,%.3f,%.3f)",
                    v_ff_x, v_ff_y, v_ff_yaw, pid_out_x, pid_out_y, pid_out_yaw,
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
      } else {
        // ===================================================================
        // LEGACY MODE: Pure PID on goal error (no feedforward)
        // ===================================================================
        Eigen::MatrixXd pid_input = error_matrix;
        if (enable_decoupling_) {
          pid_input = decoupling_matrix_ * error_matrix;
        }

        Eigen::MatrixXd pid_output =
            mimo_.compute(pid_input, position_matrix, dt);

        cmd_vel.linear.x = pid_output(0, 0);
        cmd_vel.linear.y = pid_output(1, 0);
        cmd_vel.angular.z = pid_output(2, 0);

        pid_out_x = pid_output(0, 0);
        pid_out_y = pid_output(1, 0);
        pid_out_yaw = pid_output(2, 0);
        log_err_x = error_matrix(0, 0);
        log_err_y = error_matrix(1, 0);
        log_err_yaw = error_matrix(2, 0);

        RCLCPP_INFO(this->get_logger(),
                    "Error: (%.3f, %.3f, %.3f) PID: (%.3f, %.3f, %.3f)",
                    log_err_x, log_err_y, log_err_yaw, pid_out_x, pid_out_y,
                    pid_out_yaw);
      }

      cmd_vel_->publish(cmd_vel);

      // PID Tuning Logging
      double timestamp = (current_time - start_time).seconds();
      log_pid_data(timestamp, log_err_x, log_err_y, log_err_yaw,
                   cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                   target_x, target_y, target_yaw, v_ff_x, v_ff_y, v_ff_yaw,
                   pid_out_x, pid_out_y, pid_out_yaw);

      previous_time = current_time;
    } else {
      // Ziel erreicht
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_->publish(stop_cmd);

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
                  "Ziel erreicht - Toleranz erfüllt (Yaw: %f <= %f, Distance: "
                  "%f <= %f)",
                  yaw, YAW_TOLERANCE, distance, DISTANCE_TOLERANCE);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw);
      RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f",
                  error.pose.position.x, error.pose.position.y);
      finalize_tuning_logging();
      return;
    }

    loop_rate.sleep();
  }

  result->success = false;
  goal_handle->abort(result);
  RCLCPP_WARN(this->get_logger(), "Goal execution ended without success");
  finalize_tuning_logging();
}
} // namespace motor_move

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_move::MotorMove>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
