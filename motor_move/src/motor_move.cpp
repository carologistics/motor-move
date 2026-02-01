// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/motor_move.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/utils.h"
#include <cmath>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <filesystem>  // NEU: für Verzeichnis-Erstellung
#include <chrono>      // NEU: für Timestamp
#include <iomanip>     // NEU: für Timestamp-Formatierung
#include <sstream>     // NEU: für Timestamp-Formatierung
#include <cstdlib>     // NEU: für system()

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

  // Timestamp für dieses Experiment generieren
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream oss;
  oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
  experiment_timestamp_ = oss.str();

  // Verzeichnis erstellen
  std::string dir_path = tuning_log_path_ + "/" + experiment_timestamp_;
  try {
    std::filesystem::create_directories(dir_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create tuning log directory: %s", e.what());
    logging_active_ = false;
    return;
  }

  // CSV-Datei öffnen
  std::string csv_path = dir_path + "/pid_data.csv";
  csv_file_.open(csv_path);

  if (!csv_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
    logging_active_ = false;
    return;
  }

  // CSV-Header schreiben
  csv_file_ << "timestamp,error_x,error_y,error_yaw,cmd_vel_x,cmd_vel_y,cmd_vel_yaw,target_x,target_y,target_yaw\n";
  csv_file_.flush();

  logging_active_ = true;
  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Logging started: %s", csv_path.c_str());
}

void MotorMove::log_pid_data(double timestamp, double error_x, double error_y, double error_yaw,
                              double cmd_vel_x, double cmd_vel_y, double cmd_vel_yaw,
                              double target_x, double target_y, double target_yaw) {
  if (!logging_active_ || !csv_file_.is_open()) {
    return;
  }

  csv_file_ << std::fixed << std::setprecision(6)
            << timestamp << ","
            << error_x << ","
            << error_y << ","
            << error_yaw << ","
            << cmd_vel_x << ","
            << cmd_vel_y << ","
            << cmd_vel_yaw << ","
            << target_x << ","
            << target_y << ","
            << target_yaw << "\n";
}

void MotorMove::generate_plot() {
  std::string local_dir = tuning_log_path_ + "/" + experiment_timestamp_;
  std::string csv_path = local_dir + "/pid_data.csv";
  std::string png_path = local_dir + "/pid_analysis.png";

  // Pfad zum Plot-Skript (relativ zum Package)
  std::string script_path = "/home/robotino/ros2/robotino_navigation_ws/src/motor-move/motor_move/scripts/plot_pid_data.py";

  // Plot-Befehl zusammenbauen
  std::string plot_cmd = "python3 " + script_path + " " + csv_path + " " + png_path + " 2>/dev/null";

  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Generating plot...");

  int result = std::system(plot_cmd.c_str());

  if (result == 0) {
    RCLCPP_INFO(this->get_logger(), "[PID TUNING] Plot saved: %s", png_path.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "[PID TUNING] Plot generation failed (exit code: %d)", result);
  }
}

void MotorMove::transfer_to_remote() {
  if (tuning_remote_target_.empty()) {
    return;
  }

  // Lokaler Pfad
  std::string local_dir = tuning_log_path_ + "/" + experiment_timestamp_;

  // Remote-Zielverzeichnis erstellen und Dateien kopieren
  // Format von tuning_remote_target_: "user@host:/path"
  size_t colon_pos = tuning_remote_target_.find(':');
  if (colon_pos == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "[PID TUNING] Invalid remote target format: %s (expected user@host:/path)",
                 tuning_remote_target_.c_str());
    return;
  }

  std::string user_host = tuning_remote_target_.substr(0, colon_pos);
  std::string remote_base_path = tuning_remote_target_.substr(colon_pos + 1);
  std::string remote_full_path = remote_base_path + "/" + experiment_timestamp_;

  // Verzeichnis auf Remote erstellen
  std::string mkdir_cmd = "ssh -o ConnectTimeout=5 -o BatchMode=yes " + user_host + " 'mkdir -p " + remote_full_path + "' 2>/dev/null";
  int mkdir_result = std::system(mkdir_cmd.c_str());

  if (mkdir_result != 0) {
    RCLCPP_WARN(this->get_logger(), "[PID TUNING] Failed to create remote directory (exit code: %d). Trying scp anyway...",
                mkdir_result);
  }

  // Ganzen Ordner kopieren (CSV + PNG)
  std::string scp_cmd = "scp -o ConnectTimeout=5 -o BatchMode=yes -r " +
                        local_dir + "/* " +
                        user_host + ":" + remote_full_path + "/ 2>/dev/null &";

  RCLCPP_INFO(this->get_logger(), "[PID TUNING] Transferring to remote: %s:%s", user_host.c_str(), remote_full_path.c_str());

  // SCP im Hintergrund ausführen (blockiert nicht)
  int result = std::system(scp_cmd.c_str());

  if (result == 0) {
    RCLCPP_INFO(this->get_logger(), "[PID TUNING] Transfer initiated (CSV + PNG)");
  } else {
    RCLCPP_WARN(this->get_logger(), "[PID TUNING] SCP command returned non-zero. Check SSH key authentication.");
  }
}

void MotorMove::finalize_tuning_logging() {
  if (!logging_active_) {
    return;
  }

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
    RCLCPP_INFO(this->get_logger(), "[PID TUNING] Logging finished: %s/%s/pid_data.csv",
                tuning_log_path_.c_str(), experiment_timestamp_.c_str());
  }

  // Plot generieren
  generate_plot();

  // Transfer zu Remote wenn konfiguriert
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
  RCLCPP_INFO(this->get_logger(), "base_link frame id: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom frame id: %s", odom_frame_.c_str());

  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, _1, _2),
      std::bind(&MotorMove::handle_cancel, this, _1),
      std::bind(&MotorMove::handle_accepted, this, _1));

  // --- PID Gains ---
  std::vector<double> default_Kp = {1.8, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 1.8};
  std::vector<double> default_Ki = {0.38, 0.0, 0.0, 0.0, 0.38, 0.0, 0.0, 0.0, 0.38};
  std::vector<double> default_Kd = {0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2};

  this->declare_parameter("Kp", default_Kp);
  this->declare_parameter("Ki", default_Ki);
  this->declare_parameter("Kd", default_Kd);

  Eigen::MatrixXd Kp_matrix = get_matrix_parameter("Kp", 3, 3);
  Eigen::MatrixXd Ki_matrix = get_matrix_parameter("Ki", 3, 3);
  Eigen::MatrixXd Kd_matrix = get_matrix_parameter("Kd", 3, 3);

  RCLCPP_INFO(this->get_logger(), "Kp:\n%s", matrix_to_string(Kp_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Ki:\n%s", matrix_to_string(Ki_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Kd:\n%s", matrix_to_string(Kd_matrix).c_str());

  mimo_.set_Kp(Kp_matrix);
  mimo_.set_Ki(Ki_matrix);
  mimo_.set_Kd(Kd_matrix);

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
  RCLCPP_INFO(this->get_logger(), "  Distance tolerance: %f meters", dist_tol_val);

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
    RCLCPP_WARN(this->get_logger(),
                "=== PID TUNING LOGGING ENABLED ===");
    RCLCPP_WARN(this->get_logger(),
                "CSV logs will be saved to: %s/<timestamp>/pid_data.csv", tuning_log_path_.c_str());

    if (!tuning_remote_target_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Remote transfer enabled: %s/<timestamp>/pid_data.csv", tuning_remote_target_.c_str());
    }
  }

  // =========================================================================
  // LIVE TUNING FEATURE
  // =========================================================================
  // Wenn enable_live_tuning=true, werden Änderungen an Kp, Ki, Kd
  // sofort auf den MIMO-Regler angewendet (per ros2 param set).
  // Kann beim Launch aktiviert werden:
  //   ros2 launch ... enable_live_tuning:=true
  //
  // Oder per CLI:
  //   ros2 run motor_move motor_move --ros-args -p enable_live_tuning:=true
  //
  // Im Normalbetrieb (Produktion) sollte das deaktiviert bleiben,
  // da der Parameter-Callback bei jedem Parameter-Change aufgerufen wird.
  // =========================================================================

  this->declare_parameter("enable_live_tuning", false);
  bool live_tuning_enabled = false;
  this->get_parameter("enable_live_tuning", live_tuning_enabled);

  if (live_tuning_enabled) {
    RCLCPP_WARN(this->get_logger(),
                "=== LIVE TUNING MODE ENABLED ===");
    RCLCPP_WARN(this->get_logger(),
                "PID gains (Kp, Ki, Kd) and control parameters can be changed at runtime.");
    RCLCPP_WARN(this->get_logger(),
                "Use: ros2 param set <node> Kp \"[1.8, 0, 0, 0, 1.8, 0, 0, 0, 1.8]\"");

    // Parameter-Callback registrieren
    // Wird aufgerufen bei JEDER Parameter-Änderung an dieser Node
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MotorMove::on_parameter_change, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Live tuning disabled. Launch with enable_live_tuning:=true to enable.");
  }
}

// =============================================================================
// PARAMETER CHANGE CALLBACK (Live Tuning)
// =============================================================================
/*
 * Wird aufgerufen wenn ein Parameter per "ros2 param set" geändert wird.
 * Prüft ob es sich um PID-Gains handelt und aktualisiert den MIMO-Regler.
 *
 * Beispiel:
 *   ros2 param set /robotino1/motor_move Kp "[2.0, 0, 0, 0, 2.0, 0, 0, 0, 2.0]"
 *   -> Callback feuert, erkennt "Kp", baut 3x3 Matrix, setzt mimo_.set_Kp()
 */
rcl_interfaces::msg::SetParametersResult
MotorMove::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    // --- PID Gains ---
    if (name == "Kp" || name == "Ki" || name == "Kd") {
      // Validierung: Muss ein double-Array mit genau 9 Elementen sein (3x3)
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        result.successful = false;
        result.reason = name + " must be a double array";
        RCLCPP_ERROR(this->get_logger(), "Invalid type for %s: expected double array",
                     name.c_str());
        return result;
      }

      auto values = param.as_double_array();
      if (values.size() != 9) {
        result.successful = false;
        result.reason = name + " must have exactly 9 elements (3x3 matrix)";
        RCLCPP_ERROR(this->get_logger(), "Invalid size for %s: got %zu, expected 9",
                     name.c_str(), values.size());
        return result;
      }

      // Vektor in 3x3 Eigen-Matrix umwandeln
      Eigen::MatrixXd matrix = Eigen::Map<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
          values.data(), 3, 3);

      // Auf den MIMO-Regler anwenden (thread-safe durch Mutex)
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

    // --- Andere Parameter loggen (werden in execute() ohnehin neu gelesen) ---
    if (name == "loop_rate" || name == "timeout_seconds" ||
        name == "yaw_tolerance_degrees" || name == "distance_tolerance") {
      RCLCPP_WARN(this->get_logger(), "[LIVE TUNING] %s updated to: %f",
                   name.c_str(), param.as_double());
    }

    // --- enable_live_tuning selbst kann nicht zur Laufzeit geändert werden ---
    if (name == "enable_live_tuning") {
      result.successful = false;
      result.reason = "enable_live_tuning can only be set at launch time";
      RCLCPP_WARN(this->get_logger(),
                  "Cannot change enable_live_tuning at runtime. Restart the node.");
      return result;
    }
  }

  return result;
}

// Destruktor
MotorMove::~MotorMove() {
  // Sicherstellen dass CSV geschlossen wird
  finalize_tuning_logging();
}

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

  rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout_seconds);
  const double YAW_TOLERANCE = yaw_tolerance_degrees * M_PI / 180.0;
  const double DISTANCE_TOLERANCE = distance_tolerance;

  rclcpp::Rate loop_rate(loop_rate_hz);
  rclcpp::Time start_time = this->now();
  rclcpp::Time current_time = this->now();
  rclcpp::Time previous_time = current_time;

  // PID Tuning Logging starten
  init_tuning_logging();

  // Target-Pose für Logging (im odom-Frame)
  double target_x, target_y, target_yaw;
  {
    std::lock_guard lock{target_pose_mutex_};
    target_x = target_pose_.pose.position.x;
    target_y = target_pose_.pose.position.y;
    target_yaw = tf2::getYaw(target_pose_.pose.orientation);
  }

  while (rclcpp::ok()) {
    // Cancel-Check
    if (goal_handle->is_canceling()) {
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      finalize_tuning_logging();  // Logging beenden
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
      RCLCPP_WARN(this->get_logger(), "Goal timed out after %f seconds", timeout_seconds);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      finalize_tuning_logging();  // Logging beenden
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Execute goal");
    PoseStamped error =
        to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_);
    distance = calculate_distance(error);
    goal_handle->publish_feedback(feedback);

    float yaw = std::abs(tf2::getYaw(error.pose.orientation));

    if (yaw > YAW_TOLERANCE || distance > DISTANCE_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f, Yaw error: %f (tolerance: %f)",
                  distance, yaw, YAW_TOLERANCE);

      rclcpp::Duration delta_t = current_time - previous_time;
      double dt = delta_t.seconds();
      if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / loop_rate_hz;
        RCLCPP_WARN(this->get_logger(), "Invalid delta_t, using expected loop time: %f", dt);
      }

      RCLCPP_INFO(this->get_logger(), "Time delta %f", dt);
      Eigen::MatrixXd error_matrix(3, 1);
      error_matrix << error.pose.position.x, error.pose.position.y, tf2::getYaw(error.pose.orientation);

      // mimo_.compute() nutzt die aktuellen Kp/Ki/Kd
      // Bei aktivem Live-Tuning wurden diese ggf. durch den
      // Parameter-Callback bereits aktualisiert
      Eigen::MatrixXd output = mimo_.compute(error_matrix, dt);

      RCLCPP_INFO(this->get_logger(), "Error matrix - x: %f, y: %f, yaw: %f",
                  error_matrix(0, 0), error_matrix(1, 0), error_matrix(2, 0));
      RCLCPP_INFO(this->get_logger(), "PID Output - x: %f, y: %f, yaw: %f",
                  output(0, 0), output(1, 0), output(2, 0));

      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = output(0, 0);
      cmd_vel.linear.y = output(1, 0);
      cmd_vel.angular.z = output(2, 0);
      cmd_vel_->publish(cmd_vel);

      // PID Tuning Logging
      double timestamp = (current_time - start_time).seconds();
      log_pid_data(timestamp,
                   error_matrix(0, 0), error_matrix(1, 0), error_matrix(2, 0),
                   cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                   target_x, target_y, target_yaw);

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
                  "Ziel erreicht - Toleranz erfüllt (Yaw: %f <= %f, Distance: %f <= %f)",
                  yaw, YAW_TOLERANCE, distance, DISTANCE_TOLERANCE);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw);
      RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f", error.pose.position.x,
                  error.pose.position.y);
      finalize_tuning_logging();  // Logging beenden
      return;
    }

    loop_rate.sleep();
  }

  result->success = false;
  goal_handle->abort(result);
  RCLCPP_WARN(this->get_logger(), "Goal execution ended without success");
  finalize_tuning_logging();  // Logging beenden
}
} // namespace motor_move

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_move::MotorMove>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
