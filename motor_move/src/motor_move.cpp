// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/motor_move.hpp" // Include the header file for the MotorMove class.
#include <eigen3/Eigen/src/Core/Matrix.h> // Include Eigen for matrix operations.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // JW
#include "tf2/utils.h"
#include <cmath> // For M_PI constant

namespace motor_move {

// Set a parameter as a flattened matrix
void MotorMove::set_matrix_parameter(const std::string &name,
                                     const Eigen::MatrixXd &matrix) {
  std::vector<double> flat_matrix(matrix.data(), matrix.data() + matrix.size()); // Flatten Eigen matrix to a vector: flat_matrix(begin,end)
  this->declare_parameter(name, rclcpp::ParameterValue(flat_matrix)); // rclcpp := "ros client library" assigns parameter "name" the parameter value of flat_matrix
}

//Retrieve a matrix parameter and reshape it into a proper Eigen matrix
Eigen::MatrixXd MotorMove::get_matrix_parameter(const std::string &name,
                                                int rows, int cols) {
  std::vector<double> flat_matrix; // Vector to store the parameter.
  this->get_parameter(name, flat_matrix); // Retrieve the parameter as a vector.
  Eigen::MatrixXd matrix = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      flat_matrix.data(), rows, cols); // Map the vector back to a matrix.
  return matrix;
}

// Convert an Eigen matrix to a string for logging or debugging
std::string MotorMove::matrix_to_string(const Eigen::MatrixXd &matrix) {
  std::ostringstream ss; // String stream for conversion.
  ss << matrix; // Insert matrix data into the stream.
  return ss.str(); // Return the string representation.
}

// Constructor for MotorMove class
MotorMove::MotorMove(const rclcpp::NodeOptions &options)
    : Node("motor_move", options), mimo_() { // Initialize the node with the name "motor_move".

  namespace_ = this->get_namespace(); // Retrieve the namespace of the node.
  std::string odom_frame = "odom"; // Default odometry frame.
  std::string base_frame = "base_link"; // Default base frame.

  if (namespace_ != "/") { // Check if namespace is not root.
    if (!namespace_.empty() && namespace_[0] == '/') { // Remove leading '/' if it exists.
      namespace_ = namespace_.substr(1);
    }
    odom_frame = namespace_ + "/" + odom_frame; // Prefix namespace to odometry frame.
    base_frame = namespace_ + "/" + base_frame; // Prefix namespace to base frame.

  }

  base_frame_ = base_frame; // Assign base frame to class variable.
  odom_frame_ = odom_frame; // Assign odometry frame to class variable.
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str()); // Log the namespace.
  RCLCPP_INFO(this->get_logger(), "base_link frame id: %s",
              base_frame_.c_str()); // Log base frame.
  RCLCPP_INFO(this->get_logger(), "odom frame id: %s", odom_frame_.c_str()); // Log odometry frame.

  // Create a publisher for cmd_vel topic with a queue size of 10.
  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  using namespace std::placeholders; // Simplify placeholder usage for callbacks.

  // Create an action server for motor_move_action. Here the goal inputs get read
  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, _1, _2), // Callback for goal reception.
      std::bind(&MotorMove::handle_cancel, this, _1), // Callback for cancel requests.
      std::bind(&MotorMove::handle_accepted, this, _1)); // Callback for accepted goals.

  // Declare and retrieve PID control gains from parameters
  // Default values (will be used if parameter file doesn't specify them)
  std::vector<double> default_Kp = {1.8, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 1.8};
  std::vector<double> default_Ki = {0.38, 0.0, 0.0, 0.0, 0.38, 0.0, 0.0, 0.0, 0.38};
  std::vector<double> default_Kd = {0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2};
  
  this->declare_parameter("Kp", default_Kp);
  this->declare_parameter("Ki", default_Ki);
  this->declare_parameter("Kd", default_Kd);

  // Retrieve control gains from parameters.
  Eigen::MatrixXd Kp_matrix = get_matrix_parameter("Kp", 3, 3);
  Eigen::MatrixXd Ki_matrix = get_matrix_parameter("Ki", 3, 3);
  Eigen::MatrixXd Kd_matrix = get_matrix_parameter("Kd", 3, 3);

  // Log the retrieved control matrices.
  RCLCPP_INFO(this->get_logger(), "Kp:\n%s",
              matrix_to_string(Kp_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Ki:\n%s",
              matrix_to_string(Ki_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Kd:\n%s",
              matrix_to_string(Kd_matrix).c_str());

  // Set control gains in the MIMO controller.
  mimo_.set_Kp(Kp_matrix);
  mimo_.set_Ki(Ki_matrix);
  mimo_.set_Kd(Kd_matrix);
  
  RCLCPP_INFO(this->get_logger(), "Using Kp:\n%s", matrix_to_string(Kp_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Using Ki:\n%s", matrix_to_string(Ki_matrix).c_str());
  RCLCPP_INFO(this->get_logger(), "Using Kd:\n%s", matrix_to_string(Kd_matrix).c_str());

  // Initialize TF2 buffer and listener for transformations.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Declare control parameters
  this->declare_parameter("loop_rate", 15.0);
  this->declare_parameter("timeout_seconds", 10.0);
  this->declare_parameter("yaw_tolerance_degrees", 5.0);
  this->declare_parameter("distance_tolerance", 0.05);
  
  // Log loaded parameters
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
}

// Destructor for MotorMove class
MotorMove::~MotorMove() {}

// Transform a pose to a specified frame
PoseStamped MotorMove::to_frame(const PoseStamped::SharedPtr point_ptr,
                                std::string frame_id) {
  PoseStamped point_out; // Output pose.
  try {
    tf_buffer_->transform(*point_ptr, point_out, frame_id); // Perform the transformation.
    return point_out;
  } catch (const tf2::TransformException &ex) { // Handle transformation errors.
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    throw ex; // Re-throw exception.
  }
}

// Handle incoming goals
rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid; // Silence unused variable warning.
  RCLCPP_INFO(this->get_logger(), "Received goal requst with order x: %f y: %f",
              goal->motor_goal.pose.position.x,
              goal->motor_goal.pose.position.y); // Log goal details.
  try {
    std::lock_guard lock{target_pose_mutex_}; // Lock the mutex for thread safety.
    target_pose_ =
        to_frame(std::make_shared<PoseStamped>(goal->motor_goal), odom_frame_); // Transform target pose to odom frame.
    target_pose_.header.stamp = rclcpp::Time(0); // Set timestamp to zero.
  } catch (tf2::TransformException &ex) { // Handle transformation errors.
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return rclcpp_action::GoalResponse::REJECT; // Reject the goal.
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal.
}

// Handle goal cancellation requests
rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  (void)goal_handle; // Silence unused variable warning.
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT; // Accept cancellation.
}

// Handle accepted goals and start execution
void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  using namespace std::placeholders; // Simplify placeholder usage.
  std::thread{std::bind(&MotorMove::execute, this, _1), goal_handle}.detach(); // Start goal execution in a new thread.
}

// Calculate distance to a target pose
inline float MotorMove::calculate_distance(const PoseStamped pose) {
  return std::sqrt(pose.pose.position.x * pose.pose.position.x +
                   pose.pose.position.y * pose.pose.position.y); // error? Should multiply z by itself.
}

// // Convert a quaternion to yaw angle
// double MotorMove::quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
//   tf2::Quaternion tf2_quat; // TF2 quaternion.
//   tf2::convert(q, tf2_quat); // Convert geometry_msgs quaternion to TF2 quaternion.
//   tf2::Matrix3x3 m(tf2_quat); // Create a rotation matrix from the quaternion.
//   double roll, pitch, yaw; // Variables to store roll, pitch, and yaw.
//   m.getRPY(roll, pitch, yaw); // Extract roll, pitch, and yaw.
//   return yaw; // Return yaw angle.
// }

// Execute the goal
void MotorMove::execute(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  MotorMoveAction::Feedback::SharedPtr feedback =
      std::make_shared<MotorMoveAction::Feedback>(); // Create feedback object.
  MotorMoveAction::Result::SharedPtr result =
      std::make_shared<MotorMoveAction::Result>(); // Create result object.
  float &distance = feedback->distance_to_target; // Reference to distance feedback.
  
  // Get parameters from node
  double timeout_seconds;
  double loop_rate_hz;
  double yaw_tolerance_degrees;
  double distance_tolerance;
  
  this->get_parameter("timeout_seconds", timeout_seconds);
  this->get_parameter("loop_rate", loop_rate_hz);
  this->get_parameter("yaw_tolerance_degrees", yaw_tolerance_degrees);
  this->get_parameter("distance_tolerance", distance_tolerance);
  
  rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout_seconds);
  
  // Convert yaw tolerance from degrees to radians
  const double YAW_TOLERANCE = yaw_tolerance_degrees * M_PI / 180.0;
  const double DISTANCE_TOLERANCE = distance_tolerance;
  
  rclcpp::Rate loop_rate(loop_rate_hz); // Set loop rate from parameter.
  rclcpp::Time start_time = this->now(); // Record start time for timeout.
  rclcpp::Time current_time = this->now(); // Get current time.
  rclcpp::Time previous_time = current_time; // Initialize previous time for delta_t calculation.
  
  while (rclcpp::ok()) { // Main loop.
    // Check for cancellation
    if (goal_handle->is_canceling()) {
      goal_handle->publish_feedback(feedback); // Publish feedback.
      result->success = false; // Set failure state.
      goal_handle->canceled(result); // Cancel the goal.
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return; // Exit the loop.
    }
    
    // Update current time and check for timeout
    current_time = this->now();
    rclcpp::Duration elapsed = current_time - start_time;
    if (elapsed > timeout_duration) {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_->publish(stop_cmd);
      
      result->success = false; // Set failure state.
      goal_handle->abort(result); // Abort with timeout.
      RCLCPP_WARN(this->get_logger(), "Goal timed out after %f seconds", timeout_seconds);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      return; // Exit the loop.
    }
    
    RCLCPP_INFO(this->get_logger(), "Execute goal");
    PoseStamped error =
        to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_); // Transform target pose to base frame.
    distance = calculate_distance(error); // Calculate distance to target.
    goal_handle->publish_feedback(feedback); // Publish feedback.
    
    float yaw = std::abs(tf2::getYaw(error.pose.orientation)); // Calculate absolute yaw to target.
    
    // Check if within tolerance (5° for yaw, 5cm for distance)
    if (yaw > YAW_TOLERANCE || distance > DISTANCE_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f, Yaw error: %f (tolerance: %f)", 
                  distance, yaw, YAW_TOLERANCE);
      
      // Calculate time difference for PID controller
      rclcpp::Duration delta_t = current_time - previous_time;
      
      // Ensure minimum delta_t to avoid division by zero or very large derivatives
      double dt = delta_t.seconds();
      if (dt <= 0.0 || dt > 1.0) {
        // If delta_t is invalid, use expected loop time
        dt = 1.0 / loop_rate_hz;
        RCLCPP_WARN(this->get_logger(), "Invalid delta_t, using expected loop time: %f", dt);
      }
      
      RCLCPP_INFO(this->get_logger(), "Time delta %f", dt);
      Eigen::MatrixXd error_matrix(3, 1); // Create error matrix.
      error_matrix << error.pose.position.x, error.pose.position.y, tf2::getYaw(error.pose.orientation); // Fill error matrix (signed yaw).
      Eigen::MatrixXd output = mimo_.compute(error_matrix, dt); // Compute control output.
      
      // Enhanced debugging for x vs y axis issue
      RCLCPP_INFO(this->get_logger(), "Error matrix - x: %f, y: %f, yaw: %f", 
                  error_matrix(0, 0), error_matrix(1, 0), error_matrix(2, 0));
      RCLCPP_INFO(this->get_logger(), "PID Output - x: %f, y: %f, yaw: %f", 
                  output(0, 0), output(1, 0), output(2, 0));
      
      geometry_msgs::msg::Twist cmd_vel; // Create Twist message for velocity commands.
      cmd_vel.linear.x = output(0, 0); // Set linear x velocity.
      cmd_vel.linear.y = output(1, 0); // Set linear y velocity.
      cmd_vel.angular.z = output(2, 0); // Set angular velocity.
      cmd_vel_->publish(cmd_vel); // Publish velocity command.
      
      // Update previous_time for next iteration
      previous_time = current_time;
    } else {
      // Tolerance reached - success!
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_->publish(stop_cmd);
      
      result->success = true; // Set success state.
      goal_handle->succeed(result); // Succeed with result.
      RCLCPP_INFO(this->get_logger(), "Ziel erreicht - Toleranz erfüllt (Yaw: %f <= %f, Distance: %f <= %f)", 
                  yaw, YAW_TOLERANCE, distance, DISTANCE_TOLERANCE);
      RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
      RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw);
      RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f", error.pose.position.x,
                  error.pose.position.y);
      return;
    }
    
    loop_rate.sleep(); // Sleep to maintain loop rate.
  }
  
  // If we exit the loop without success, set failure
  result->success = false;
  goal_handle->abort(result);
  RCLCPP_WARN(this->get_logger(), "Goal execution ended without success");
}
} // namespace motor_move

// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // Initialize ROS2.
  auto node = std::make_shared<motor_move::MotorMove>(); // Create MotorMove node.
  rclcpp::spin(node); // Spin the node to process callbacks.
  rclcpp::shutdown(); // Shutdown ROS2.
}
