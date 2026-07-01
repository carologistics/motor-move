#pragma once

#include "motor_move_rviz_plugin/plot_widget.hpp"

#include <motor_move_msgs/action/motor_move.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include <vector>
#include <mutex>

namespace motor_move_rviz_plugin {

class MotorMovePanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit MotorMovePanel(QWidget *parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void updatePreview();
  void scheduleParameterUpdate();
  void sendParameters();
  void sendGoal();

private:
  using MotorMoveAction = motor_move_msgs::action::MotorMove;
  using GoalHandleMotorMove =
      rclcpp_action::ClientGoalHandle<MotorMoveAction>;

  void buildUi();
  void setupRos();
  void refreshParameterClient();
  void refreshOdomSubscription();
  void setStatus(const QString &text, bool ok);
  void resetTraveledDistance();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void appendRealPoint(double time, double speed, double distance);
  double rotationTime(double yaw_error, double max_speed,
                      double acceleration, double kp) const;
  std::vector<QPointF> makeIdealSpeed(double distance, double yaw_error,
                                      double max_linear_speed,
                                      double linear_acceleration,
                                      double max_angular_speed,
                                      double angular_acceleration,
                                      double linear_kp, double angular_kp) const;
  std::vector<QPointF> makeIdealError(double distance, double yaw_error,
                                      double max_linear_speed,
                                      double linear_acceleration,
                                      double max_angular_speed,
                                      double angular_acceleration,
                                      double linear_kp, double angular_kp) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MotorMoveAction>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  QString current_parameter_node_;
  QString current_odom_topic_;

  QLineEdit *action_name_edit_;
  QLineEdit *parameter_node_edit_;
  QLineEdit *odom_topic_edit_;
  QLineEdit *frame_edit_;
  QDoubleSpinBox *x_spin_;
  QDoubleSpinBox *y_spin_;
  QDoubleSpinBox *yaw_spin_;
  QDoubleSpinBox *max_linear_speed_spin_;
  QDoubleSpinBox *linear_acceleration_spin_;
  QDoubleSpinBox *max_angular_speed_spin_;
  QDoubleSpinBox *angular_acceleration_spin_;
  QDoubleSpinBox *linear_kp_spin_;
  QDoubleSpinBox *angular_kp_spin_;
  QLabel *parameter_status_;
  QLabel *traveled_distance_label_;
  QPushButton *send_button_;
  PlotWidget *speed_plot_;
  PlotWidget *error_plot_;
  QTimer *parameter_timer_;

  std::vector<QPointF> real_speed_;
  std::vector<QPointF> real_error_;

  std::mutex odom_mutex_;
  bool tracking_distance_ = false;
  bool have_last_odom_position_ = false;
  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  double traveled_distance_ = 0.0;
};

} // namespace motor_move_rviz_plugin
