#pragma once

#include "motor_move_rviz_plugin/plot_widget.hpp"

#include <motor_move_msgs/action/motor_move.hpp>
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
  void setStatus(const QString &text, bool ok);
  void appendRealPoint(double time, double speed, double distance);
  std::vector<QPointF> makeIdealSpeed(double distance, double max_speed,
                                      double acceleration) const;
  std::vector<QPointF> makeIdealError(double distance, double max_speed,
                                      double acceleration) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MotorMoveAction>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
  QString current_parameter_node_;

  QLineEdit *action_name_edit_;
  QLineEdit *parameter_node_edit_;
  QLineEdit *frame_edit_;
  QDoubleSpinBox *x_spin_;
  QDoubleSpinBox *y_spin_;
  QDoubleSpinBox *yaw_spin_;
  QDoubleSpinBox *max_speed_spin_;
  QDoubleSpinBox *acceleration_spin_;
  QLabel *parameter_status_;
  QPushButton *send_button_;
  PlotWidget *speed_plot_;
  PlotWidget *error_plot_;
  QTimer *parameter_timer_;

  std::vector<QPointF> real_speed_;
  std::vector<QPointF> real_error_;
};

} // namespace motor_move_rviz_plugin
