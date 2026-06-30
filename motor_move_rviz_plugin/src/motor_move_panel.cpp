#include "motor_move_rviz_plugin/motor_move_panel.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

namespace motor_move_rviz_plugin {
namespace {

double brakingSpeed(double error, double max_speed, double acceleration) {
  if (error <= 0.0 || max_speed <= 0.0 || acceleration <= 0.0) {
    return 0.0;
  }
  return std::min(max_speed, std::sqrt(2.0 * acceleration * error));
}

double rampToward(double current, double target, double max_delta) {
  if (target > current) {
    return std::min(target, current + max_delta);
  }
  return std::max(target, current - max_delta);
}

} // namespace

MotorMovePanel::MotorMovePanel(QWidget *parent) : rviz_common::Panel(parent) {
  buildUi();
}

void MotorMovePanel::buildUi() {
  action_name_edit_ = new QLineEdit("/motor_move_action");
  parameter_node_edit_ = new QLineEdit("/motor_move");
  frame_edit_ = new QLineEdit("base_link");

  x_spin_ = new QDoubleSpinBox();
  y_spin_ = new QDoubleSpinBox();
  yaw_spin_ = new QDoubleSpinBox();
  max_speed_spin_ = new QDoubleSpinBox();
  acceleration_spin_ = new QDoubleSpinBox();

  for (auto *spin : {x_spin_, y_spin_}) {
    spin->setRange(-1000.0, 1000.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.05);
    spin->setSuffix(" m");
  }
  yaw_spin_->setRange(-360.0, 360.0);
  yaw_spin_->setDecimals(1);
  yaw_spin_->setSingleStep(1.0);
  yaw_spin_->setSuffix(" deg");

  max_speed_spin_->setRange(0.001, 10.0);
  max_speed_spin_->setDecimals(3);
  max_speed_spin_->setSingleStep(0.05);
  max_speed_spin_->setValue(0.5);
  max_speed_spin_->setSuffix(" m/s");

  acceleration_spin_->setRange(0.001, 10.0);
  acceleration_spin_->setDecimals(3);
  acceleration_spin_->setSingleStep(0.05);
  acceleration_spin_->setValue(0.5);
  acceleration_spin_->setSuffix(" m/s^2");

  parameter_status_ = new QLabel("-");
  send_button_ = new QPushButton("Send");
  speed_plot_ = new PlotWidget();
  error_plot_ = new PlotWidget();
  parameter_timer_ = new QTimer(this);
  parameter_timer_->setSingleShot(true);
  parameter_timer_->setInterval(200);

  speed_plot_->setTitle("Speed", "speed");
  error_plot_->setTitle("Position Error", "error");

  auto *connection_group = new QGroupBox("Connection");
  auto *connection_layout = new QFormLayout(connection_group);
  connection_layout->addRow("Action", action_name_edit_);
  connection_layout->addRow("Param node", parameter_node_edit_);

  auto *target_group = new QGroupBox("Target");
  auto *target_layout = new QFormLayout(target_group);
  target_layout->addRow("Frame", frame_edit_);
  target_layout->addRow("X", x_spin_);
  target_layout->addRow("Y", y_spin_);
  target_layout->addRow("Rot", yaw_spin_);

  auto *motion_group = new QGroupBox("Motion");
  auto *motion_layout = new QGridLayout(motion_group);
  motion_layout->addWidget(new QLabel("Max speed"), 0, 0);
  motion_layout->addWidget(max_speed_spin_, 0, 1);
  motion_layout->addWidget(new QLabel("Acceleration"), 1, 0);
  motion_layout->addWidget(acceleration_spin_, 1, 1);
  motion_layout->addWidget(parameter_status_, 2, 0);
  motion_layout->addWidget(send_button_, 2, 1);

  auto *layout = new QVBoxLayout(this);
  layout->addWidget(connection_group);
  layout->addWidget(target_group);
  layout->addWidget(motion_group);
  layout->addWidget(speed_plot_);
  layout->addWidget(error_plot_);

  connect(x_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(y_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(yaw_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(max_speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::updatePreview);
  connect(acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(max_speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::scheduleParameterUpdate);
  connect(acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::scheduleParameterUpdate);
  connect(parameter_timer_, &QTimer::timeout, this,
          &MotorMovePanel::sendParameters);
  connect(send_button_, &QPushButton::clicked, this, &MotorMovePanel::sendGoal);

  updatePreview();
}

void MotorMovePanel::onInitialize() {
  rviz_common::Panel::onInitialize();
  setupRos();
}

void MotorMovePanel::setupRos() {
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("no rviz node", false);
    return;
  }

  node_ = abstraction->get_raw_node();
  refreshParameterClient();
  action_client_ = rclcpp_action::create_client<MotorMoveAction>(
      node_, action_name_edit_->text().toStdString());
}

void MotorMovePanel::refreshParameterClient() {
  if (!node_) {
    return;
  }
  if (parameters_client_ &&
      current_parameter_node_ == parameter_node_edit_->text()) {
    return;
  }
  current_parameter_node_ = parameter_node_edit_->text();
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, current_parameter_node_.toStdString());
}

void MotorMovePanel::setStatus(const QString &text, bool ok) {
  parameter_status_->setText((ok ? QString::fromUtf8("✓ ") : QString("! ")) +
                             text);
  parameter_status_->setStyleSheet(ok ? "color: #22863a;" : "color: #b31d28;");
}

void MotorMovePanel::scheduleParameterUpdate() {
  parameter_timer_->start();
}

void MotorMovePanel::sendParameters() {
  if (!node_) {
    setupRos();
  }
  refreshParameterClient();
  if (!parameters_client_) {
    setStatus("no param client", false);
    return;
  }

  setStatus("setting", true);
  auto future = parameters_client_->set_parameters({
      rclcpp::Parameter("max_speed", max_speed_spin_->value()),
      rclcpp::Parameter("acceleration", acceleration_spin_->value()),
  });

  std::thread([this, future = std::move(future)]() mutable {
    bool ok = false;
    QString reason = "failed";
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      ok = true;
      reason = "set";
      for (const auto &result : future.get()) {
        ok = ok && result.successful;
        if (!result.successful) {
          reason = QString::fromStdString(result.reason);
        }
      }
    } else {
      reason = "timeout";
    }

    QMetaObject::invokeMethod(
        this, [this, reason, ok]() { setStatus(reason, ok); },
        Qt::QueuedConnection);
  }).detach();
}

void MotorMovePanel::sendGoal() {
  if (!node_) {
    setupRos();
  }
  action_client_ = rclcpp_action::create_client<MotorMoveAction>(
      node_, action_name_edit_->text().toStdString());

  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    setStatus("no action", false);
    return;
  }

  real_speed_.clear();
  real_error_.clear();
  speed_plot_->clearRealData();
  error_plot_->clearRealData();

  MotorMoveAction::Goal goal;
  goal.motor_goal.header.frame_id = frame_edit_->text().toStdString();
  goal.motor_goal.header.stamp = node_->now();
  goal.motor_goal.pose.position.x = x_spin_->value();
  goal.motor_goal.pose.position.y = y_spin_->value();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw_spin_->value() * M_PI / 180.0);
  goal.motor_goal.pose.orientation.x = quat.x();
  goal.motor_goal.pose.orientation.y = quat.y();
  goal.motor_goal.pose.orientation.z = quat.z();
  goal.motor_goal.pose.orientation.w = quat.w();

  rclcpp_action::Client<MotorMoveAction>::SendGoalOptions options;
  options.feedback_callback =
      [this](GoalHandleMotorMove::SharedPtr,
             const std::shared_ptr<const MotorMoveAction::Feedback> feedback) {
        const double time = feedback->elapsed_time;
        const double speed = feedback->linear_speed;
        const double distance = feedback->distance_to_target;
        QMetaObject::invokeMethod(
            this,
            [this, time, speed, distance]() {
              appendRealPoint(time, speed, distance);
            },
            Qt::QueuedConnection);
      };
  options.result_callback = [this](const GoalHandleMotorMove::WrappedResult &result) {
    QMetaObject::invokeMethod(
        this,
        [this, result]() {
          setStatus(result.code == rclcpp_action::ResultCode::SUCCEEDED
                        ? "done"
                        : "action failed",
                    result.code == rclcpp_action::ResultCode::SUCCEEDED);
        },
        Qt::QueuedConnection);
  };

  action_client_->async_send_goal(goal, options);
}

void MotorMovePanel::appendRealPoint(double time, double speed,
                                     double distance) {
  real_speed_.emplace_back(time, speed);
  real_error_.emplace_back(time, distance);
  speed_plot_->setRealData(real_speed_);
  error_plot_->setRealData(real_error_);
}

std::vector<QPointF> MotorMovePanel::makeIdealSpeed(double distance,
                                                    double max_speed,
                                                    double acceleration) const {
  std::vector<QPointF> points;
  constexpr double dt = 0.05;
  constexpr double tolerance = 0.02;
  double time = 0.0;
  double error = std::max(0.0, distance);
  double speed = 0.0;

  points.emplace_back(time, speed);
  for (int i = 0; i < 2000 && error > tolerance; ++i) {
    const double target_speed = brakingSpeed(error, max_speed, acceleration);
    speed = rampToward(speed, target_speed, acceleration * dt);
    error = std::max(0.0, error - speed * dt);
    time += dt;
    points.emplace_back(time, speed);
  }
  points.emplace_back(time + dt, 0.0);
  return points;
}

std::vector<QPointF> MotorMovePanel::makeIdealError(double distance,
                                                    double max_speed,
                                                    double acceleration) const {
  std::vector<QPointF> points;
  constexpr double dt = 0.05;
  constexpr double tolerance = 0.02;
  double time = 0.0;
  double error = std::max(0.0, distance);
  double speed = 0.0;

  points.emplace_back(time, error);
  for (int i = 0; i < 2000 && error > tolerance; ++i) {
    const double target_speed = brakingSpeed(error, max_speed, acceleration);
    speed = rampToward(speed, target_speed, acceleration * dt);
    error = std::max(0.0, error - speed * dt);
    time += dt;
    points.emplace_back(time, error);
  }
  points.emplace_back(time + dt, 0.0);
  return points;
}

void MotorMovePanel::updatePreview() {
  const double distance = std::hypot(x_spin_->value(), y_spin_->value());
  const double max_speed = max_speed_spin_->value();
  const double acceleration = acceleration_spin_->value();

  speed_plot_->setIdealData(makeIdealSpeed(distance, max_speed, acceleration));
  error_plot_->setIdealData(makeIdealError(distance, max_speed, acceleration));
  Q_EMIT configChanged();
}

void MotorMovePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString text;
  float value = 0.0F;
  if (config.mapGetString("Action", &text)) {
    action_name_edit_->setText(text);
  }
  if (config.mapGetString("ParameterNode", &text)) {
    parameter_node_edit_->setText(text);
  }
  if (config.mapGetString("Frame", &text)) {
    frame_edit_->setText(text);
  }
  if (config.mapGetFloat("X", &value)) {
    x_spin_->setValue(value);
  }
  if (config.mapGetFloat("Y", &value)) {
    y_spin_->setValue(value);
  }
  if (config.mapGetFloat("YawDegrees", &value)) {
    yaw_spin_->setValue(value);
  }
  if (config.mapGetFloat("MaxSpeed", &value)) {
    max_speed_spin_->setValue(value);
  }
  if (config.mapGetFloat("Acceleration", &value)) {
    acceleration_spin_->setValue(value);
  }
  updatePreview();
}

void MotorMovePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Action", action_name_edit_->text());
  config.mapSetValue("ParameterNode", parameter_node_edit_->text());
  config.mapSetValue("Frame", frame_edit_->text());
  config.mapSetValue("X", static_cast<float>(x_spin_->value()));
  config.mapSetValue("Y", static_cast<float>(y_spin_->value()));
  config.mapSetValue("YawDegrees", static_cast<float>(yaw_spin_->value()));
  config.mapSetValue("MaxSpeed", static_cast<float>(max_speed_spin_->value()));
  config.mapSetValue("Acceleration",
                     static_cast<float>(acceleration_spin_->value()));
}

} // namespace motor_move_rviz_plugin

PLUGINLIB_EXPORT_CLASS(motor_move_rviz_plugin::MotorMovePanel,
                       rviz_common::Panel)
