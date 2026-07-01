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
  action_name_edit_ = new QLineEdit("/robotinobase1/motor_move_action");
  parameter_node_edit_ = new QLineEdit("/robotinobase1/motor_move");
  odom_topic_edit_ = new QLineEdit("/robotinobase1/odom");
  frame_edit_ = new QLineEdit("robotinobase1/base_link");

  x_spin_ = new QDoubleSpinBox();
  y_spin_ = new QDoubleSpinBox();
  yaw_spin_ = new QDoubleSpinBox();
  max_linear_speed_spin_ = new QDoubleSpinBox();
  linear_acceleration_spin_ = new QDoubleSpinBox();
  max_angular_speed_spin_ = new QDoubleSpinBox();
  angular_acceleration_spin_ = new QDoubleSpinBox();
  linear_kp_spin_ = new QDoubleSpinBox();
  angular_kp_spin_ = new QDoubleSpinBox();

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

  max_linear_speed_spin_->setRange(0.001, 10.0);
  max_linear_speed_spin_->setDecimals(3);
  max_linear_speed_spin_->setSingleStep(0.05);
  max_linear_speed_spin_->setValue(0.5);
  max_linear_speed_spin_->setSuffix(" m/s");

  linear_acceleration_spin_->setRange(0.001, 10.0);
  linear_acceleration_spin_->setDecimals(3);
  linear_acceleration_spin_->setSingleStep(0.05);
  linear_acceleration_spin_->setValue(0.5);
  linear_acceleration_spin_->setSuffix(" m/s^2");

  max_angular_speed_spin_->setRange(0.001, 10.0);
  max_angular_speed_spin_->setDecimals(3);
  max_angular_speed_spin_->setSingleStep(0.05);
  max_angular_speed_spin_->setValue(0.5);
  max_angular_speed_spin_->setSuffix(" rad/s");

  angular_acceleration_spin_->setRange(0.001, 10.0);
  angular_acceleration_spin_->setDecimals(3);
  angular_acceleration_spin_->setSingleStep(0.05);
  angular_acceleration_spin_->setValue(0.5);
  angular_acceleration_spin_->setSuffix(" rad/s^2");

  linear_kp_spin_->setRange(0.001, 20.0);
  linear_kp_spin_->setDecimals(3);
  linear_kp_spin_->setSingleStep(0.1);
  linear_kp_spin_->setValue(1.0);

  angular_kp_spin_->setRange(0.001, 20.0);
  angular_kp_spin_->setDecimals(3);
  angular_kp_spin_->setSingleStep(0.1);
  angular_kp_spin_->setValue(1.5);

  parameter_status_ = new QLabel("-");
  traveled_distance_label_ = new QLabel("0.000 m");
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
  connection_layout->addRow("Odom", odom_topic_edit_);

  auto *target_group = new QGroupBox("Target");
  auto *target_layout = new QFormLayout(target_group);
  target_layout->addRow("Frame", frame_edit_);
  target_layout->addRow("X", x_spin_);
  target_layout->addRow("Y", y_spin_);
  target_layout->addRow("Rot", yaw_spin_);

  auto *motion_group = new QGroupBox("Motion");
  auto *motion_layout = new QGridLayout(motion_group);
  motion_layout->addWidget(new QLabel("Max linear"), 0, 0);
  motion_layout->addWidget(max_linear_speed_spin_, 0, 1);
  motion_layout->addWidget(new QLabel("Linear accel"), 1, 0);
  motion_layout->addWidget(linear_acceleration_spin_, 1, 1);
  motion_layout->addWidget(new QLabel("Max angular"), 2, 0);
  motion_layout->addWidget(max_angular_speed_spin_, 2, 1);
  motion_layout->addWidget(new QLabel("Angular accel"), 3, 0);
  motion_layout->addWidget(angular_acceleration_spin_, 3, 1);
  motion_layout->addWidget(new QLabel("Linear kp"), 4, 0);
  motion_layout->addWidget(linear_kp_spin_, 4, 1);
  motion_layout->addWidget(new QLabel("Angular kp"), 5, 0);
  motion_layout->addWidget(angular_kp_spin_, 5, 1);
  motion_layout->addWidget(new QLabel("Odom traveled"), 6, 0);
  motion_layout->addWidget(traveled_distance_label_, 6, 1);
  motion_layout->addWidget(parameter_status_, 7, 0);
  motion_layout->addWidget(send_button_, 7, 1);

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
  connect(max_linear_speed_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(linear_acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(max_angular_speed_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(angular_acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::updatePreview);
  connect(linear_kp_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::updatePreview);
  connect(angular_kp_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::updatePreview);
  connect(max_linear_speed_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::scheduleParameterUpdate);
  connect(linear_acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::scheduleParameterUpdate);
  connect(max_angular_speed_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::scheduleParameterUpdate);
  connect(angular_acceleration_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &MotorMovePanel::scheduleParameterUpdate);
  connect(linear_kp_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::scheduleParameterUpdate);
  connect(angular_kp_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MotorMovePanel::scheduleParameterUpdate);
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
  refreshOdomSubscription();
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

void MotorMovePanel::refreshOdomSubscription() {
  if (!node_) {
    return;
  }
  if (odom_sub_ && current_odom_topic_ == odom_topic_edit_->text()) {
    return;
  }
  current_odom_topic_ = odom_topic_edit_->text();
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      current_odom_topic_.toStdString(), rclcpp::SensorDataQoS(),
      std::bind(&MotorMovePanel::odomCallback, this, std::placeholders::_1));
}

void MotorMovePanel::setStatus(const QString &text, bool ok) {
  parameter_status_->setText((ok ? QString::fromUtf8("✓ ") : QString("! ")) +
                             text);
  parameter_status_->setStyleSheet(ok ? "color: #22863a;" : "color: #b31d28;");
}

void MotorMovePanel::resetTraveledDistance() {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  tracking_distance_ = true;
  have_last_odom_position_ = false;
  traveled_distance_ = 0.0;
  traveled_distance_label_->setText("0.000 m");
}

void MotorMovePanel::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double traveled = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!tracking_distance_) {
      return;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    if (have_last_odom_position_) {
      traveled_distance_ += std::hypot(x - last_odom_x_, y - last_odom_y_);
    }
    last_odom_x_ = x;
    last_odom_y_ = y;
    have_last_odom_position_ = true;
    traveled = traveled_distance_;
  }

  QMetaObject::invokeMethod(
      this,
      [this, traveled]() {
        traveled_distance_label_->setText(
            QString("%1 m").arg(traveled, 0, 'f', 3));
      },
      Qt::QueuedConnection);
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
      rclcpp::Parameter("max_linear_speed", max_linear_speed_spin_->value()),
      rclcpp::Parameter("linear_acceleration",
                        linear_acceleration_spin_->value()),
      rclcpp::Parameter("max_angular_speed", max_angular_speed_spin_->value()),
      rclcpp::Parameter("angular_acceleration",
                        angular_acceleration_spin_->value()),
      rclcpp::Parameter("linear_kp", linear_kp_spin_->value()),
      rclcpp::Parameter("angular_kp", angular_kp_spin_->value()),
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
  refreshOdomSubscription();
  action_client_ = rclcpp_action::create_client<MotorMoveAction>(
      node_, action_name_edit_->text().toStdString());
  resetTraveledDistance();

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
  options.goal_response_callback = [this](GoalHandleMotorMove::SharedPtr goal) {
    QMetaObject::invokeMethod(
        this,
        [this, accepted = static_cast<bool>(goal)]() {
          setStatus(accepted ? "sent" : "rejected", accepted);
        },
        Qt::QueuedConnection);
  };
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

double MotorMovePanel::rotationTime(double yaw_error, double max_speed,
                                    double acceleration, double kp) const {
  constexpr double dt = 0.05;
  constexpr double tolerance = 2.0 * M_PI / 180.0;
  double time = 0.0;
  double error = std::max(0.0, std::fabs(yaw_error));
  double speed = 0.0;

  for (int i = 0; i < 2000 && error > tolerance; ++i) {
    const double target_speed =
        std::min(brakingSpeed(error, max_speed, acceleration), kp * error);
    speed = rampToward(speed, target_speed, acceleration * dt);
    speed = std::min(speed, error / dt);
    error = std::max(0.0, error - speed * dt);
    time += dt;
  }

  return time;
}

std::vector<QPointF>
MotorMovePanel::makeIdealSpeed(double distance, double yaw_error,
                               double max_linear_speed,
                               double linear_acceleration,
                               double max_angular_speed,
                               double angular_acceleration, double linear_kp,
                               double angular_kp) const {
  std::vector<QPointF> points;
  constexpr double dt = 0.05;
  constexpr double tolerance = 0.02;
  double time =
      rotationTime(yaw_error, max_angular_speed, angular_acceleration,
                   angular_kp);
  double error = std::max(0.0, distance);
  double speed = 0.0;

  points.emplace_back(0.0, 0.0);
  if (time > 0.0) {
    points.emplace_back(time, 0.0);
  }
  for (int i = 0; i < 2000 && error > tolerance; ++i) {
    const double target_speed =
        std::min(brakingSpeed(error, max_linear_speed, linear_acceleration),
                 linear_kp * error);
    speed = rampToward(speed, target_speed, linear_acceleration * dt);
    speed = std::min(speed, error / dt);
    error = std::max(0.0, error - speed * dt);
    time += dt;
    points.emplace_back(time, speed);
  }
  points.emplace_back(time + dt, 0.0);
  return points;
}

std::vector<QPointF>
MotorMovePanel::makeIdealError(double distance, double yaw_error,
                               double max_linear_speed,
                               double linear_acceleration,
                               double max_angular_speed,
                               double angular_acceleration, double linear_kp,
                               double angular_kp) const {
  std::vector<QPointF> points;
  constexpr double dt = 0.05;
  constexpr double tolerance = 0.02;
  double time =
      rotationTime(yaw_error, max_angular_speed, angular_acceleration,
                   angular_kp);
  double error = std::max(0.0, distance);
  double speed = 0.0;

  points.emplace_back(0.0, error);
  if (time > 0.0) {
    points.emplace_back(time, error);
  }
  for (int i = 0; i < 2000 && error > tolerance; ++i) {
    const double target_speed =
        std::min(brakingSpeed(error, max_linear_speed, linear_acceleration),
                 linear_kp * error);
    speed = rampToward(speed, target_speed, linear_acceleration * dt);
    speed = std::min(speed, error / dt);
    error = std::max(0.0, error - speed * dt);
    time += dt;
    points.emplace_back(time, error);
  }
  points.emplace_back(time + dt, 0.0);
  return points;
}

void MotorMovePanel::updatePreview() {
  const double distance = std::hypot(x_spin_->value(), y_spin_->value());
  const double yaw_error = yaw_spin_->value() * M_PI / 180.0;
  const double max_linear_speed = max_linear_speed_spin_->value();
  const double linear_acceleration = linear_acceleration_spin_->value();
  const double max_angular_speed = max_angular_speed_spin_->value();
  const double angular_acceleration = angular_acceleration_spin_->value();
  const double linear_kp = linear_kp_spin_->value();
  const double angular_kp = angular_kp_spin_->value();

  speed_plot_->setIdealData(makeIdealSpeed(
      distance, yaw_error, max_linear_speed, linear_acceleration,
      max_angular_speed, angular_acceleration, linear_kp, angular_kp));
  error_plot_->setIdealData(makeIdealError(
      distance, yaw_error, max_linear_speed, linear_acceleration,
      max_angular_speed, angular_acceleration, linear_kp, angular_kp));
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
  if (config.mapGetString("OdomTopic", &text)) {
    odom_topic_edit_->setText(text);
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
  if (config.mapGetFloat("MaxLinearSpeed", &value)) {
    max_linear_speed_spin_->setValue(value);
  }
  if (config.mapGetFloat("LinearAcceleration", &value)) {
    linear_acceleration_spin_->setValue(value);
  }
  if (config.mapGetFloat("MaxAngularSpeed", &value)) {
    max_angular_speed_spin_->setValue(value);
  }
  if (config.mapGetFloat("AngularAcceleration", &value)) {
    angular_acceleration_spin_->setValue(value);
  }
  if (config.mapGetFloat("LinearKp", &value)) {
    linear_kp_spin_->setValue(value);
  }
  if (config.mapGetFloat("AngularKp", &value)) {
    angular_kp_spin_->setValue(value);
  }
  updatePreview();
}

void MotorMovePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Action", action_name_edit_->text());
  config.mapSetValue("ParameterNode", parameter_node_edit_->text());
  config.mapSetValue("OdomTopic", odom_topic_edit_->text());
  config.mapSetValue("Frame", frame_edit_->text());
  config.mapSetValue("X", static_cast<float>(x_spin_->value()));
  config.mapSetValue("Y", static_cast<float>(y_spin_->value()));
  config.mapSetValue("YawDegrees", static_cast<float>(yaw_spin_->value()));
  config.mapSetValue("MaxLinearSpeed",
                     static_cast<float>(max_linear_speed_spin_->value()));
  config.mapSetValue("LinearAcceleration",
                     static_cast<float>(linear_acceleration_spin_->value()));
  config.mapSetValue("MaxAngularSpeed",
                     static_cast<float>(max_angular_speed_spin_->value()));
  config.mapSetValue("AngularAcceleration",
                     static_cast<float>(angular_acceleration_spin_->value()));
  config.mapSetValue("LinearKp", static_cast<float>(linear_kp_spin_->value()));
  config.mapSetValue("AngularKp",
                     static_cast<float>(angular_kp_spin_->value()));
}

} // namespace motor_move_rviz_plugin

PLUGINLIB_EXPORT_CLASS(motor_move_rviz_plugin::MotorMovePanel,
                       rviz_common::Panel)
