#include "motor_move_rviz_plugin/plot_widget.hpp"

#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>

namespace motor_move_rviz_plugin {

PlotWidget::PlotWidget(QWidget *parent) : QWidget(parent) {
  setMinimumHeight(150);
}

void PlotWidget::setTitle(const QString &title, const QString &y_label) {
  title_ = title;
  y_label_ = y_label;
  update();
}

void PlotWidget::setIdealData(const std::vector<QPointF> &data) {
  ideal_data_ = data;
  update();
}

void PlotWidget::setRealData(const std::vector<QPointF> &data) {
  real_data_ = data;
  update();
}

void PlotWidget::clearRealData() {
  real_data_.clear();
  update();
}

QRect PlotWidget::plotRect() const {
  return rect().adjusted(44, 24, -12, -28);
}

double PlotWidget::maxTime() const {
  double value = 1.0;
  for (const auto &point : ideal_data_) {
    value = std::max(value, point.x());
  }
  for (const auto &point : real_data_) {
    value = std::max(value, point.x());
  }
  return value;
}

double PlotWidget::maxValue() const {
  double value = 0.1;
  for (const auto &point : ideal_data_) {
    value = std::max(value, point.y());
  }
  for (const auto &point : real_data_) {
    value = std::max(value, point.y());
  }
  return value * 1.1;
}

QPointF PlotWidget::mapPoint(const QPointF &point, const QRect &rect,
                             double max_time, double max_value) const {
  const double x = rect.left() + point.x() / max_time * rect.width();
  const double y = rect.bottom() - point.y() / max_value * rect.height();
  return QPointF(x, y);
}

void PlotWidget::drawSeries(QPainter &painter, const QRect &rect,
                            const std::vector<QPointF> &data,
                            const QColor &color, double max_time,
                            double max_value) const {
  if (data.size() < 2) {
    return;
  }

  QPainterPath path;
  path.moveTo(mapPoint(data.front(), rect, max_time, max_value));
  for (size_t i = 1; i < data.size(); ++i) {
    path.lineTo(mapPoint(data[i], rect, max_time, max_value));
  }

  QPen pen(color, 2);
  painter.setPen(pen);
  painter.drawPath(path);
}

void PlotWidget::paintEvent(QPaintEvent *event) {
  (void)event;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), palette().window());

  const QRect plot_rect = plotRect();
  const double max_time = maxTime();
  const double max_value = maxValue();

  painter.setPen(QPen(QColor(80, 80, 80), 1));
  painter.drawRect(plot_rect);

  painter.setPen(QColor(180, 180, 180));
  for (int i = 1; i < 4; ++i) {
    const int x = plot_rect.left() + plot_rect.width() * i / 4;
    const int y = plot_rect.top() + plot_rect.height() * i / 4;
    painter.drawLine(x, plot_rect.top(), x, plot_rect.bottom());
    painter.drawLine(plot_rect.left(), y, plot_rect.right(), y);
  }

  painter.setPen(palette().text().color());
  painter.drawText(8, 16, title_);
  painter.drawText(6, plot_rect.center().y(), y_label_);
  painter.drawText(plot_rect.center().x() - 18, height() - 8, "time [s]");
  painter.drawText(plot_rect.left(), height() - 8, "0");
  painter.drawText(plot_rect.right() - 46, height() - 8,
                   QString::number(max_time, 'f', 1));
  painter.drawText(6, plot_rect.top() + 10, QString::number(max_value, 'f', 2));

  painter.setPen(QColor(35, 120, 210));
  painter.drawText(plot_rect.right() - 130, 16, "ideal");
  painter.setPen(QColor(220, 95, 30));
  painter.drawText(plot_rect.right() - 76, 16, "real");

  drawSeries(painter, plot_rect, ideal_data_, QColor(35, 120, 210), max_time,
             max_value);
  drawSeries(painter, plot_rect, real_data_, QColor(220, 95, 30), max_time,
             max_value);
}

} // namespace motor_move_rviz_plugin
