#pragma once

#include <QPointF>
#include <QWidget>

#include <vector>

namespace motor_move_rviz_plugin {

class PlotWidget : public QWidget {
  Q_OBJECT

public:
  explicit PlotWidget(QWidget *parent = nullptr);

  void setTitle(const QString &title, const QString &y_label);
  void setIdealData(const std::vector<QPointF> &data);
  void setRealData(const std::vector<QPointF> &data);
  void clearRealData();

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  QRect plotRect() const;
  QPointF mapPoint(const QPointF &point, const QRect &rect, double max_time,
                   double max_value) const;
  double maxTime() const;
  double maxValue() const;
  void drawSeries(QPainter &painter, const QRect &rect,
                  const std::vector<QPointF> &data, const QColor &color,
                  double max_time, double max_value) const;

  QString title_;
  QString y_label_;
  std::vector<QPointF> ideal_data_;
  std::vector<QPointF> real_data_;
};

} // namespace motor_move_rviz_plugin
