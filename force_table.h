#ifndef force_table_H
#define force_table_H

#include <QWidget>
#include <QWheelEvent>
#include <QPainter>
#include <vector>

class force_table : public QWidget {
    Q_OBJECT

public:
    explicit force_table(QWidget *parent = nullptr);
    std::vector<std::vector<double>> values;
    void setParticles(const std::vector<QColor>& colors);

protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    int size;
    std::vector<QRect> particle_rects;
    std::vector<std::vector<QRect>> value_rects;
    void update_rects();
    std::vector<QColor> particle_colors;

    QColor interpolateColor(double value);
};

#endif // force_table_H
