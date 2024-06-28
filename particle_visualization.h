#ifndef PARTICLE_VISUALIZATION_H
#define PARTICLE_VISUALIZATION_H

#include "particle_life.h"
#include <QWidget>
#include <QPainter>
#include <QTimer>

class particle_visualization : public QWidget
{
    Q_OBJECT

public:
    bool paused;
    int size_x;
    int size_y;
    double tps;
    std::vector<QColor> colormap;
    class particle_life *particle_life_ptr;

    void take_screenshot();

    particle_visualization(QWidget *parent = nullptr);
    ~particle_visualization();

private slots:
    void update_visualization();

public slots:
    void reset_particles(int num);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QTimer *timer;
    QTime t_last;
    std::vector<particle> get_random_particles(int num);

};

#endif // PARTICLE_VISUALIZATION_H
