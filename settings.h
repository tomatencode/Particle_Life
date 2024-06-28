#ifndef CALCULATORFORM_H
#define CALCULATORFORM_H


#include "ui_settings.h"
#include "particle_visualization.h"
#include <QLabel>
#include <QPalette>
#include <QTimer>
#include <QTime>
#include <vector>


class Settings : public QWidget, private Ui::Settings_Widget
{
    Q_OBJECT

public:
    void update_lables();
    explicit Settings(QWidget *parent = nullptr, particle_visualization *visualization = nullptr);
    QPalette pal;
private:
    particle_visualization *visualization;
    QTimer *timer;
    std::vector<double> last_tps;
    QTime start_time;
private slots:
    void on_Reset_Button_clicked();
    void on_dt_slider_valueChanged(int value);
    void on_friction_slider_valueChanged(int value);
    void on_force_range_slider_valueChanged(int value);

    void on_stable_dist_slider_valueChanged(int value);
    void on_play_pause_Button_toggled(bool checked);
    void on_screenshot_Button_clicked();
};

#endif // SETTINGS_H
