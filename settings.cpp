#include "settings.h"
#include <QWidget>
#include <QPalette>
#include <string>
#include <QIcon>

void Settings::update_lables() {
    // sets the force table of the simulation to the force table of the widet
    this->visualization->particle_life_ptr->force_tb = this->force_tb->values;

    // gets the avrage of the last 100 tps values and diplays them
    last_tps.insert(last_tps.begin(), visualization->tps);
    if (static_cast<int>(last_tps.size()) > 100) {
        last_tps.pop_back();
    }
    this->tps_label->setText(QString::fromStdString("tps: " + std::to_string(static_cast<int>(std::reduce(last_tps.begin(), last_tps.end()) / static_cast<int>(last_tps.size())))));

    // displays the Delta Time
    this->dt_label->setText(QString::fromStdString("dt: " + std::to_string(visualization->particle_life_ptr->dt).substr(0, 5)));

    // displays the friction
    this->friction_label->setText(QString::fromStdString("Friction: " + std::to_string(visualization->particle_life_ptr->friction).substr(0, 5)));

    // displays num particles
    int num = visualization->particle_life_ptr->particles.size();
    this->num_particles_label->setText(QString::fromStdString("Particles: " + std::to_string(num)));

    // displays num Types
    this->num_types_label->setText(QString::fromStdString("Types: " + std::to_string(static_cast<int>(visualization->colormap.size()))));

    // displays Force Range
    this->force_range_label->setText(QString::fromStdString("Force Range: " + std::to_string(static_cast<int>(visualization->particle_life_ptr->force_range))));

    // displays Stable Dist
    this->sable_dist_label->setText(QString::fromStdString("Stable Distance: " + std::to_string(visualization->particle_life_ptr->stable_dist).substr(0, 4)));

    // displays Time Running
    std::string time_running = "0sec";
    if (start_time.secsTo(QTime::currentTime()) < 60) {
        time_running = std::to_string(start_time.secsTo(QTime::currentTime())) + " sec";
    }
    else if (QTime::currentTime().minute() - start_time.minute() < 60) {
        time_running = std::to_string(QTime::currentTime().minute() - start_time.minute()) + " min";
    }
    else {
        time_running = std::to_string(QTime::currentTime().hour() - start_time.hour()) + " h";
    }
    this->time_running->setText(QString::fromStdString("Time Running: " + time_running));
}

void Settings::on_Reset_Button_clicked() {
    start_time = QTime::currentTime();
    visualization->reset_particles(visualization->particle_life_ptr->particles.size());
}

void Settings::on_play_pause_Button_toggled(bool checked)
{
    if (checked){
        this->play_pause_Button->setIcon(QIcon(":/icons/play_icon.png"));
        visualization->paused = true;
    }
    else {
        this->play_pause_Button->setIcon(QIcon(":/icons/pause_icon.png"));
        visualization->paused = false;
    }
}


void Settings::on_screenshot_Button_clicked()
{
    visualization->take_screenshot();
}

void Settings::on_dt_slider_valueChanged(int value)
{
    visualization->particle_life_ptr->dt = 0.02 + static_cast<double>(value)/1000.0 * 0.18;
}

void Settings::on_friction_slider_valueChanged(int value)
{
    visualization->particle_life_ptr->friction = 0.95 + static_cast<double>(value)/1000.0 * 0.05;
}


void Settings::on_force_range_slider_valueChanged(int value)
{
    visualization->particle_life_ptr->force_range = 10.0 + static_cast<double>(value)/1000.0 * 140.0;
}

void Settings::on_stable_dist_slider_valueChanged(int value)
{
    visualization->particle_life_ptr->stable_dist = 0.1 + static_cast<double>(value)/1000.0 * 0.7;
}

Settings::Settings(QWidget *parent, particle_visualization *visualization)
    : QWidget(parent)
{
    this->visualization = visualization;
    setupUi(this);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Settings::update_lables);
    timer->start(10);

    start_time = QTime::currentTime();
    last_tps = {};

    // set default dlider values
    this->dt_slider->setSliderPosition(((visualization->particle_life_ptr->dt-0.02)/0.18)*1000.0+1.0);
    this->friction_slider->setSliderPosition(((visualization->particle_life_ptr->friction-0.95)/0.05)*1000.0+1.0);
    this->force_range_slider->setSliderPosition(((visualization->particle_life_ptr->force_range-10.0)/140.0)*1000.0+1.0);
    this->stable_dist_slider->setSliderPosition(((visualization->particle_life_ptr->stable_dist-0.1)/0.7)*1000.0+1.0);

    // set default force tb
    this->force_tb->setParticles(visualization->colormap);
    this->force_tb->values = visualization->particle_life_ptr->force_tb;
}
