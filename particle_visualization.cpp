#include "particle_visualization.h"
#include <vector>
#include <random>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTime>
#include <QImage>
#include <QPixmap>

particle_visualization::particle_visualization(QWidget *parent) : QWidget(parent) {

    t_last  = QTime::currentTime();
    paused = false;

    // particle life default parameters
    std::vector<std::vector<double>>  force_tb = {{{0.0,0.0,0.0,0.0}},{{0.0,0.0,0.0,0.0}},{{0.0,0.0,0.0,0.0}},{{0.0,0.0,0.0,0.0}}};
    force_tb[0][0] = 1.0;
    force_tb[0][1] = -1.0;
    force_tb[0][2] = 0.3;
    force_tb[0][3] = 1.0;

    force_tb[1][0] = -1.0;
    force_tb[1][1] = 0.4;
    force_tb[1][2] = -1.0;
    force_tb[1][3] = 1.0;

    force_tb[2][0] = 1.0;
    force_tb[2][1] = -1.0;
    force_tb[2][2] = -0.5;
    force_tb[2][3] = -0.2;

    force_tb[3][0] = 0.8;
    force_tb[3][1] = 1.0;
    force_tb[3][2] = -0.3;
    force_tb[3][3] = 0.2;


    size_x = 1845;
    size_y = 1020;

    std::vector<particle> particles = {};

    double dt = 0.08;
    double friction = 0.982;
    double force_range = 100.0;
    double stable_dist = 0.3;

    this->particle_life_ptr = new class particle_life(particles, force_tb, force_range, stable_dist, dt, friction, size_x, size_y);
    reset_particles(800); // spawns 800 random particles

    // visualization parameters
    particle_visualization::colormap = {
        QColor(255,0,0),
        QColor(0,255,0),
        QColor(0,0,255),
        QColor(255,0,255)
    };

    // setting up a timer to call update each tick
    particle_visualization::tps = 0;
    particle_visualization::timer = new QTimer(this);
    particle_visualization::connect(timer, &QTimer::timeout, this, &particle_visualization::update_visualization);
    particle_visualization::timer->start(0);
}

particle_visualization::~particle_visualization()
{
    delete particle_life_ptr; // Clean up dynamically allocated memory
    delete timer; // Clean up timer
}

void particle_visualization::update_visualization()
{

    if (not paused)
    {
        particle_life_ptr->update(); // update particle life if not paused
    }

    update(); // Request repaint after update

    // measure tps (ticks per second)
    tps = 1000/t_last.msecsTo(QTime::currentTime());
    t_last = QTime::currentTime();
}

void particle_visualization::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing);

    // draw background
    painter.setBrush(QBrush(QColor(10,10,10)));
    painter.drawRect(this->rect());
    painter.setBrush(QBrush(QColor(25,25,25)));
    painter.drawRect(QRect(0,0,size_x,size_y));

    // draw particles
    for (particle &particle : particle_life_ptr->particles)
    {
        painter.setBrush(QBrush(colormap[particle.type]));
        double pos_x = particle.position[0];
        double pos_y = particle.position[1];
        painter.drawEllipse(QPointF(pos_x,pos_y), 2, 2);
    }
}

void particle_visualization::take_screenshot()
{
    QPixmap pixmap(size_x, size_y);
    pixmap.fill(QColor(10,10,10)); // Fill the pixmap with white color

    QPainter painter(&pixmap);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing);

    // draw particles
    for (particle &particle : particle_life_ptr->particles)
    {
        painter.setBrush(QBrush(colormap[particle.type]));
        double pos_x = particle.position[0];
        double pos_y = particle.position[1];
        painter.drawEllipse(QPointF(pos_x,pos_y), 2, 2);
    }

    // Save the pixmap to a file
    QString filePath = "/home/simon/Pictures/Particle_life_screenshot.png";
    pixmap.save(filePath);
}

std::vector<particle> particle_visualization::get_random_particles(int num)
{
    // innitilise the random number generators
    std::uniform_real_distribution<double> rand_x(0,size_x);
    std::uniform_real_distribution<double> rand_y(0,size_y);
    std::uniform_int_distribution<> rand_type(0, 3);
    std::random_device rd;

    // spawn particles
    std::vector<particle> particles;
    for (int i = 0; i < num; i++) {
        particles.push_back(particle({rand_x(rd),rand_y(rd)},rand_type(rd)));
    }

    return particles;
}


// slots
void particle_visualization::reset_particles(int num)
{
    particle_life_ptr->particles = get_random_particles(num);
}
