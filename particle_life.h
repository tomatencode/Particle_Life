#ifndef PARTICLE_LIFE_H
#define PARTICLE_LIFE_H

#include "particle.h"
#include <QTime>
#include <vector>
#include <Eigen/Dense>


class particle_life
{
public:
    std::vector<particle> particles;
    std::vector<std::vector<double>> force_tb;
    double force_range;
    double stable_dist;
    double dt;
    double friction;
    int max_x;
    int max_y;
    particle_life(std::vector<particle> particles, std::vector<std::vector<double>> force_tb,double force_range, double stable_dist, double dt, double friction, int max_x, int max_y);

    void update();

private:
    void apply_all_forces(int start_num, int end_num);
    double get_force(double dist, int type_p1, int type_p2);
    Eigen::Vector2d get_dist(const particle p1, const particle p2);
    void apply_force(particle& p1, particle& p2);
    QTime t_last;
};


#endif // PARTICLE_LIFE_H
