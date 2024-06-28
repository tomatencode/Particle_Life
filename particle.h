#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>

class particle
{
public:
    Eigen::Vector2d position;
    Eigen::Vector2d velocety;
    Eigen::Vector2d force;
    int type;
    particle(Eigen::Vector2d position, int type);
    void update(double dt, double friction, int max_x, int max_y);
};
#endif // PARTICLE_H
