#include "particle.h"

particle::particle(Eigen::Vector2d position, int type) {
    this->position = position;
    this->velocety = {0.0,0.0};
    this->force = {0.0, 0.0};
    this->type = type;
}

void particle::update(double dt, double friction, int max_x, int max_y)
{
    position += velocety * dt;
    velocety = velocety*friction + force * dt;
    force = {0.0, 0.0};

    if (position[0]> max_x) {
        position[0] = position[0] - max_x;
    }
    else if (position[0] < 0) {
        position[0] = position[0] + max_x;
    }
    if (position[1]> max_y) {
        position[1] = position[1] - max_y;
    }
    else if (position[1] < 0) {
        position[1] = position[1] + max_y;
    }
}

