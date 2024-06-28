#include "particle_life.h"
#include <vector>
#include <thread>
#include <algorithm>


particle_life::particle_life(std::vector<particle> particles, std::vector<std::vector<double>> force_tb,double force_range, double stable_dist, double dt, double friction, int max_x, int max_y)
{
    this->particles = particles;
    this->force_tb = force_tb;
    this->force_range = force_range;
    this->dt = dt;
    this->friction = friction;
    this->max_x = max_x;
    this->max_y = max_y;
    this->stable_dist = stable_dist;

}

bool compare_x_pos(particle p1, particle p2)
{
    return p1.position[0] > p2.position[0];
}


void particle_life::update()
{
    int num_particles = particles.size();
    sort(particles.begin(), particles.end(), compare_x_pos);


    // multi threading
    int num_threads = std::min(16,num_particles);
    std::vector<std::thread> threads;

    for (int i = 0; i < num_threads; i++) {
        double i_double = static_cast<double>(i);
        double num_particles_double = static_cast<double>(num_particles);
        double num_threads_double = static_cast<double>(num_threads);
        threads.push_back(std::thread(&particle_life::apply_all_forces, this, static_cast<int>(num_particles_double/num_threads_double*i_double), static_cast<int>(num_particles_double/num_threads_double*(i_double+1.0)) ));
    }

    for (auto &th : threads) {
        th.join();
    }
}

void particle_life::apply_all_forces(int start_num, int end_num)
{
    int num_particles = particles.size();
    for (int i=start_num;i< end_num;i++)
    {
        int j = (i+1)%num_particles;

        while (get_dist(particles[i], particles[j])[0] < force_range and i != j)
        {
            apply_force(particles[j], particles[i]);

            j = (j+1)%num_particles;
        }
        particles[i].update(dt, friction, max_x, max_y);
    }
}

double particle_life::get_force(double dist, int type_p1, int type_p2)
{
    if (dist < 1.0)
    {
        double force = force_tb[type_p1][type_p2];
        if (force >= 0.0)
        {
            if (dist < stable_dist)
            {
                return dist/stable_dist - 1.0;
            }
            else
            {
                return force * (1-abs(2*dist-1-stable_dist)/(1-stable_dist));
            }
        }
        else
        {
            if (dist < stable_dist)
            {
                return dist*(1.0+force)/stable_dist-1.0;
            }
            else
            {
                return dist*force/(stable_dist-1.0) + force-force*stable_dist/(stable_dist-1.0);
            }
        }
    }
    else {
        return 0.0;
    }
}

void particle_life::apply_force(particle& p1, particle& p2)
{
    Eigen::Vector2d diff = get_dist(p2, p1);


    if (diff[1] < force_range)
    {
        double dist = sqrt(diff.dot(diff));
        if (dist < force_range and dist != 0.0)
        {
            Eigen::Vector2d normal = diff / dist;
            double force_p1 = get_force(dist/force_range, p1.type, p2.type);
            p1.force += normal * force_p1;
            double force_p2 = get_force(dist/force_range, p2.type, p1.type);
            p2.force += -normal * force_p2;
        }
    }
}

Eigen::Vector2d particle_life::get_dist(const particle p1, const particle p2)
{
    double x_diff;
    if (abs(p2.position[0] - p1.position[0]) < abs(abs(p2.position[0] - p1.position[0]) - max_x)) {
        x_diff = -(p2.position[0] - p1.position[0]);
    }
    else {
        if (p1.position[0] > p2.position[0]) {
            x_diff = p1.position[0] - p2.position[0]-max_x;
        }
        else {
            x_diff = -(p2.position[0] - p1.position[0]-max_x);
        }
    }
    double y_diff;
    if (abs(p2.position[1] - p1.position[1]) < abs(abs(p2.position[1] - p1.position[1])-max_y)) {
        y_diff = -(p2.position[1] - p1.position[1]);
    }
    else {
        if (p1.position[1] > p2.position[1]) {
            y_diff = p1.position[1] - p2.position[1]-max_y;
        }
        else {
            y_diff = -(p2.position[1] - p1.position[1]-max_y);
        }
    }
    return {x_diff , y_diff};
}

