#include <vector>
#include <array>
#include <map>
#include <Eigen/Dense>

namespace ParticleLife
{

    class particle
    {
    public:
        Eigen::Vector2d position;
        Eigen::Vector2d velocety;
        Eigen::Vector2d force;
        int type;
        particle(Eigen::Vector2d position, Eigen::Vector2d velocety, int type)
        {
            this->position = position;
            this->velocety = velocety;
            this->force = {0.0, 0.0};
            this->type = type;
        }
        void update(double dt)
        {
            position += velocety * dt;
            velocety += force * dt;
            force = {0.0, 0.0};
        }
    };

    template <int num_particle_types>
    class particle_life
    {
    public:
        std::vector<ParticleLife::particle> particles;
        Eigen::Matrix<double, num_particle_types, num_particle_types> force_tb;
        double force_range;
        double dt;
        particle_life(std::vector<ParticleLife::particle> particles, Eigen::Matrix<double, num_particle_types, num_particle_types> force_tb,double force_range, double dt)
        {
            this->particles = particles;
            this->force_tb = force_tb;
            this->force_range = force_range;
            this->dt = dt;
        }

        void update()
        {
            int num_particles = particles.size();
            for (int i=0;i<num_particles;i++){
                for (int j=i+1;j<num_particles;j++){
                    apply_force(particles[i], particles[j]);
                }
                particles[i].update(dt);
            }
        }

    private:
        double get_force(double dist, int type_p1, int type_p2)
        {
            if (dist < 1.0)
            {
                double force = force_tb(type_p1, type_p2);
                if (force >= 0.0)
                {
                    if (dist < 0.2)
                    {
                        return 5.0 * dist - 1.0;
                    }
                    else if (dist < 0.6)
                    {
                        return force * dist - force * 0.2;
                    }
                    else
                    {
                        return force * dist + dist;
                    }
                }
                else
                {
                    if (dist < 0.2)
                    {
                        return (force - 1.0) * 5.0 * dist - 1.0;
                    }
                    else
                    {
                        return force * (1 - (dist * 1.25 - 0.25));
                    }
                }
            }
            else {
                return 0.0;
            }
        }

        void apply_force(ParticleLife::particle& p1, ParticleLife::particle& p2)
        {
            Eigen::Vector2d diff = p2.position - p1.position;
            if (diff[0] < force_range and diff[1] < force_range)
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
    };
}
