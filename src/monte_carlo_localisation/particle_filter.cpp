#include "particle_filter.hpp"
#include <random>

ParticleFilter::ParticleFilter(const std::shared_ptr<MeasurementModel> &measurement_model,
                               const std::shared_ptr<MotionModel> &motion_model,
                               const ResamplingMethod resampling_method=ResamplingMethod::DEFAULT) :
                               measurement_model_(measurement_model),
                               motion_model_(motion_model)
{
    // get num_of_particles, init_pose, 
    initialiseFilter(initial_pose, init_number_of_particles);
}

/**
 * Initialises the particle filter by adding particles that are normally distributed 
 * around the given pose, to avoid having all the particles at the same point.
 * 
 * @param init_pose The approximate pose to which the filter should be initialised
 * @param number_of_particles The number of particles to initialise the filter with.
 */
void ParticleFilter::initialiseFilter(const geometry_msgs::TransformStamped &init_pose,
                                      const int &number_of_particles)
{
    particles_t_1_.clear();

    // TODO:: implement fluctuations in the angle as well
    std::default_random_engine generator;
    std::normal_distribution<double> rand_distribution(0.0, 0.5);    // mean of 0.0 and std of 0.5

    for(int i=0; i<number_of_particles; i++)
    {
        double delta_x = rand_distribution(generator);
        double delta_y = rand_distribution(generator);

        geometry_msgs::TransformStamped pose;
        pose.transform.translation.x = init_pose.transform.translation.x + delta_x;
        pose.transform.translation.y = init_pose.transform.translation.y + delta_y;
        particles_t_1_.push_back(Particle(pose, 1.0));  // arbitrarily assing a weight of 1.0
    }
}

std::vector<Particle> ParticleFilter::getParticles() const
{
    return particles_t_;
}

void ParticleFilter::update(const geometry_msgs::TransformStamped &prev_odom,
                            const geometry_msgs::TransformStamped &curr_odom,
                            const sensor_msgs::LaserScan::ConstPtr &scan)
{
    particles_t_.clear();   // clear current set of particles
    std::vector<Particle> particles_t_bar;

    for(int i=0; i<particles_t_1_.size(); i++)
        particles_t_bar.push_back(sample(particles_t_1_[i], prev_odom, curr_odom, scan));

    particles_t_ = resample(particles_t_bar);
    particles_t_1_ = particles_t_;
}

Particle ParticleFilter::sample(const Particle &x_t_1,
                                const geometry_msgs::TransformStamped &prev_odom,
                                const geometry_msgs::TransformStamped &curr_odom,
                                const sensor_msgs::LaserScan::ConstPtr &scan)
{
    geometry_msgs::TransformStamped most_likely_pose = motion_model_->getMostLikelyPose(x_t_1.pose_, prev_odom, curr_odom);
    double weight = measurement_model_->getProbability(scan, most_likely_pose);
    return Particle(most_likely_pose, weight);
}

std::vector<Particle> ParticleFilter::resample(const std::vector<Particle> &x_t_bar)
{
    switch (resampling_method_)
    {
        case ResamplingMethod::DEFAULT:
            return defaultResample(x_t_bar);
        
        default:
            throw std::runtime_error("This resampling method is not implemented.");
    }
}

std::vector<Particle> defaultResample(const std::vector<Particle> &x_t_bar)
{
    // TODO: Find a more efficient way to do this
    std::vector<Particle> x_t;
    std::vector<double> weights;

    // extract all the weights into a vector
    for(auto particle : x_t_bar)
        weights.push_back(particle.weight_);

    std::default_random_engine generator;
    std::discrete_distribution<int> weights_dist(weights.begin(), weights.end());

    for(int i=0; i<x_t_bar.size(); i++)
    {
        int drawn_particle_index = weights_dist(generator); 
        x_t.push_back(x_t_bar[drawn_particle_index]);
    }

    return x_t;
}