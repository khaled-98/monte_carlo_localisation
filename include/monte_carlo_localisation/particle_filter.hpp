#ifndef B8F66BB3_CA4F_43CA_B05B_5337918AC21F
#define B8F66BB3_CA4F_43CA_B05B_5337918AC21F

#include "interfaces/measurement_model.hpp"
#include "interfaces/motion_model.hpp"
#include "particle.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/LaserScan.h"

/**
 * A particle filter implementatation. This is where all the Bayesian
 * filtering happens.
 */
class ParticleFilter
{
public:
    enum class ResamplingMethod{DEFAULT};

    ParticleFilter(const std::shared_ptr<MeasurementModel> &measurement_model,
                   const std::shared_ptr<MotionModel> &motion_model,
                   const ResamplingMethod resampling_method=ResamplingMethod::DEFAULT);
    void initialiseFilter(const geometry_msgs::TransformStamped &init_pose,
                          const int &number_of_particles);
    std::vector<Particle> getParticles() const;
    void update(const geometry_msgs::TransformStamped &prev_odom,
                const geometry_msgs::TransformStamped &curr_odom,
                const sensor_msgs::LaserScan::ConstPtr &scan);

private:
    Particle sample(const Particle &x_t_1,
                    const geometry_msgs::TransformStamped &prev_odom,
                    const geometry_msgs::TransformStamped &curr_odom,
                    const sensor_msgs::LaserScan::ConstPtr &scan);
    std::vector<Particle> resample(const std::vector<Particle> &x_t_bar);
    std::vector<Particle> defaultResample(const std::vector<Particle> &x_t_bar);
    
    std::shared_ptr<MeasurementModel> measurement_model_;
    std::shared_ptr<MotionModel> motion_model_;
    std::vector<Particle> particles_t_1_;
    std::vector<Particle> particles_t_;
    ResamplingMethod resampling_method_;
};

#endif /* B8F66BB3_CA4F_43CA_B05B_5337918AC21F */
