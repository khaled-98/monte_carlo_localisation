#ifndef B8F66BB3_CA4F_43CA_B05B_5337918AC21F
#define B8F66BB3_CA4F_43CA_B05B_5337918AC21F

#include "interfaces/measurement_model.hpp"
#include "interfaces/motion_model.hpp"
#include "particle.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"

/**
 * A particle filter implementatation. This is where all the Bayesian
 * filtering happens.
 */
class ParticleFilter
{
public:
    enum class ResamplingMethod{DEFAULT, AUGMENTED};

    ParticleFilter(const std::shared_ptr<MeasurementModel> &measurement_model,
                   const std::shared_ptr<MotionModel> &motion_model,
                   const ResamplingMethod &resampling_method=ResamplingMethod::AUGMENTED);
    void initialiseFilter(const geometry_msgs::TransformStamped &init_pose,
                          const int &number_of_particles);
    std::vector<Particle> getParticles() const;
    geometry_msgs::PoseArray getPoses() const;
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
    std::vector<Particle> augmentedResample(const std::vector<Particle> &x_t_bar);
    
    ros::NodeHandle nh_, private_nh_;
    std::shared_ptr<MeasurementModel> measurement_model_;
    std::shared_ptr<MotionModel> motion_model_;
    std::vector<Particle> particles_t_1_;
    std::vector<Particle> particles_t_;
    geometry_msgs::TransformStamped most_likely_pose_;
    ResamplingMethod resampling_method_;
    double w_slow_, w_fast_;
    double alpha_slow_, alpha_fast_;
};

#endif /* B8F66BB3_CA4F_43CA_B05B_5337918AC21F */
