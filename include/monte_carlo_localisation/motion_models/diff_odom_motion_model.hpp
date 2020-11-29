#ifndef C066DDDD_C6CC_41E7_97E3_662E7B160658
#define C066DDDD_C6CC_41E7_97E3_662E7B160658

#include "interfaces/motion_model.hpp"

/**
 * A differential drive odometry motion model
 */
class DiffOdomMotionModel : public MotionModel
{
public:
    DiffOdomMotionModel();
    geometry_msgs::TransformStamped getMostLikelyPose(const geometry_msgs::TransformStamped &prev_pose,
                                                      const geometry_msgs::TransformStamped &prev_odom,
                                                      const geometry_msgs::TransformStamped &curr_odom) override;
private:
    double alpha1_, alpha2_, alpha3_, alpha4_;  // motion model noise parameters
};

#endif /* C066DDDD_C6CC_41E7_97E3_662E7B160658 */
