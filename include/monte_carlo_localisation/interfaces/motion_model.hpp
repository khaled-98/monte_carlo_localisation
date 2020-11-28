#ifndef D3467842_8472_4BD8_B524_8933E6C1D0AB
#define D3467842_8472_4BD8_B524_8933E6C1D0AB

#include "geometry_msgs/TransformStamped.h"

class MotionModel
{
public:
    virtual geometry_msgs::TransformStamped getMostLikelyPose(const geometry_msgs::TransformStamped &prev_pose,
                                                              const geometry_msgs::TransformStamped &prev_odom,
                                                              const geometry_msgs::TransformStamped &curr_odom) = 0;
};

#endif /* D3467842_8472_4BD8_B524_8933E6C1D0AB */
