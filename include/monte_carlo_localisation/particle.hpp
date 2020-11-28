#ifndef A44CB69F_8F2F_42C0_9A22_F4D9B79AD5F6
#define A44CB69F_8F2F_42C0_9A22_F4D9B79AD5F6

#include "geometry_msgs/TransformStamped.h"

class Particle
{
public:
    Particle(const geometry_msgs::TransformStamped &pose, const double &weight);
    geometry_msgs::TransformStamped pose_;
    double weight_;
};

#endif /* A44CB69F_8F2F_42C0_9A22_F4D9B79AD5F6 */
