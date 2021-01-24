#include "monte_carlo_localisation/motion_utils.hpp"
#include "tf2/utils.h"
#include <cmath>

double MotionUtils::angleDiff(const double &a, const double &b)
{
    double angle = a - b;
    angle = fmod(angle, 2.0*M_PI);

    if(angle <= M_PI && angle >= -M_PI)
        return angle;

    else if(angle > M_PI)   
        return angle - 2.0*M_PI;
  
    else
        return angle + 2.0*M_PI;
}

bool MotionUtils::hasMoved(const geometry_msgs::TransformStamped &start,
                           const geometry_msgs::TransformStamped &end,
                           const double &trans_tol,
                           const double &rot_tol)
{
    double theta = tf2::getYaw(start.transform.rotation);
    double x = start.transform.translation.x;
    double y = start.transform.translation.y;

    double theta_prime = tf2::getYaw(end.transform.rotation);
    double x_prime = end.transform.translation.x;
    double y_prime = end.transform.translation.y;
    
    double translation = hypot(x_prime-x, y_prime-y);
    double rotation = abs(angleDiff(theta_prime, theta));

    if(translation > trans_tol || rotation > rot_tol)
        return true;
    else
        return false;
}

geometry_msgs::Quaternion MotionUtils::getQuat(const double &yaw)
{
    tf2::Quaternion tf_quat;
    geometry_msgs::Quaternion geo_quat;

    tf_quat.setRPY(0.0, 0.0, yaw);
    tf2::convert(tf_quat, geo_quat);

    return geo_quat;
}