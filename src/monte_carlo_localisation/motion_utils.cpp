#include "motion_utils.hpp"
#include "tf2/utils.h"
#include <cmath>

double MotionUtils::angleDiff(const double &a, const double &b)
{
    double angle = a - b;
    angle = fmod(angle, 2.0*M_PI); // limit the angle from 0 to 2*Pi

    if(angle <= M_PI && angle >= -M_PI) // angle within the desired limit
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
    
    double translation = sqrt(pow(x_prime-x, 2) + pow(y_prime-y, 2));
    double rotation = abs(angleDiff(theta_prime, theta));

    if(translation > trans_tol || rotation > rot_tol)
        return true;
    else
        return false;
}