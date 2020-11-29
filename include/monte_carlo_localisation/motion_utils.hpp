#ifndef F131C896_98D6_4D16_A90D_4E1BCFDDFB2E
#define F131C896_98D6_4D16_A90D_4E1BCFDDFB2E

#include "geometry_msgs/TransformStamped.h"

class MotionUtils
{
public:
    static double angleDiff(const double &a, const double &b);
    static bool hasMoved(const geometry_msgs::TransformStamped &start,
                         const geometry_msgs::TransformStamped &end,
                         const double &trans_tol,
                         const double &rot_tol);
};

#endif /* F131C896_98D6_4D16_A90D_4E1BCFDDFB2E */
