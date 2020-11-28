#ifndef E380396B_DB16_441E_A151_513A29E95EB5
#define E380396B_DB16_441E_A151_513A29E95EB5

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"

class MeasurementModel
{
public:
    virtual void setMap(const nav_msgs::OccupancyGrid &map) = 0;
    virtual double getProbability(const sensor_msgs::LaserScan &scan,
                                  const geometry_msgs::TransformStamped &curr_pose) = 0;
};

#endif /* E380396B_DB16_441E_A151_513A29E95EB5 */
