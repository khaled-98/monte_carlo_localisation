#ifndef E380396B_DB16_441E_A151_513A29E95EB5
#define E380396B_DB16_441E_A151_513A29E95EB5

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"

class MeasurementModel
{
public:
    virtual void setMap(const nav_msgs::OccupancyGrid &map) = 0;
    virtual double getProbability(const sensor_msgs::LaserScan::ConstPtr &scan,
                                  const geometry_msgs::TransformStamped &curr_pose) = 0;
    virtual void setLaserPose(const geometry_msgs::TransformStamped &laser_pose) = 0;
protected:
    MeasurementModel(std::string node_namespace) : private_nh_(node_namespace) {}; 
    
    ros::NodeHandle private_nh_;
    nav_msgs::OccupancyGrid map_;
	geometry_msgs::TransformStamped laser_pose_;
};

#endif /* E380396B_DB16_441E_A151_513A29E95EB5 */
