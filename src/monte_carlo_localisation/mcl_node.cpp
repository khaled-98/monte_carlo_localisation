#include "mcl_node.hpp"
#include "nav_msgs/GetMap.h"

MclNode::MclNode()
{
    measurement_model_ = std::make_shared<LikelihoodFieldModel>();
    measurement_model_->setMap(getMap());    
    motion_model_ = std::make_shared<DiffOdomMotionModel>();
    
    particle_filter_ = std::make_shared<ParticleFilter>(measurement_model_, motion_model_);    
}

nav_msgs::OccupancyGrid getMap()
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;

    ROS_INFO("Waiting for map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    ROS_INFO("Map received!");
    
    return resp.map;
}