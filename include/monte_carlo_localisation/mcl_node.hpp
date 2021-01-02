#ifndef ACBE86F8_961A_4350_B53D_2DEAAD3CDBE7
#define ACBE86F8_961A_4350_B53D_2DEAAD3CDBE7

#include "ros/ros.h"
#include "motion_models/diff_odom_motion_model.hpp"
#include "measurement_models/likelihood_field_model.hpp"
#include "particle_filter.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/TransformStamped.h"

class MclNode
{
public:
    MclNode();

private:
    nav_msgs::OccupancyGrid getMap();
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan);
    void publishTransform(const geometry_msgs::TransformStamped &odom_to_base,
                          const geometry_msgs::TransformStamped &map_to_base);

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher particle_cloud_pub_;
    std::shared_ptr<DiffOdomMotionModel> motion_model_;
    std::shared_ptr<LikelihoodFieldModel> measurement_model_;
    std::shared_ptr<ParticleFilter> particle_filter_;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;
    std::string laser_topic_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
    geometry_msgs::TransformStamped prev_odom_;
    double linear_tol_, angular_tol_;
};

#endif /* ACBE86F8_961A_4350_B53D_2DEAAD3CDBE7 */