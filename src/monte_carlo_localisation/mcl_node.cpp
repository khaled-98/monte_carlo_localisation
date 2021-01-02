#include "monte_carlo_localisation/mcl_node.hpp"
#include "monte_carlo_localisation/motion_utils.hpp"
#include "nav_msgs/GetMap.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

MclNode::MclNode() : private_nh_("~")
{
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
    private_nh_.param("laser_topic", laser_topic_, std::string("base_scan"));
    private_nh_.param("linear_tol", linear_tol_, 0.1);
    private_nh_.param("angular_tol", angular_tol_, 0.087);  // ~ 5deg in rads

    particle_cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 100);
    
    measurement_model_ = std::make_shared<LikelihoodFieldModel>();
    measurement_model_->setMap(getMap());    
    motion_model_ = std::make_shared<DiffOdomMotionModel>();

    laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    laser_scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                          *tf_buffer_,
                                                                                          odom_frame_id_,
                                                                                          100,
                                                                                          nh_);

    laser_scan_filter_->registerCallback(std::bind(&MclNode::laserScanCallback, this, std::placeholders::_1));
    particle_filter_ = std::make_shared<ParticleFilter>(measurement_model_, motion_model_);    
}

nav_msgs::OccupancyGrid MclNode::getMap()
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

void MclNode::laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    static bool laser_pose_set{false};
    if(!laser_pose_set)
    {
        ROS_INFO("Getting laser pose...");
        measurement_model_->setLaserPose(tf_buffer_->lookupTransform(base_frame_id_, scan->header.frame_id, ros::Time(0)));
        ROS_INFO("Got laser pose!");
        laser_pose_set = true;
    }
    
    try
    {
        static bool first_run{true};
        geometry_msgs::TransformStamped curr_odom = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, scan->header.stamp, ros::Duration(1.0));
        
        // FIXME: There has to be a more elegant way to do this.
        if(first_run)
            prev_odom_ = curr_odom;

        if(first_run || MotionUtils::hasMoved(prev_odom_, curr_odom, linear_tol_, angular_tol_))
        {
            particle_filter_->update(prev_odom_, curr_odom, scan);
            prev_odom_ = curr_odom;
        }

        geometry_msgs::TransformStamped map_to_base = particle_filter_->getMostLikelyPose();
        publishTransform(curr_odom, map_to_base);
        
        geometry_msgs::PoseArray pose_array;
        pose_array = particle_filter_->getPoses();
        pose_array.header.frame_id = global_frame_id_;
        pose_array.header.stamp = ros::Time::now();
        particle_cloud_pub_.publish(pose_array);
    
        if(first_run)
            first_run = false;
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what());
    }
    
}

void MclNode::publishTransform(const geometry_msgs::TransformStamped &odom_to_base,
                               const geometry_msgs::TransformStamped &map_to_base)
{
    tf2::Stamped<tf2::Transform> map_to_base_transform;
    tf2::fromMsg(map_to_base, map_to_base_transform);

    tf2::Stamped<tf2::Transform> odom_to_base_transform;
    tf2::fromMsg(odom_to_base, odom_to_base_transform);

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.transform = tf2::toMsg(map_to_base_transform.inverseTimes(odom_to_base_transform));
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = global_frame_id_;
    map_to_odom.child_frame_id = odom_frame_id_;

    tf_broadcaster_->sendTransform(map_to_odom);
}