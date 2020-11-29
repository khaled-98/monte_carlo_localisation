#include "motion_models/diff_odom_motion_model.hpp"
#include "motion_utils.hpp"
#include "tf2/utils.h"
#include <random>

DiffOdomMotionModel::DiffOdomMotionModel() : MotionModel("~")
{
    private_nh_.param("alpha1", alpha1_, 0.2);
    private_nh_.param("alpha2", alpha2_, 0.2);
    private_nh_.param("alpha3", alpha3_, 0.2);
    private_nh_.param("alpha4", alpha4_, 0.2);
}

geometry_msgs::TransformStamped DiffOdomMotionModel::getMostLikelyPose(const geometry_msgs::TransformStamped &prev_pose,
                                                                       const geometry_msgs::TransformStamped &prev_odom,
                                                                       const geometry_msgs::TransformStamped &curr_odom)
{
    double x_bar = prev_odom.transform.translation.x;
    double y_bar = prev_odom.transform.translation.y;
    double theta_bar = tf2::getYaw(prev_odom.transform.rotation);

    double x_bar_prime = curr_odom.transform.translation.x;
    double y_bar_prime = curr_odom.transform.translation.y;
    double theta_bar_prime = tf2::getYaw(curr_odom.transform.rotation);

    double x = prev_pose.transform.translation.x;
    double y = prev_pose.transform.translation.y;
    double theta = tf2::getYaw(prev_pose.transform.rotation);

    double delta_rot_1 = MotionUtils::angleDiff(atan2(y_bar_prime-y_bar, x_bar_prime-x_bar), theta_bar);
    if(isnan(delta_rot_1) || isinf(delta_rot_1))	
    {	
        delta_rot_1 = 0.0; // TODO: consider a different value	
    }	
    double delta_trans = hypot(x_bar_prime-x_bar, y_bar_prime-y_bar);	
    double delta_rot_2 = MotionUtils::angleDiff(MotionUtils::angleDiff(theta_bar_prime, theta_bar), delta_rot_1);	

    std::random_device device_;	
    std::mt19937 generator_(device_());	

    std::normal_distribution<double> delta_rot_1_noise_dist(0.0, sqrt(alpha1_*delta_rot_1 + alpha2_*delta_trans));	
    double delta_rot_1_hat = delta_rot_1 - delta_rot_1_noise_dist(generator_);	

    std::normal_distribution<double> delta_trans_noise_dist(0.0, sqrt(alpha3_*delta_trans + alpha4_*(delta_rot_1+delta_rot_2)));	
    double delta_trans_hat = delta_trans - delta_trans_noise_dist(generator_);	

    std::normal_distribution<double> delta_rot_2_noise_dist(0.0, sqrt(alpha1_*delta_rot_2 + alpha2_*delta_trans));	
    double delta_rot_2_hat = delta_rot_2 - delta_rot_2_noise_dist(generator_);	

    geometry_msgs::TransformStamped most_likely_pose;	
    most_likely_pose.transform.translation.x = x + delta_trans_hat*cos(theta + delta_rot_1_hat);	
    most_likely_pose.transform.translation.y = y + delta_trans_hat*sin(theta + delta_rot_1_hat);	

    tf2::Quaternion theta_prime_quat;	
    double theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;	
    theta_prime_quat.setEuler(theta_prime, 0.0, 0.0);	
    most_likely_pose.transform.rotation = tf2::toMsg(theta_prime_quat);	

    return most_likely_pose;
}