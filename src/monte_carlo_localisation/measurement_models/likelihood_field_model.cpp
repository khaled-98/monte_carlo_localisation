#include "monte_carlo_localisation/measurement_models/likelihood_field_model.hpp"
#include "monte_carlo_localisation/map_utils.hpp"
#include "tf2/utils.h"

LikelihoodFieldModel::LikelihoodFieldModel() : MeasurementModel("~")
{
	private_nh_.param("max_likelihood_distance", max_likelihood_distance_, 2.0);
	private_nh_.param("max_number_of_beams", max_number_of_beams_, 30);
	private_nh_.param("z_hit", z_hit_, 0.95);
    private_nh_.param("sigma_hit", sigma_hit_, 0.2);
    private_nh_.param("z_rand", z_rand_, 0.05);
}

void LikelihoodFieldModel::setMap(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
	ROS_INFO("Computing likelihood field...");
	preComputeLikelihoodField();
	ROS_INFO("Likelihood field computed!");
}

void LikelihoodFieldModel::preComputeLikelihoodField()
{
    std::vector<int> occupied_cells;

    // Identify all the occupied cells
	for(auto index=0; index < map_.info.width*map_.info.height; index++)
	{
		if(map_.data[index]==100) // the cell is occupied
		{
			pre_computed_likelihood_field_[index] = 0.0;
			occupied_cells.push_back(index);
		}
		else
			pre_computed_likelihood_field_[index] = max_likelihood_distance_;
	}

    // Depth first search for other cells
	for(auto index : occupied_cells)
	{
		std::vector<bool> visited(map_.info.width*map_.info.height, false);
		DFS(index, index, visited);
	}

    // Apply zero-mean norrmal distribution
	for(auto index=0; index < map_.info.width*map_.info.height; index++)
		pre_computed_likelihood_field_[index] = (1.0/(sqrt(2.0*M_PI)*sigma_hit_))*exp(-0.5*(pow(pre_computed_likelihood_field_[index], 2)/pow(sigma_hit_, 2)));
}

void LikelihoodFieldModel::DFS(const int &index_curr,
                               const int &index_of_obstacle,
                               std::vector<bool> &visited)
{
    visited[index_curr] = true;
	std::pair<uint32_t, uint32_t> coord_curr = MapUtils::index_to_coordinates(index_curr, map_.info.width);
	std::pair<uint32_t, uint32_t> coord_obs = MapUtils::index_to_coordinates(index_of_obstacle, map_.info.width);

	// This cell is NOT an obstacle
	if(pre_computed_likelihood_field_[index_curr]!=0.0)	
	{
		double distance_to_obstacle = MapUtils::distance_between_two_points(coord_curr.first, coord_curr.second, coord_obs.first, coord_obs.second)*map_.info.resolution;

		// Getting far from the obstacle
		if(distance_to_obstacle > max_likelihood_distance_)
			return;

		// Found a closer obstacle
		if(distance_to_obstacle < pre_computed_likelihood_field_[index_curr])
			pre_computed_likelihood_field_[index_curr] = distance_to_obstacle;
	}

	// left
	if(coord_curr.first > 0)
	{
		int left_cell_index =  MapUtils::coordinates_to_index(coord_curr.first-1, coord_curr.second, map_.info.width);
		if(!visited[left_cell_index])
			DFS(left_cell_index, index_of_obstacle, visited);
	}

	// right
	if(coord_curr.first < map_.info.width-1)
	{
		int right_cell_index =  MapUtils::coordinates_to_index(coord_curr.first+1, coord_curr.second, map_.info.width);
		if(!visited[right_cell_index])
			DFS(right_cell_index, index_of_obstacle, visited);
	}

	// up
	if(coord_curr.second > 0)
	{
		int up_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second-1, map_.info.width);
		if(!visited[up_cell_index])
			DFS(up_cell_index, index_of_obstacle, visited);
	}

	// down
	if(coord_curr.second < map_.info.height-1)
	{
		int down_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second+1, map_.info.width);
		if(!visited[down_cell_index])
			DFS(down_cell_index, index_of_obstacle, visited);
	}
}

double LikelihoodFieldModel::getProbability(const sensor_msgs::LaserScan::ConstPtr &scan,
                                            const geometry_msgs::TransformStamped &curr_pose)
{
    double q = 1;
	double theta = tf2::getYaw(curr_pose.transform.rotation);

	z_max_ = scan->range_max;
	z_min_ = scan->range_min;

	// in case the user specfies more beams than is avaiable
	int max_number_of_beams = std::min(max_number_of_beams_, static_cast<int>(scan->ranges.size()));
	int beams_to_skip = static_cast<int>(scan->ranges.size())/max_number_of_beams;
	for(int i=0; i<scan->ranges.size(); i+=beams_to_skip)
	{
		if(scan->ranges[i]<z_max_ && scan->ranges[i]>z_min_) // within sensor range
		{
			double beam_angle = tf2::getYaw(laser_pose_.transform.rotation) + 
								scan->angle_min + scan->angle_increment*i;
			double x_z_kt = curr_pose.transform.translation.x +
							laser_pose_.transform.translation.x * cos(theta) -
							laser_pose_.transform.translation.y * sin(theta) +
							scan->ranges[i] * 
							cos(theta + beam_angle);
			double y_z_kt = curr_pose.transform.translation.y +
							laser_pose_.transform.translation.y * cos(theta) +
							laser_pose_.transform.translation.x * sin(theta) +
							scan->ranges[i] * 
							sin(theta + beam_angle + tf2::getYaw(laser_pose_.transform.rotation));
			int end_point_index = MapUtils::coordinates_to_index(x_z_kt, y_z_kt, map_.info.width);
			double dist_prob = pre_computed_likelihood_field_[end_point_index];

			q *= (z_hit_* dist_prob + (z_rand_/z_max_));
		}
	}
	return q;
}

void LikelihoodFieldModel::setLaserPose(const geometry_msgs::TransformStamped &laser_pose)
{
	laser_pose_ = laser_pose;
}