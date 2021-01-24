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
	for(auto index=0; index < map_.data.size(); index++)
	{
		if(map_.data[index]==100) // the cell is occupied
		{
			likelihood_field_dist_[index] = 0.0;
			occupied_cells.push_back(index);
		}
		else
			likelihood_field_dist_[index] = max_likelihood_distance_;
	}

    // Depth first search for other cells
	for(auto index : occupied_cells)
	{
		std::vector<bool> visited(map_.data.size(), false);
		DFS(index, index, visited);
	}
}

void LikelihoodFieldModel::DFS(const int &index_curr,
                               const int &index_of_obstacle,
                               std::vector<bool> &visited)
{
    visited[index_curr] = true;
	if(index_curr<0 || index_curr>=map_.data.size())
		return;

	std::pair<uint32_t, uint32_t> coord_curr = MapUtils::mapIndexToCoord(index_curr, map_.info.width);
	std::pair<uint32_t, uint32_t> coord_obs = MapUtils::mapIndexToCoord(index_of_obstacle, map_.info.width);

	// This cell is NOT an obstacle
	if(likelihood_field_dist_[index_curr]!=0.0)	
	{
		double distance_to_obstacle = MapUtils::distanceBetweenTwoPoints(coord_curr.first, coord_curr.second, coord_obs.first, coord_obs.second)*map_.info.resolution;

		// Getting too far from the obstacle
		if(distance_to_obstacle > max_likelihood_distance_)
			return;

		// Found a closer obstacle
		if(distance_to_obstacle < likelihood_field_dist_[index_curr])
			likelihood_field_dist_[index_curr] = distance_to_obstacle;
	}

	// left
	if(coord_curr.first > 0)
	{
		int left_cell_index =  MapUtils::mapCoordToIndex(coord_curr.first-1, coord_curr.second, map_.info.width);
		if(!visited[left_cell_index])
			DFS(left_cell_index, index_of_obstacle, visited);
	}

	// right
	if(coord_curr.first < map_.info.width-1)
	{
		int right_cell_index =  MapUtils::mapCoordToIndex(coord_curr.first+1, coord_curr.second, map_.info.width);
		if(!visited[right_cell_index])
			DFS(right_cell_index, index_of_obstacle, visited);
	}

	// up
	if(coord_curr.second > 0)
	{
		int up_cell_index =  MapUtils::mapCoordToIndex(coord_curr.first, coord_curr.second-1, map_.info.width);
		if(!visited[up_cell_index])
			DFS(up_cell_index, index_of_obstacle, visited);
	}

	// down
	if(coord_curr.second < map_.info.height-1)
	{
		int down_cell_index =  MapUtils::mapCoordToIndex(coord_curr.first, coord_curr.second+1, map_.info.width);
		if(!visited[down_cell_index])
			DFS(down_cell_index, index_of_obstacle, visited);
	}
}

double LikelihoodFieldModel::getProbability(const sensor_msgs::LaserScan::ConstPtr &scan,
                                            const geometry_msgs::TransformStamped &curr_pose)
{
    double q = 1.0;
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
			std::pair<int, int> map_coord = MapUtils::worldToMapCoord(x_z_kt, y_z_kt, map_.info);
			int map_x = map_coord.first;
			int map_y = map_coord.second;
			int map_index = MapUtils::mapCoordToIndex(map_x, map_y, map_.info.width);

			double dist;
			if(map_index < 0 || map_index>=map_.data.size())
				dist = max_likelihood_distance_;
			else
				dist = likelihood_field_dist_[map_index];
			
			double prob = (1.0/(sigma_hit_*sqrt(2*M_PI)))*exp(-(dist*dist)/(2*sigma_hit_*sigma_hit_));
			double weight = z_hit_* prob + (z_rand_/z_max_);
			q *= weight;
		}
	}
	return q;
}

void LikelihoodFieldModel::setLaserPose(const geometry_msgs::TransformStamped &laser_pose)
{
	laser_pose_ = laser_pose;
}