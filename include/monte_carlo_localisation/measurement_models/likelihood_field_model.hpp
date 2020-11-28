#ifndef FE0A2F84_4CF9_4948_BCE6_E9631BB04964
#define FE0A2F84_4CF9_4948_BCE6_E9631BB04964

#include "interfaces/measurement_model.hpp"

/**
 * Likelihood field laser measurement model.
 */
class LikelihoodFieldModel : public MeasurementModel
{
public:
    LikelihoodFieldModel(const geometry_msgs::TransformStamped &laser_pose,
                         const int &max_number_of_beams,
                         const double &max_likelihood_distance,
                         const double &sigma_hit,
                         const double &z_hit,
                         const double &z_rand);
    void setMap(const nav_msgs::OccupancyGrid::ConstPtr &map) override;
    double getProbability(const sensor_msgs::LaserScan::ConstPtr &scan,
                          const geometry_msgs::TransformStamped &curr_pose) override;
private:
    void preComputeLikelihoodField();	
	void DFS(const int &index_curr,
             const int &index_of_obstacle,
             std::vector<bool> &visited); //Deep First Search used in Likelihood field calculations.

	geometry_msgs::TransformStamped laser_pose_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::unordered_map<int, double>	pre_computed_likelihood_field_;
	int max_number_of_beams_;       // The maximum number of laser beams to use in the calculations 
	double max_likelihood_distance_; // The distance beyond which the likelihood is 0
	// Model parameters
    double sigma_hit_;
	double z_hit_;
	double z_rand_;
	double z_max_;
	double z_min_;
}

#endif /* FE0A2F84_4CF9_4948_BCE6_E9631BB04964 */
