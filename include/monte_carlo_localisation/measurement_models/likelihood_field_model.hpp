#ifndef FE0A2F84_4CF9_4948_BCE6_E9631BB04964
#define FE0A2F84_4CF9_4948_BCE6_E9631BB04964

#include "monte_carlo_localisation/interfaces/measurement_model.hpp"
#include <unordered_map>

/**
 * Likelihood field laser measurement model.
 */
class LikelihoodFieldModel : public MeasurementModel
{
public:
    LikelihoodFieldModel();
    void setMap(const nav_msgs::OccupancyGrid &map) override;
    double getProbability(const sensor_msgs::LaserScan::ConstPtr &scan,
                          const geometry_msgs::TransformStamped &curr_pose) override;
    void setLaserPose(const geometry_msgs::TransformStamped &laser_pose) override;

private:
    void preComputeLikelihoodField();	
	void DFS(const int &index_curr,
             const int &index_of_obstacle,
             std::vector<bool> &visited); //Deep First Search used in Likelihood field calculations.

    std::unordered_map<int, double>	likelihood_field_dist_;
	int max_number_of_beams_;       // The maximum number of laser beams to use in the calculations 
	double max_likelihood_distance_; // The distance beyond which the likelihood is 0
	// Model parameters
    double sigma_hit_;
	double z_hit_;
	double z_rand_;
	double z_max_;
	double z_min_;
};

#endif /* FE0A2F84_4CF9_4948_BCE6_E9631BB04964 */
