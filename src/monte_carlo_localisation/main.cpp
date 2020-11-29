#include "ros/ros.h"
#include "monte_carlo_localisation/mcl_node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mcl_localisation");
  MclNode mcl_node;
  ros::spin();
  return 0;
}