#include "particle.hpp"

Particle::Particle(const geometry_msgs::TransformStamped &pose, const double &weight) : pose_(pose), weight_(weight) {}