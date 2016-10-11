// Std
#include <vector>

#include <stdio.h>
#include <iostream>

// KDL
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

// ROS
#include <geometry_msgs/Pose.h>

std::vector<double> min_angles ;
std::vector<double> max_angles ;

std::vector<double> checkIK(geometry_msgs::Pose, bool&);
void initializeLimits() ;
bool isSolutionValid(const std::vector<double> &solution);
