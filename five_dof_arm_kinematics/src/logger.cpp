/*
 * logger.cpp
 *
 *  Created on: 31-Aug-2016
 *      Author: rohin
 */

#include <five_dof_arm_kinematics/logger.h>

using namespace five_dof_arm_kinematics;

Logger Logger::null;


Logger::Logger()
{
}


Logger::~Logger()
{
}


void Logger::write(const std::string &msg, const char *file, int line)
{
}
