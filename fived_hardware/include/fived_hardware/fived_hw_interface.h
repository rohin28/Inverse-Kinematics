/*
 * fived_hw_interface.h
 *
 *  Created on: 28-Jun-2016
 *      Author: rohin
 */

#ifndef FIVED_HW_INTERFACE_H_
#define FIVED_HW_INTERFACE_H_

// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <std_msgs/Float64MultiArray.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <hardware_interface/posvel_command_interface.h>

/// \brief Hardware interface for a robot
class FiveDHW : public hardware_interface::RobotHW
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   * \param urdf - optional pointer to a parsed robot model
   */
  FiveDHW(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

  /** \brief Destructor */
  virtual ~FiveDHW() {}

  /** \brief Initialize the hardware interface */
  void init();

  /** \brief Read the state from the robot hardware. */
  void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  void write(ros::Duration &elapsed_time);
  
  ros::Publisher serial_command ;
  ros::Subscriber readEncoderFeedback ;

  /** \brief Set all members to default values */
  void reset();

  /**
   * \brief Check (in non-realtime) if given controllers could be started and stopped from the
   * current state of the RobotHW
   * with regard to necessary hardware interface switches. Start and stop list are disjoint.
   * This is just a check, the actual switch is done in doSwitch()
   */
  bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                         const std::list<hardware_interface::ControllerInfo> &stop_list) const
  {
    return true;
  }

  /**
   * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
   * and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
  }

  /**
   * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
   * are retrieved from the urdf_model.
   *
   * \return the joint's type, lower position limit, upper position limit, and effort limit.
   */
  void registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                           const hardware_interface::JointHandle &joint_handle_velocity,
                           const hardware_interface::JointHandle &joint_handle_effort,
                           std::size_t joint_id);

  /** \breif Enforce limits for all values before writing */
  void enforceLimits(ros::Duration &period);

  /** \brief Helper for debugging a joint's state */
  void printState();
  std::string printStateHelper();

  /** \brief Helper for debugging a joint's command */
  std::string printCommandHelper();
  
  std_msgs::Float64MultiArray rosserial_command;

protected:

  /** \brief Get the URDF XML from the parameter server */
  void loadURDF(ros::NodeHandle& nh, std::string param_name);

  // Short name of this class
  std::string name_;

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  //hardware_interface::PosVelJointInterface posvel_joint_interface_;

  // Joint limits interfaces - Saturation
  joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
  joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;

  // Joint limits interfaces - Soft limits
  joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
  joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

  // Configuration
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;
  urdf::Model *urdf_model_;

  // Modes
  bool use_rosparam_joint_limits_;
  bool use_soft_limits_if_available_;

  // States
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  //std::vector<double> joint_pos2_command_ ;
  //std::vector<double> joint_vel2_command_ ;

  // Copy of limits, in case we need them later in our control stack
  std::vector<double> joint_position_lower_limits_;
  std::vector<double> joint_position_upper_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_effort_limits_;

};  // class

#endif /*FIVED_HW_INTERFACE_H_ */
