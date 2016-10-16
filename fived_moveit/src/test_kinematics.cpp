//ROS
#include <ros/ros.h>

// MoveIt
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/RobotState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "test_kinematics");

  ros::NodeHandle nh ;

  // Wait a bit for ros things to initialize
  ros::WallDuration(10.0).sleep();
  moveit::planning_interface::MoveGroup group("five_dof_arm");

  ros::ServiceClient fk_client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk") ;
  ros::ServiceClient ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik") ;
  
  moveit_msgs::GetPositionFK fk_msg ;
  
  fk_msg.request.header.frame_id = "test1" ;
  
  // Fill in Link Names
  std::vector<std::string> link_names ;
  link_names.push_back("link4") ;
  fk_msg.request.fk_link_names = link_names ;

  
  // Fill in Robot State
  moveit_msgs::RobotState r_state ;
  r_state.joint_state.name.resize(5) ;
  r_state.joint_state.name[0] = "joint1" ;
  r_state.joint_state.name[1] = "joint2" ;
  r_state.joint_state.name[2] = "joint3" ;
  r_state.joint_state.name[3] = "joint4" ;
  r_state.joint_state.name[4] = "joint5" ;

  r_state.joint_state.position.resize(5) ;
  r_state.joint_state.position[0] = 0.0 ;
  r_state.joint_state.position[1] = 0.0 ;
  r_state.joint_state.position[2] = 0.0 ;
  r_state.joint_state.position[3] = 0.0 ;
  r_state.joint_state.position[4] = 0.0 ;
  
  // Call service
  fk_msg.request.robot_state = r_state ;
  fk_client.call(fk_msg) ;
  
  std::stringstream ss;
  std::cout.precision(5);
  
  ss << std::fixed << "X: " << fk_msg.response.pose_stamped[0].pose.position.x << "\n "; 
  ss << std::fixed << "Y: " << fk_msg.response.pose_stamped[0].pose.position.y << "\n "; 
  ss << std::fixed << "Z: " << fk_msg.response.pose_stamped[0].pose.position.z << "\n ";

  //*********************** Compute IK ***************************************//

  moveit_msgs::GetPositionIK ik_msg ;
  
  // Fill in group name
  ik_msg.request.ik_request.group_name = "five_dof_arm" ;

  // Fill in Robot State
  r_state.joint_state.position.resize(5) ;
  r_state.joint_state.position[0] = 0.0 ;
  r_state.joint_state.position[1] = 0.0 ;
  r_state.joint_state.position[2] = 0.0 ;
  r_state.joint_state.position[3] = 0.0 ;
  r_state.joint_state.position[4] = 0.0 ;
  
  ik_msg.request.ik_request.robot_state = r_state ;

  // Fill in ik_link_name   
  ik_msg.request.ik_request.ik_link_name = "link5" ;

  // Fill in Pose
  geometry_msgs::PoseStamped target_pose ;
  target_pose.pose.position.x = -0.0940014 ; 
  target_pose.pose.position.y =  0.176434  ;
  target_pose.pose.position.z =  0.537308  ;
  
  target_pose.pose.orientation.x = 0.526333 ; 
  target_pose.pose.orientation.y = 0.80995  ;
  target_pose.pose.orientation.z = 0.246604 ;
  target_pose.pose.orientation.w = -0.0783694 ;

  ik_msg.request.ik_request.pose_stamped = target_pose ;

  // Call service
  ik_client.call(ik_msg) ;

  ss << std::fixed << "Joint1: " << ik_msg.response.solution.joint_state.position[0] << "\n "; 
  ss << std::fixed << "Joint2: " << ik_msg.response.solution.joint_state.position[1] << "\n "; 
  ss << std::fixed << "Joint3: " << ik_msg.response.solution.joint_state.position[2] << "\n ";
  ss << std::fixed << "Joint4: " << ik_msg.response.solution.joint_state.position[3] << "\n "; 
  ss << std::fixed << "Joint5: " << ik_msg.response.solution.joint_state.position[4] << "\n "; 

  ROS_INFO_STREAM( ss.str() ) ;
  
}


