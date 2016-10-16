#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// TF

#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "test_motion");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // We will use the :planning_scene_interface:`PlanningSceneInterface
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  


  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroup group("five_dof_arm");
  group.setPlanningTime(20.0);

  // wait a bit for ros things to initialize
  ros::WallDuration(7.0).sleep();

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  
  std::vector<double> joint_values;
  joint_values.resize(5) ;
  joint_values[0] = 0.0 ;
  joint_values[1] = 1.0 ;
  joint_values[2] = 1.72 ;
  joint_values[3] = 0.39 ;
  joint_values[4] = 0.0 ;

  group.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroup::Plan my_plan3;
  bool success3 = group.plan(my_plan3);

  group.move() ;
  sleep(5.0) ;

  ros::waitForShutdown();
  return 0;
}
