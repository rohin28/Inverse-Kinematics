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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// TF

#include <tf/transform_datatypes.h>


void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = 0.24616;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.163348;
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.699065;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 0.715059;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.z = -1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;

  g.pre_grasp_posture.joint_names.resize(1, "joint5");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 0;

  g.grasp_posture.joint_names.resize(1, "joint5");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  grasps.push_back(g);
  group.pick("box", grasps);
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "pick_box");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // We will use the :planning_scene_interface:`PlanningSceneInterface
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  


  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration sleep_time(10.0);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("five_dof_arm");
  //moveit::planning_interface::MoveGroup eff("end_effector");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  moveit_msgs::AttachedCollisionObject aco;

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.1;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.25;
	box_pose.position.y = 0.0;
	box_pose.position.z =  0.05;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	// Now, let's add the collision object into the world
	  ROS_INFO("Add an object into the world");  
	  planning_scene_interface.addCollisionObjects(collision_objects);
          //pub_co.publish(co) ;
	  
	  /* Sleep so we have time to see the object in RViz */
	  sleep(5.0);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  //target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,-1.5707,1.5707) ;
  target_pose1.position.x = 0.191799;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.189095;

  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.699065;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.715059;
 
  group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  //moveit::planning_interface::MoveGroup::Plan my_plan;
  //bool success = group.plan(my_plan);

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(1.0);

  group.move() ;

  sleep(1.0) ;
  
  geometry_msgs::Pose target_pose2;
 
  target_pose2.position.x = 0.24616;
  target_pose2.position.y = 0.0;
  target_pose2.position.z = 0.1400;

  target_pose2.orientation.x = 0.0;
  target_pose2.orientation.y = 0.699065;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.715059;
 
  group.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  //moveit::planning_interface::MoveGroup::Plan my_plan2;
  //bool success2 = group.plan(my_plan2);

  //ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(1.0);

  group.move() ;

  sleep(1.0) ;

  ROS_INFO("Attach the object to the robot");
  group.attachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(2.0);

  //ROS_INFO("Remove the object from the world");
  //std::vector<std::string> object_ids;
  //object_ids.push_back(collision_object.id);
  //planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  //sleep(4.0);

  geometry_msgs::Pose target_pose3;
 
  target_pose3.position.x = 0.442839;
  target_pose3.position.y = 0.0;
  target_pose3.position.z = 0.235561;

  target_pose3.orientation.x = 0.0;
  target_pose3.orientation.y = 0.0;
  target_pose3.orientation.z = 0.0;
  target_pose3.orientation.w = 1.0;
 
  group.setPoseTarget(target_pose3);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  //moveit::planning_interface::MoveGroup::Plan my_plan3;
  //bool success3 = group.plan(my_plan3);

  //ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(1.0);

  group.move() ;

  sleep(1.0) ;
 
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose pose4,pose5,pose6 ;

  //Pose 4
  pose4.position.x = 0.442839;
  pose4.position.y = 0.0;
  pose4.position.z = 0.235561;

  pose4.orientation.x = 0.506335 ; 
  pose4.orientation.y = 0.0 ;
  pose4.orientation.z = 0.0 ;
  pose4.orientation.w = 0.862337 ;

  waypoints.push_back(pose4);  

  //Pose 5
  pose5.position.x = 0.442839;
  pose5.position.y = 0.0;
  pose5.position.z = 0.235561;

  pose5.orientation.x = 0.99382 ; 
  pose5.orientation.y = 0.0 ;
  pose5.orientation.z = 0.0 ;
  pose5.orientation.w = 0.110359 ;

  waypoints.push_back(pose5);  

  //Pose 6
  
  pose6.position.x = 0.442839;
  pose6.position.y = 0.0;
  pose6.position.z = 0.235561;

  pose6.orientation.x = 0.952502 ; 
  pose6.orientation.y = 0.0 ;
  pose6.orientation.z = 0.0 ;
  pose6.orientation.w = -0.304532 ;

  waypoints.push_back(pose6);  // up and out
  
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                             0.05,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);

  group.move() ;
  sleep(10.0) ;
  
  geometry_msgs::Pose target_pose7;
 
  target_pose7.position.x = 0.172617;
  target_pose7.position.y = -0.192845;
  target_pose7.position.z = 0.192608;

  target_pose7.orientation.x = 0.285363;
  target_pose7.orientation.y = 0.638415;
  target_pose7.orientation.z = -0.291708;
  target_pose7.orientation.w = 0.652611;
 
  group.setPoseTarget(target_pose7);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan3;
  bool success3 = group.plan(my_plan3);

  ROS_INFO("Visualizing plan 4 (pose goal) %s",success3?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(2.0);

  group.move() ;

  sleep(1.0) ;
  
  geometry_msgs::Pose target_pose8;
 
  target_pose8.position.x = 0.172617;
  target_pose8.position.y = -0.192845;
  target_pose8.position.z = 0.1400;

  target_pose8.orientation.x = 0.285363;
  target_pose8.orientation.y = 0.638415;
  target_pose8.orientation.z = -0.291708;
  target_pose8.orientation.w = 0.652611;
 
  group.setPoseTarget(target_pose8);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan5;
  bool success5 = group.plan(my_plan5);

  ROS_INFO("Visualizing plan 5 (pose goal) %s",success5?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(2.0);

  group.move() ;

  sleep(1.0) ;
 
  ROS_INFO("Dettach the object from the robot");
  group.detachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(2.0);

  geometry_msgs::Pose target_pose9;
 
  target_pose9.position.x = 0.172617;
  target_pose9.position.y = -0.192845;
  target_pose9.position.z = 0.1900;

  target_pose9.orientation.x = 0.285363;
  target_pose9.orientation.y = 0.638415;
  target_pose9.orientation.z = -0.291708;
  target_pose9.orientation.w = 0.652611;
 
  group.setPoseTarget(target_pose9);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  //moveit::planning_interface::MoveGroup::Plan my_plan6;
  //bool success6 = group.plan(my_plan6);

  //ROS_INFO("Visualizing plan 6 (pose goal) %s",success6?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(2.0);

  group.move() ;
 
 
  

  ros::waitForShutdown();
  return 0;
}
