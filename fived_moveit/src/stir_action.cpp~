#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

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

#include <five_dof_arm_kinematics/ik_utils.h>

#define PI 3.14

int main(int argc, char **argv)
{
  ros::init (argc, argv, "stir_action");
  ros::AsyncSpinner spinner(3);
  spinner.start() ;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::NodeHandle nh;
 
  ros::Duration sleep_time(10.0);
 
  ros::WallDuration(10.0).sleep();

  moveit::planning_interface::MoveGroup group("five_dof_arm");
  group.setPlanningTime(45.0);

  //************** Initialize Limits ************************//

  initializeLimits() ;

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }


  //***************** Add Pan Object ******************************// 

  moveit_msgs::CollisionObject co;
  co.header.frame_id = group.getPlanningFrame();

  shapes::Mesh* m = shapes::createMeshFromResource("file://home/home/rohin/catkin_ws/src/five_d/fived_moveit/meshes/pan.STL");
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  co.meshes.resize(1);
  co.meshes[0] = co_mesh;
  co.mesh_poses.resize(1);
  co.mesh_poses[0].position.x = 0.3;
  co.mesh_poses[0].position.y = 0.0;
  co.mesh_poses[0].position.z = 0.1455;
  co.mesh_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw( 1.57, 0.0, 0.0 ) ;  
  co.id = "pan" ;

  co.meshes.push_back(co_mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(co);

  ROS_INFO("Add PAN Object to the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0); 

  //***************** Add Spatula Object *****************//

  moveit_msgs::CollisionObject co2;
  co2.header.frame_id = group.getPlanningFrame();

  shapes::Mesh* m2 = shapes::createMeshFromResource("file://home/home/rohin/catkin_ws/src/five_d/fived_moveit/meshes/spatula.STL");
  shape_msgs::Mesh co_mesh2;
  shapes::ShapeMsg co_mesh_msg2;
  shapes::constructMsgFromShape(m2,co_mesh_msg2);
  co_mesh2 = boost::get<shape_msgs::Mesh>(co_mesh_msg2);
  co2.meshes.resize(1);
  co2.meshes[0] = co_mesh2;
  co2.mesh_poses.resize(1);
  co2.mesh_poses[0].position.x = -0.12;
  co2.mesh_poses[0].position.y = 0.235;
  co2.mesh_poses[0].position.z = 0.1455;
  co2.mesh_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw( 1.57, 0.0, 3.14 ) ;  
  co2.id = "spatula" ;

  co2.meshes.push_back(co_mesh2);
  co2.mesh_poses.push_back(co2.mesh_poses[0]);
  co2.operation = co2.ADD;

  collision_objects.push_back(co2);

  ROS_INFO("Add SPATULA Object to world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0); 

  //************* Move Towards Spatula **********************//

  geometry_msgs::Pose start_pose;
  bool b ;
  std::vector<double> solution ;
  
  start_pose.position.x = 0.00549874 ;
  start_pose.position.y = 0.241382 ;
  start_pose.position.z = 0.374676 ;

  start_pose.orientation.x = -0.198037 ;
  start_pose.orientation.y = 0.2026 ;
  start_pose.orientation.z = 0.670369 ;
  start_pose.orientation.w = 0.685814 ;

  solution = checkIK(start_pose,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  } 

  sleep(5.0) ;

  start_pose.position.x = 0.00522976 ;
  start_pose.position.y = 0.23 ;
  start_pose.position.z = 0.1925 ;

  start_pose.orientation.x = -0.491534 ;
  start_pose.orientation.y = 0.502858 ;
  start_pose.orientation.z = 0.496998 ;
  start_pose.orientation.w = 0.508449 ;

  solution = checkIK(start_pose,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  } 
  
  //************* Attach SPATULA to arm ******************//

  ROS_INFO("Attach the object to the robot");
  group.attachObject(co2.id);
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(2.0);

  //************* Move Above PAN *************************//

  geometry_msgs::Pose pose2 ;
 
  pose2.position.x = 0.295955 ;
  pose2.position.y = 0.0 ;
  pose2.position.z = 0.432992 ;

  pose2.orientation.x = -0.698869 ;
  pose2.orientation.y = -0.0143049 ;
  pose2.orientation.z = -0.0139827 ;
  pose2.orientation.w = 0.71497 ;

  group.setPoseTarget(pose2) ;

  moveit::planning_interface::MoveGroup::Plan my_plan2;
  bool success2 = group.plan(my_plan2);
  group.move() ;
  sleep(2.0) ;

  //************ Move Into PAN ***************************//

  pose2.position.x = 0.295955 ;
  pose2.position.y = 0.0 ;
  pose2.position.z = 0.35 ;

  pose2.orientation.x = -0.698869 ;
  pose2.orientation.y = -0.0143049 ;
  pose2.orientation.z = -0.0139827 ;
  pose2.orientation.w = 0.71497 ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  
  //*********************** STIR *********************//

  sleep(3.0) ;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit::planning_interface::MoveGroup::Plan plan;

  pose2.position.x = 0.274971 ;
  pose2.position.y = -0.0308444 ;
  pose2.position.z = 0.34886 ;

  pose2.orientation.x = -0.706624 ;
  pose2.orientation.y = 0.0260839 ;
  pose2.orientation.z = -0.0528489 ;
  pose2.orientation.w = 0.70513 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.244983 ;
  pose2.position.y = -0.0393823 ;
  pose2.position.z = 0.347767 ;

  pose2.orientation.x = -0.705785 ;
  pose2.orientation.y = 0.0429884 ;
  pose2.orientation.z = -0.0695789 ;
  pose2.orientation.w = 0.703689 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.217128 ;
  pose2.position.y = -0.0139768 ;
  pose2.position.z = 0.346551 ;

  pose2.orientation.x = -0.707033 ;
  pose2.orientation.y = 0.00924188 ;
  pose2.orientation.z = -0.0361961 ;
  pose2.orientation.w = 0.706193 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.20687 ;
  pose2.position.y = 0.00272391 ;
  pose2.position.z = 0.346141 ;

  pose2.orientation.x = -0.706912 ;
  pose2.orientation.y = -0.0181956 ;
  pose2.orientation.z = -0.00888878 ;
  pose2.orientation.w = 0.707011 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.229898 ;
  pose2.position.y = 0.0309118 ;
  pose2.position.z = 0.343077 ;

  pose2.orientation.x = -0.711094 ;
  pose2.orientation.y = -0.0531236 ;
  pose2.orientation.z = 0.0412155 ;
  pose2.orientation.w = 0.699875 ;

  waypoints.push_back(pose2);
  
  //**//

  pose2.position.x = 0.256991 ;
  pose2.position.y = 0.0417086 ;
  pose2.position.z = 0.343383 ;

  pose2.orientation.x = -0.71031 ;
  pose2.orientation.y = -0.0628247 ;
  pose2.orientation.z = 0.0508506 ;
  pose2.orientation.w = 0.699233 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.286717 ;
  pose2.position.y = 0.030609 ;
  pose2.position.z = 0.344112 ;

  pose2.orientation.x = -0.711801 ;
  pose2.orientation.y = -0.0434431 ;
  pose2.orientation.z = 0.0316245 ;
  pose2.orientation.w = 0.700323 ;

  waypoints.push_back(pose2);

  //**//

  pose2.position.x = 0.304459 ;
  pose2.position.y = 0.0124458 ;
  pose2.position.z = 0.34466 ;

  pose2.orientation.x = -0.712802 ;
  pose2.orientation.y = -0.0201044 ;
  pose2.orientation.z = 0.00868609 ;
  pose2.orientation.w = 0.701024 ;

  waypoints.push_back(pose2);

  //**//

  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
       
                                      0.0,   // jump_threshold
                                             trajectory_msg);
  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
      fraction * 100.0);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  group.execute(plan);

  std::vector<double> RPY;
  geometry_msgs::PoseStamped current_pose ;
  current_pose = group.getCurrentPose() ;
  RPY = group.getCurrentRPY() ;

  std::stringstream ss;
  std::cout.precision(5);
  ss << "Final X: " << current_pose.pose.position.x << std::endl ;
  ss << "Final Y: " << current_pose.pose.position.y << std::endl ;
  ss << "Final Z: " << current_pose.pose.position.z << std::endl ;

  ss << "Roll: " << RPY[0] << std::endl ;
  ss << "Pitch: " << RPY[1] << std::endl ;
  ss << "Yaw: " << RPY[2] << std::endl ;

  ROS_INFO_STREAM( ss.str() ) ;
  
  ros::waitForShutdown();
  return 0;
}
