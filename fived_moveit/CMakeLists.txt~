cmake_minimum_required(VERSION 2.8.3)
project(fived_moveit)

find_package(catkin REQUIRED COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             joint_trajectory_controller
             joint_trajectory_action
             moveit_simple_controller_manager
             industrial_robot_simulator
             actionlib
             actionlib_msgs
             control_msgs
             trajectory_msgs
             moveit_msgs
             shape_msgs
             five_dof_arm_kinematics
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
    ik_utils
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    std_msgs
    actionlib_msgs
    shape_msgs
    five_dof_arm_kinematics
)

add_executable(pick_box src/pick_box.cpp)
target_link_libraries(pick_box ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_box DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(stir_action src/stir_action.cpp)
target_link_libraries(stir_action ik_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS stir_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(masala_dispensing src/masala_dispensing.cpp)
target_link_libraries(masala_dispensing ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS masala_dispensing DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_motion src/test_motion.cpp)
target_link_libraries(test_motion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS test_motion DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_kinematics src/test_kinematics.cpp)
target_link_libraries(test_kinematics ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS test_kinematics DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(maggi src/maggi.cpp)
target_link_libraries(maggi ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS maggi DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
