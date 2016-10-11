cmake_minimum_required(VERSION 2.8.3)
project(five_dof_arm_kinematics_moveit)

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    moveit_core
    urdf
    tf_conversions
    five_dof_arm_kinematics
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(five_dof_arm_kinematics_moveit
  src/five_dof_arm_kinematics_moveit.cpp
  src/kinematics_logger.cpp
)

target_link_libraries(five_dof_arm_kinematics_moveit
  ${catkin_LIBRARIES}
)

install(TARGETS five_dof_arm_kinematics_moveit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_gtest(load_moveit_plugin_test
  test/load_moveit_plugin_test.cpp
)

target_link_libraries(load_moveit_plugin_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(five_dof_arm_kinematics_moveit_test
  test/five_dof_arm_kinematics_moveit_test.cpp
)

target_link_libraries(five_dof_arm_kinematics_moveit_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(configuration_comparator_test
  test/configuration_comparator_test.cpp
)
