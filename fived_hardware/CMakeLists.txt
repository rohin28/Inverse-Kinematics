cmake_minimum_required(VERSION 2.8.3)
project(fived_hardware)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
  LIBRARIES
    fived_hw_control_loop
    fived_hw_interface
)

###########
## Build ##
###########2

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

# Control loop
add_library(fived_hw_control_loop src/fived_hw_control_loop.cpp)
target_link_libraries(fived_hw_control_loop
  ${catkin_LIBRARIES}
)

# Generic Hardware Interface
add_library(fived_hw_interface src/fived_hw_interface.cpp)
target_link_libraries(fived_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(fived_hw_main src/fived_hw_main.cpp)
target_link_libraries(fived_hw_main
  fived_hw_interface
  fived_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    fived_hw_control_loop
    fived_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    fived_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
