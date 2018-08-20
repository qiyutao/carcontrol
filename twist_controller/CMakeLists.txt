cmake_minimum_required(VERSION 2.8.3)
project(twist_controller)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_node_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  can_node_msg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs can_node_msg control_node_msgs message_runtime
  INCLUDE_DIRS include
)

# add_library(
#   lowpass_filter 
#   include/lowpass.h
#   src/lowpass.cpp
# )

# add_library(
#   pid_controller 
#   include/pid.h
#   src/pid.cpp
# )

# add_library(
#   t_controller
#   include/twist_controller.h
#   src/twist_controller.cpp
# )

# add_library(
#   yaw_controller
#   include/yaw_controller.h
#   src/yaw_controller.cpp
# )

add_executable(twist_controller src/dbw_node.cpp src/lowpass.cpp src/pid.cpp src/twist_controller.cpp src/yaw_controller.cpp)
target_link_libraries(twist_controller
   ${catkin_LIBRARIES}
)
