cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  control_node_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime control_node_msgs
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lane_detector.cpp
# )

add_executable(${PROJECT_NAME}_node src/lane_detector_node.cpp src/lane_detector_core.cpp src/lane_detect_algo.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)