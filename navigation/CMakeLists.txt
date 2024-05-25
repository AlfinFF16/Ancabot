cmake_minimum_required(VERSION 3.0.2)
project(navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hexapod_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  ultralytics_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation
#  CATKIN_DEPENDS geometry_msgs hexapod_msgs move_base nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${ultralytics_ros_INCLUDE_DIRS}
)
add_executable(nav24_1 src/nav24_1.cpp)
target_link_libraries(nav24_1 ${catkin_LIBRARIES})

add_executable(sensor_debugger src/sensor_debugger.cpp)
target_link_libraries(sensor_debugger ${catkin_LIBRARIES})

add_executable(navigation_logger src/navigation_logger.cpp)
target_link_libraries(navigation_logger ${catkin_LIBRARIES})