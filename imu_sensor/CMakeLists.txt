cmake_minimum_required(VERSION 3.0.2)
project(imu_sensor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_sensor
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/imu_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)