cmake_minimum_required(VERSION 3.0.2)
project(tof_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tof_sensor
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  scripts/tof_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)