cmake_minimum_required(VERSION 3.0.2)
project(hexapod_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  rviz
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hexapod_description
#  CATKIN_DEPENDS joint_state_publisher joint_state_publisher_gui robot_state_publisher rviz xacro
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)