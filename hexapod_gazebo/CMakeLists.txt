cmake_minimum_required(VERSION 2.8.3)
project(hexapod_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  std_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)



# add support for C++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

