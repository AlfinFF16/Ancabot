#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/jetson/ancabot_ws/devel/setup.bash 

# Start the main robot launch file in the background
roslaunch hexapod_bringup ancabot.launch &

