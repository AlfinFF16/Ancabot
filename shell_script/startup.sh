#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/jetson/ancabot_ws/devel/setup.bash 

roslaunch hexapod_bringup ancabot.launch &

# Running Node Index
NAVIGATION_MODE=1

# GPIO pin for the button - Pin 7
BUTTON=216

# Export GPIO pin
echo $BUTTON > /sys/class/gpio/export

# Set direction
echo "in" > /sys/class/gpio/gpio$BUTTON/direction

# Loop until Ctrl+C
while true; do
    # Read button state
    variable=$(cat /sys/class/gpio/gpio$BUTTON/value)
    
    # Check if button is pressed
    if [ "$variable" -eq 1 ]; then
        # Execute navigation script
        roslaunch hexapod_bringup navigation_$NAVIGATION_MODE.launch &
        break  # Exit the loop after executing the script
    fi
    
    sleep 0.1  # Introduce a small delay to reduce CPU usage
done

# Unexport GPIO pin
echo $BUTTON > /sys/class/gpio/unexport
