#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/jetson/ancabot_ws/devel/setup.bash 

# Start the main robot launch file in the background
roslaunch hexapod_bringup ancabot.launch &

# Running Node Index
NAVIGATION_MODE=1

# GPIO pin for the button - Pin 7 corresponds to gpio-216 on Jetson Nano
BUTTON=216

# Export GPIO pin
if [ ! -d "/sys/class/gpio/gpio$BUTTON" ]; then
    echo $BUTTON > /sys/class/gpio/export
fi

# Set direction
echo "in" > /sys/class/gpio/gpio$BUTTON/direction

# Function to cleanup GPIO pin on exit
cleanup() {
    echo $BUTTON > /sys/class/gpio/unexport
}

# Trap SIGINT (Ctrl+C) to cleanup GPIO on script exit
trap cleanup EXIT

# Loop until button is pressed
while true; do
    # Read button state
    button_state=$(cat /sys/class/gpio/gpio$BUTTON/value)
    
    # Check if button is pressed
    if [ "$button_state" -eq 1 ]; then
        # Execute navigation script
        roslaunch hexapod_bringup navigation_$NAVIGATION_MODE.launch &
        break  # Exit the loop after executing the script
    fi
    
    sleep 0.1  # Introduce a small delay to reduce CPU usage
done

# Unexport GPIO pin (cleanup will handle this)
cleanup

echo "Navigation script launched."
