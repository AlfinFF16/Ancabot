#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/jetson/Ancabots/devel/setup.bash 

echo "Launching application, yea please wait!"
roslaunch hexapod_bringup ancabot.launch

# Common path for all GPIO access
BASE_GPIO_PATH=/sys/class/gpio

# Assign name to GPIO pin number for the button
BUTTON=11

# Assign names to states
ON="1"
OFF="0"

# Utility function to export a pin if not already exported
exportPin()
{
  if [ ! -e $BASE_GPIO_PATH/gpio$1 ]; then
    echo "$1" > $BASE_GPIO_PATH/export
  fi
}

# Utility function to set a pin as an input
setInput()
{
  echo "in" > $BASE_GPIO_PATH/gpio$1/direction
}

# Utility function to read the state of an input pin
readInput()
{
  cat $BASE_GPIO_PATH/gpio$1/value
}

# Ctrl-C handler for clean shutdown
shutdown()
{
  exit 0
}

trap shutdown SIGINT

# Export the button pin so that we can use it
exportPin $BUTTON

# Set the button pin as input
setInput $BUTTON

# Loop forever until user presses Ctrl-C
while [ 1 ]
do
  # Check if button is pressed
  if [ $(readInput $BUTTON) -eq 0 ]; then
    # Button is pressed, execute rosrun command
    # Replace the following command with your actual rosrun command
    rosrun navigation  nav24_1&
    # Wait for the button to be released
    while [ $(readInput $BUTTON) -eq 0 ]; do
      sleep 0.1
    done
  fi
done

