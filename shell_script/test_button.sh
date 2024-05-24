#!/bin/bash

# GPIO pin for the button - Pin 7 corresponds to gpio-216 on Jetson Nano
BUTTON=216

# Export GPIO pin
if [ ! -d "/sys/class/gpio/gpio$BUTTON" ]; then
    echo $BUTTON > /sys/class/gpio/export
fi

# Set direction
echo "in" > /sys/class/gpio/gpio$BUTTON/direction

# Loop until button is pressed
while true; do
    # Read button state
    button_state=$(cat /sys/class/gpio/gpio$BUTTON/value)

    # Check if button is pressed
    if [ "$button_state" -eq 1 ]; then
        echo "Button pressed"
        break  # Exit the loop after detecting button press
    fi

    sleep 0.1  # Introduce a small delay to reduce CPU usage
done

# Unexport GPIO pin
echo $BUTTON > /sys/class/gpio/unexport

echo "Script ended."