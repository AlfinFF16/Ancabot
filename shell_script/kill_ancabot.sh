#!/bin/bash

# Running Node Index
NAVIGATION_MODE=1

# Search for the process PID
PID=$(pgrep -f "roslaunch hexapod_bringup navigation_$NAVIGATION_MODE.launch")

# Check if PID is found
if [ -z "$PID" ]; then
    echo "Process not found."
else
    echo "Found process with PID: $PID"
    # Kill the process
    kill $PID
    echo "Process with PID $PID killed."
fi
