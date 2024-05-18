#!/bin/bash

# Search for the process PID
PID=$(pgrep -f "roslaunch hexapod_bringup ancabot.launch")

# Check if PID is found
if [ -z "$PID" ]; then
    echo "Process not found."
else
    echo "Found process with PID: $PID"
    # Kill the process
    kill $PID
    echo "Process with PID $PID killed."
fi
