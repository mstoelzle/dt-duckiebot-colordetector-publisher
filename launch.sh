#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roscore &
sleep 5
roslaunch colordetector_publisher colordetector_publisher.launch vehicle_name:=$VEHICLE_NAME
