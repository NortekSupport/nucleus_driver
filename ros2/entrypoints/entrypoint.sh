#!/bin/bash

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the overlay workspace, if built
source /ros2/install/setup.bash
 
# Execute the command passed into this entrypoint
exec "$@"