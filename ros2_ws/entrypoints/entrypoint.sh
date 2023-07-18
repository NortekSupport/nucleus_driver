#!/bin/bash

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the overlay workspace, if built
if [ -f /ros2_ws/install/setup.bash ]
then
  source /ros2_ws/install/setup.bash
fi
 
# Execute the command passed into this entrypoint
exec "$@"