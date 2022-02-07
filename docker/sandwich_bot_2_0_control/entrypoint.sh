#!/bin/sh

# setup ros2 environment
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
. /opt/ros/${ROS_DISTRO}/setup.sh
. $ROS2_WS/install/setup.sh

exec "$@"
