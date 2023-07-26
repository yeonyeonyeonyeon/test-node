#!/bin/bash

# Add the path of your ROS workspace
source /root/catkin_ws/devel/setup.bash

# Set ROS environment variables
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# Execute the roslaunch command
roslaunch test_node test.launch
