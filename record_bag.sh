#!/bin/bash

# Create recording folder if it does not exist (inside offboard_control pkg
mkdir -p recordings

# Start with timedate
dt=$(date '+%d%m%Y_%H%M%S');

# Dump all params
rosparam dump ${dt}_landing_trials_params_dump.yaml

# Get some of the landing params
customuwb=$(rosparam get /uav/UWB_VIO_Landing/using_custom_uwb);
trials=$(rosparam get /uav/UWB_VIO_Landing/max_landing_trials);

# Record excluding the most high freq topics
rosbag record -a -x "/uav/mavlink/(.*)|/uav/t265/(.*)|/uav/mavros/imu/(.*)|/tf(.*)" -O "recordings/${dt}_landing_${trials}_trials_custom_${customuwb}.bag"
