#!/bin/bash

export DISPLAY=:1.0
export ROS_HOSTNAME=sim
export ROS_MASTER_URI=http://surface:11311

source /root/.bashrc
. /root/catkin_ws/devel/setup.bash
exec "$@"