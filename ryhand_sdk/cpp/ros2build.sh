#!/bin/bash
workspace=$(pwd)
colcon build "$@"
source install/setup.bash
sudo cp ${workspace}/install/rh15_ctrl/lib/*.so /opt/ros/foxy/lib