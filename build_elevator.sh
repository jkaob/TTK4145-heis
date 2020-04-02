#!/bin/bash
echo "Building Elevator Lab!"
echo ""
cd dev_ws/
source /opt/ros/eloquent/setup.bash
colcon build
cd ..
