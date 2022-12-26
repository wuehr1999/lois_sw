#!/bin/bash

cd colcon
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
cd ..
