#!/bin/bash

cd /home/lois/colcon
rm -r build
rm -r install
rm -r log
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
exit
