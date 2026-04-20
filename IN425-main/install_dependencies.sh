#!/bin/bash

sudo apt update -y

# Install project's dependencies 
sudo apt -y install python3-colcon-common-extensions python3-pip
sudo apt -y install ros-galactic-gazebo-ros-pkgs ros-galactic-tf-transformations ros-galactic-tf2-tools ros-galactic-teleop-twist-keyboard ros-galactic-xacro ros-galactic-slam-toolbox

pip3 install transforms3d opencv-contrib-python==4.7.0.72

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc