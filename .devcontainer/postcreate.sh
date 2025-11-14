#!/bin/bash
# Source ROS environment automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc

mkdir -p src
sudo rosdep update
sudo rosdep install --ignore-src -y
sudo chown -R $(whoami) /home/ws/