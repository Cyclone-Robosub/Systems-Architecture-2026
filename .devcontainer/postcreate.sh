#!/bin/bash
sudo rosdep fix-permissions
rosdep update
sudo rosdep install --from-paths /home/rosdev --ignore-src -y