source /opt/ros/jazzy/setup.bash
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug 
cd build/waypoint_executive

./waypoint_executive_test