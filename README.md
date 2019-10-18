# trajectory-control
Trajectory generation and control algorithms for UAVs with ROS wrapping.

## Installation
### Prerequisite
sudo apt install ros-kinetic-mavros
sudo apt install libgstreamer1.0-dev

In your catkin_ws:
cd catkin_ws/src/
git clone --recursive https://github.com/PX4/sitl_gazebo
git clone https://github.com/PX4/Firmware px4

## Install trajectory-control
In your catkin_ws:
cd catkin_ws/src/
git clone https://github.com/gipsa-lab-uav/trajectory-control
catkin_make
source devel/setup.bash
