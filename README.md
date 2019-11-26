# Trajectory Control
Trajectory generation and control algorithms for UAVs with ROS wrapping.

The project uses external software such as Mavros. Below are the links directing to their documentations.

[PX4 Development Guide](https://dev.px4.io/v1.9.0/en/)

[PX4-Firmware](https://github.com/PX4/Firmware)

[Mavros](https://github.com/mavlink/mavros/)

[Sitl-Gazebo](https://github.com/PX4/sitl_gazebo)

## Installation
For the installation, you need to have ROS melodic (or kinetic) installed, a catkin workspace and Gazebo. Follow the online documentation to set up your environement.

[ROS Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

[Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

### Prerequisites
Install mavros

```bash
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```

Mavros request the GeographicLib datasets, install it by running the install_geographiclib_datasets.sh script

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```
Install libgstreamer

```bash
sudo apt install libgstreamer1.0-dev
```

Initialize rosdep and update it

```bash
sudo rosdep init
rosdep update
```

Clone sitl_gazebo and PX4 Firmware

```bash
cd ~/catkin_ws/src/
git clone --recursive https://github.com/PX4/sitl_gazebo
git clone --recursive https://github.com/PX4/Firmware px4
```

**Note:** If you have troubles installing the different packages, it is recommended to read the related documentation.

### Install trajectory_control
Clone the trajectory_control repository
```bash
cd ~/catkin_ws/src/
git clone https://github.com/gipsa-lab-uav/trajectory_control
```

Install the requested Python2 libraries

```bash
pip install -r requirements.txt
```

Run catkin_make
```bash
cd ..
catkin_make
```

Then source your setup.bash

```bash
source devel/setup.bash
```

## Testing the installation
```bash
roslaunch trajectory_control test.launch
```

A gazebo window should open with the iris model. After few seconds, the iris quadcopter should hover at 2 meters altitude. You can open QGroundControl in parallel also to check if everything is interfacing correclty.

Now you are ready to go with the trajectory_control_node.

```bash
roslaunch trajectory_control trajectory_control_example.launch
```

## Troubleshooting

### catkin_make

In case of issue with catkin_make it is advisable to launch it with the ```VERBOSE=1``` option

If performance issues don't allow for the computer to finish `catkin_make`, it is possible to reduce the compiler usage on the computer, using the `-j` option. Use the `nproc` command to get the number of CPU cores/threads available. Example:

```bash
nproc
>> 8
catkin_make -j2
```

### rosdep

If an error of type 'cannot download default sources list from: ... Website may be down' occurs

```bash
sudo apt-get install ca-certificates
```

If it still does not work, try to update your system date/clock

### gazebo

If gazebo process dies at the start, it might be a symbol lookup error. Upgrade all your package and try again

```bash
sudo apt upgrade
```
