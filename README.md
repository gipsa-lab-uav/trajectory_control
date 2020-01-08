# Trajectory Control

## Demonstration protocol
Connect 2 terminals to the drone (be sure to be connected to the right wifi network).

```bash
ssh -X uvify@192.168.1.160
```

First terminal is used to launch the depth cubemap node (be sure to export your display).

```bash
export DISPLAY:0
roslaunch draco_r depth_cubemap.launch
```

Second terminal is used to launch the px4 node offboard.

```bash
roslaunch draco_r px4_draco_r.launch
```

**Note:** if you want to used QGroundControl, edit the launch file and put your IP address in the right field.

Open a new termial. Set ROS_MASTER_URI with the drone IP address (don't forget the port) and set ROS_IP to yours.

```bash
export ROS_MASTER_URI=http://192.168.1.160:11311
export ROS_IP=192.168.1.244
```

Then on the same terminal launch trajectory_control_example.launch.

```bash
roslaunch trajectory_control trajectory_control_example.launch
```

In order for the drone to start the trajectory, arm the drone and enable the offboard mode with the remote controller.

**Warning:** in case of unexpected behavior, it is possible to disable the offboard mode with the switch and take back control with the remote controller, but sometimes the drone does not respond so always keep a finger on the kill switch as it will be the only way to stop the drone.

**********************************************************************************************************
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

Don't forget to install the required Python packages, with:
```bash
pip install -r requirements.txt
```

And then continue the installation:
```bash
cd ..
catkin_make
```

Then source your setup.bash

```bash
source devel/setup.bash
```

## Testing the installation
On one terminal, run:
```bash
roscore
```

And open a new terminal to run the test script:
```bash
roslaunch trajectory_control test.launch
```

A gazebo window should open with the iris model. After few seconds, the iris quadcopter should hover at 2 meters altitude. You can open QGroundControl in parallel also to check if everything is interfacing correctly.

Now you are ready to go with the trajectory_control_node. Another, more complex example can be accessed as follows:

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

### Gazebo

Run Gazebo with the verbose option to get more information on issues `gazebo --verbose`. If Gazebo process dies at the start, it might be a symbol lookup error. Upgrade all your package and try again.

```bash
sudo apt upgrade
```
