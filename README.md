# trajectory-control
Trajectory generation and control algorithms for UAVs with ROS wrapping.

The project uses external software such as Mavros. Below are the links directing to their documentations.

[PX4 Development Guide](https://dev.px4.io/v1.9.0/en/)

[PX4-Firmware](https://github.com/PX4/Firmware)

[Mavros](https://github.com/mavlink/mavros/)

[Sitl-Gazebo](https://github.com/PX4/sitl_gazebo)

## Installation
### Prerequisites
Install mavros and libgstreamer

```sudo apt install ros-kinetic-mavros```

```sudo apt install libgstreamer1.0-dev```

Clone sitl_gazebo and PX4 Firmware

```cd ~/catkin_ws/src/```

```git clone --recursive https://github.com/PX4/sitl_gazebo```

```git clone https://github.com/PX4/Firmware px4```

**Note:** If you have troubles intalling the different packages, it is strongly recommended to read the related documentation.

### Install trajectory-control
```cd ~/catkin_ws/src/```

```git clone https://github.com/gipsa-lab-uav/trajectory-control```

```cd ../../```

```catkin_make```

```source devel/setup.bash```

## Testing the installation
```roslaunch trajectory-control test.launch```

A gazebo window should open with the iris model. After few seconds, the iris quadcopter should hover at 2 meters altitude. You can open QGroundControl in parallel also to check if everything is interfacing correclty.
