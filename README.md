# Trajectory Control
Trajectory generation and control algorithms for UAVs with ROS wrapping.

The project uses external software such as Mavros. Below are the links directing to their documentations.

[PX4 Development Guide](https://dev.px4.io/v1.9.0/en/)

[PX4-Firmware](https://github.com/PX4/Firmware)

[Mavros](https://github.com/mavlink/mavros/)

[Sitl-Gazebo](https://github.com/PX4/sitl_gazebo)

## Installation
For the installation, you need to have ROS melodic (or kinetic) installed and a catkin workspace. Follow the online documentation to set up your environement.

[ROS installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Prerequisites
Install mavros and libgstreamer

```bash
sudo apt install ros-melodic-mavros libgstreamer1.0-dev
```

Clone sitl_gazebo and PX4 Firmware

```bash
cd ~/catkin_ws/src/
git clone --recursive https://github.com/PX4/sitl_gazebo
git clone https://github.com/PX4/Firmware px4
```

**Note:** If you have troubles intalling the different packages, it is strongly recommended to read the related documentation.

### Install trajectory-control
```bash
cd ~/catkin_ws/src/
git clone https://github.com/gipsa-lab-uav/trajectory-control
cd ..
catkin_make
```

**Note:** If you run into the message `Configuring incomplete, errors occurred!`, and `Invoking "make cmake_check_build_system" failed`, an extra step might be necessary before continuing:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths ~/catkin_ws/src/ --ignore-src
```

```bash
catkin_make
source devel/setup.bash
```

## Testing the installation
```bash
roslaunch trajectory-control test.launch
```

A gazebo window should open with the iris model. After few seconds, the iris quadcopter should hover at 2 meters altitude. You can open QGroundControl in parallel also to check if everything is interfacing correclty.

### Install QGroundControl
Refer to [Official Documentation](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html) if any problems.

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav -y
```
Logout and login again to enable changes, then download the [AppImage for QGroundControl](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage) and run:

```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

Now you are ready to go with the trajectory-control_node:

```bash
roslaunch trajectory-control trajectory-control-example.launch
```

### Gazebo
If Gazebo doesn't start properly, run:
```bash
gazebo --verbose
```

To solve `[Err] [RTShaderSystem.cc:450] Unable to find shader lib.` issue, run (replacing the 'X' for the number of your Gazebo version):
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-X
```

If Gazebo runs, but with the `--verbose` option you get an error as `[Err] [REST.cc:205] Error in REST request`, check your Gazebo version. If you can update to Gazebo >= 9.10, it should solve the issue, else, update the `server url` field on Fuel config file with:

```bash
nano ~/.ignition/fuel/config.yaml
```
From `https://api.ignitionfuel.org/` to `https://api.ignitionrobotics.org/`.\
More info on the topic can be found [here](http://answers.gazebosim.org/question/22263/error-in-rest-request-for-accessing-apiignitionorg/).

If some of the desired models are not automatically available on the Gazebo interface, add the path to the modules to the variable `GAZEBO_MODEL_PATH`, as:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:path/to/models
```

**Note:** To make changes to path variables permanent, add the command to the `~/.bashrc` file, as example:
```bash
nano ~/.bashrc
```
And add a line as:
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-9
```

### HR Drone Model and Plugin
The model of the HR Drone is based on the Iris model, available in `sitl_gazebo`. Both the model and the plugin are available at the repository, but some adjustments must be made to ensure functionality.

The next commands assume that the repositories `sitl_gazebo` and `trajectory-control` were cloned into `~/catkin/src/`, if that is not the case, change the paths accordingly.

First, copy the wing model header and the plugin header to the header directory from `sitl_gazebo`:

```bash
cd ~/catkin_ws/src/
cp -a trajectory-control/include/magnus_plugin/ sitl_gazebo/include/magnus_plugin
cp -a trajectory-control/include/gazebo_magnus_wing_model.h sitl_gazebo/include/gazebo_magnus_wing_model.h
```

Then, copy the HR Drone model:
```bash
cp -a trajectory-control/models/iris_magnus/ sitl_gazebo/models/iris_magnus
```

And then the source files:
```bash
cp -a trajectory-control/src/magnus_plugin/ sitl_gazebo/src/magnus_plugin
cp -a trajectory-control/src/gazebo_magnus_wing_model.cpp sitl_gazebo/src/gazebo_magnus_wing_model.cpp
```

After that, some changes must be made at the `~/catkin_ws/src/sitl_gazebo/CMakeLists.txt` file. Add the following line at the `# Plugins #` section:
```makefile
add_library(gazebo_magnus_plugin SHARED src/magnus_plugin/magnus_plugin.cpp)
```

And inside the `set(plugins ... ...)` command add a line:
```makefile
magnus_plugin
```

And to finish and use the new files:
```bash
cd ..
catkin_make
source devel/setup.bash
```