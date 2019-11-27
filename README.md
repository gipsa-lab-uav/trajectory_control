# Trajectory Control
Trajectory generation and control algorithms for UAVs with ROS wrapping.

The project uses external software such as Mavros. Below are the links directing to their documentations.

[PX4 Development Guide](https://dev.px4.io/v1.9.0/en/)

[PX4-Firmware](https://github.com/PX4/Firmware)

[Mavros](https://github.com/mavlink/mavros/)

[Sitl-Gazebo](https://github.com/PX4/sitl_gazebo)

## Installation
For the installation, you need to have ROS melodic (or kinetic) installed, a catkin workspace and Gazebo. Follow the online documentation to set up your environement.

[ROS installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

[catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

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

### Install trajectory-control
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

Gazebo will open, along with a window with a trajectory plot. After this window is closed, the iris drone should start following the trajectory.

## HR Drone Model and Plugin
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
gazebo_magnus_plugin
```

And to finish and use the new files:
```bash
cd ..
catkin_make
source devel/setup.bash
```

### SDF Model
To apply changes to SDF models (as those in the various directories in `~/catkin_ws/src/sitl_gazebo/models`), it is advisable to create a new model, with its own proper name and settings. A new model must follow the same structure as the previous ones so, for example, if we wanted to create a model named `new_model`, we would create a directory in `~/catkin_ws/src/sitl_gazebo/models`, named `new_model`, and it would be like:

```
.
├── ...
├── new_model
│   ├── new_model.sdf
│   └── model.config
└── ...
```
If it happens that a lot of tags have repeated values, or values that are calculated based on other values also in the SDF model file, it can be used [embedded ruby](https://en.wikipedia.org/wiki/ERuby) to generate parametrized models. If this is the case, a new file would be created. Following the example, we could name it `new_model.rsdf`, and it would include embedded ruby code. A nice example of this feature can be found in [this file](https://bitbucket.org/osrf/gazebo_models/src/b237ea45262f51ff8d72aaa96ef19f1288723c42/cart_rigid_suspension/model.rsdf). To then transform a RSDF file to the desired SDF model file, run the following command:

```bash
erb new_model.rsdf > new_model.sdf
```

Details and more options for the command `erb` can be found [here](https://www.commandlinux.com/man-page/man1/erb.1.html).


## Troubleshooting

### catkin_make
If performance issues don't allow for the computer to finish `catkin_make`, it is possible to reduce the compiler usage on the computer, using the `-j` option. Use the `nproc` command to get the number of CPU cores/threads available. Example:

```bash
nproc
>> 8
catkin_make -j4
```

If you run into the message `Configuring incomplete, errors occurred!`, and `Invoking "make cmake_check_build_system" failed`, during `catkin_make`, an extra step might be necessary before continuing:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths ~/catkin_ws/src/ --ignore-src
```

After updating rosdep, redo the last steps on the compiling process:
```bash
catkin_make
source devel/setup.bash
```

### rosdep
If an error like `cannot download default sources list from: ... Website may be down` occurs, run:

```bash
sudo apt-get install ca-certificates
```

If it presists, check the system date/clock, and update it if incorrect.


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

### Geographic Lib
If the test file doesn't launch a proper Gazebo environment, with a related error such as `[FATAL]: UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`, some additional steps are required. First, check whether or not the directories `geoids`, `gravity` and `magnetic` already exist on the path `/usr/local/share/GeographicLib/`. If they don't, follow the next steps, otherwise, skip the following commands:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

And then, in any case, copy the desired contents from `/usr/local/share/` to `/usr/share/`, running the following command:

```bash
sudo cp -a /usr/local/share/GeographicLib/* /usr/share/GeographicLib/
```

### Gazebo
If Gazebo doesn't start properly, run:
```bash
gazebo --verbose
```

#### Shader Lib
To solve `[Err] [RTShaderSystem.cc:450] Unable to find shader lib.` issue, run (replacing the 'X' for the number of your Gazebo version):
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-X
```

#### REST Request
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

**Note:** To make permanent changes to path variables, add the command to the `~/.bashrc` file, as example:
```bash
nano ~/.bashrc
```
And add lines such as:
```bash
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/sitl_gazebo/models
```