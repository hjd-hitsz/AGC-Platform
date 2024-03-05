# Overview
We choose the `release/1.12` version of PX4 to address various issues related to package adaptation, as significant differences exist between different PX4 versions. Please follow instructions below to install PX4.
# Installation
## QGroundControl
follow this [instruction](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) to install QGC ground station
## PX4
download source code of PX4
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
```
switch to branch `release/1.12`
```bash
cd PX4-Autopilot
git checkout release/1.12
```
download submodule code
```bash
git submodule update --init --recursive
```
execute script to install environment
```bash
bash ./Tools/setup/ubuntu.sh
```
create a new script for quick setting of environment variables
```bash
touch px4origin.bash
gedit px4origin.bash
```
add commands below to this script and replace with your PX4 path
```bash
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_MODEL_DATABASE_URI=""
```
source this script (source again when openning a new terminal)
```bash
source px4origin.bash
```
compile sitl_gazebo package
```bash
make px4_sitl gazebo
```
## MAVROS
follow this [instruction](https://docs.px4.io/v1.12/en/ros/mavros_installation.html) to install MAVROS by binary mode