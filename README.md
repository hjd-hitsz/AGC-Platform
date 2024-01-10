# AGC-Platform
**AGC-Platform**, named after **Aerial-Ground Cooperation Platform**, is aimed to provide an unified simulation environment in Gazebo for testing cooperative behavior between UAV and UGV. 
# UGV Platform
We adopt a demonstrative work which integrates multiple sensors to a **Scout2 robot** including Realsense camera, Velodyne lidar and IMU, from https://github.com/linzs-online/robot_gazebo.
# UAV Platform
We directly use **PX4** in Gazebo with seamless connection with real world experiement, concurrently taking some drone models in **RotorS** (https://github.com/ethz-asl/rotors_simulator) to fulfill more realistic dynamic simulation. You need to follow this [instruction](./px4_instruction.md) first to install PX4, which is restricted with certain version (release/1.12) to keep unified interface with upper-level algorithms.
# Simulated Environment
We widely collect different scenarios from robotics community and test with our simulated models, as listed below
- **CMU exploration**: https://www.cmu-exploration.com
- 
# Installation
## Platform only (git clone without --recursive)
1. System: Ubuntu-20.04 with ROS-Noetic (test passing)
2. [PX4 installation](./px4_instruction.md)
3. install dependances (please remind me if missing)
```bash
sudo apt update
sudo apt install ros-noetic-teleop-twist-keyboard
```
4. clone this repository
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/hjd-hitsz/AGC-Platform.git
```
5. build your workspace
```bash
cd ..
catkin build -j16 # replace with the numbers of your CPU threads
```
6. launch the AGC Platform
```bash
source devel/setup.bash
```
## Platform with Autonomous Navigation Algorithms
1. [**FAST-LIO**](https://github.com/hku-mars/FAST_LIO) for UAV/UGV lidar SLAM: we add FAST-LIO only as the submodule in this repository, so you need to follow instructions to install prerequisites of FAST-LIO, especially Livox driver.
2. **Octomap** for UAV/UGV Mapping
```bash
sudo apt install ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-msgs
# optional for octomap visualization
sudo apt install octovis
```
