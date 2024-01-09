# AGC-Platform
AGC-Platform, named after Aerial-Ground Cooperation Platform, is aimed to provide an unified simulation environment in Gazebo for testing cooperative behavior between UAV and UGV. 
# UGV Platform
We adopt a mature work which integrates multiple sensors to a Scout2 robot including Realsense camera, Velodyne lidar and IMU, from https://github.com/linzs-online/robot_gazebo.
# UAV Platform
We directly use PX4 in Gazebo with seamless connection with real world experiement, concurrently taking some drone models in RotorS (https://github.com/ethz-asl/rotors_simulator) to fulfill more realistic dynamic simulation. You need to follow this instruction first to install PX4, which is restricted with certain version (release/1.12) to keep unified interface with upper-level algorithms.