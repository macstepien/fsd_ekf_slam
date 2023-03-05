# EKF SLAM algorithm for Formula Student Driverless car

<p align="center">
  <img src=".github/ekf_slam_demo.gif" height="400" />
</p>

Implementation of EKF SLAM algorithm for Formula Student Driverless car (in C++, using Eigen for matrix operations). Also includes a simple cone detector, which processes Lidar data with PCL and calculates the position of a cone. Detected cones are later used as observations for EKF SLAM. The whole thing was implemented as a ROS Node (tested on ROS Melodic, at least C++14 is required). Running:
```
roslaunch ekf_slam ekf_slam.launch
```
Parameters can be adjusted (for details check config file `ekf_slam_config.yaml`).