// Copyright 2023 Maciej Stępień

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <memory>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fsd_ekf_slam/cone_detector.h>
#include <fsd_ekf_slam/ekf.h>
#include <fsd_ekf_slam/util.h>

class EkfSlam
{
public:
  EkfSlam(ros::NodeHandle& node);
  void processScan(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
  // TODO: parameter
  float scaling_speed_ = 0.8;

  Eigen::Vector3f odometry_noise_;
  Eigen::Vector2f observation_noise_;

  Eigen::Vector3f last_odom_position_;
  ros::Time last_odom_timestamp_;

  ConeDetector cone_detector_;
  std::unique_ptr<Ekf> ekf_;

  float observation_matching_max_distance_;
  float speed_from_odom_;

  bool visualization_;

  ros::Publisher cone_marker_pub_;
  ros::Publisher position_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void parseParameters(ros::NodeHandle& node);
  void getLidarBaseLinkTransform();

  Eigen::Vector3f odometryMeasurment();

  // TF and /slam_pose publishing
  void publishPosition();

  // Visualization
  void publishConesMap();
  void publishDetectedCones(const std::vector<Observation>& cones);
};

#endif