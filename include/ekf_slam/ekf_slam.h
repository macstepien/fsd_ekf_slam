#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <ekf_slam/cone_detector.h>
#include <ekf_slam/ekf.h>
#include <ekf_slam/util.h>

#include <memory>

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
  int findMatchingObservation(const Observation& position);

  // TF and /slam_pose publishing
  void publishPosition();

  // Visualization
  void publishConesMap();
  void publishDetectedCones(const std::vector<Observation>& cones);
};

#endif