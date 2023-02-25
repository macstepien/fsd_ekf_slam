#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include "cone_detector.h"

class EkfSlam
{
public:
  EkfSlam(ros::NodeHandle& node);
  void processScan(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
  static const int kMaxNumObservations = 1000;
  static const int kObservationSize = 2;
  static const int kRobotStateSize = 3;

  // TODO: parameter
  float scaling_speed_ = 0.8;

  Eigen::Matrix3f odometry_noise_;
  Eigen::Matrix2f observation_noise_;
  Eigen::MatrixXf observation_jacobian_;

  Eigen::MatrixXf means_;
  Eigen::MatrixXf covariances_;

  Eigen::Vector3f odom_position_;
  Eigen::Vector3f position_estimate_;

  ConeDetector cone_detector_;

  float observation_matching_max_distance_;
  float speed_from_odom_;
  ros::Time last_odom_timestamp_;

  int observations_count_;

  bool visualization_;

  ros::Publisher cone_marker_pub_;
  ros::Publisher position_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void setParameters(ros::NodeHandle& node);
  void getLidarBaseLinkTransform();

  void update(const std::vector<Observation>& observations, Eigen::Vector3f odometry_measurement);

  Eigen::Vector3f odometryMeasurment();
  int findMatchingObservation(const Observation& position);
  void addNewObservation(const Observation& cone);
  void updatePositionFromObservation(const Observation& cone, int matchedIndex);

  // TF and /slam_pose publishing
  void publishPosition();

  // Visualization
  void publishConesMap();
  void publishDetectedCones(const std::vector<Observation>& cones);
};

#endif