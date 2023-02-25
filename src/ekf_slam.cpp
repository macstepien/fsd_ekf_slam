#include <ekf_slam/ekf_slam.h>

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <limits>
#include <cmath>

#include <tf/tf.h>

#include <ekf_slam/util.h>

EkfSlam::EkfSlam(ros::NodeHandle& node) : tf_listener_(tf_buffer_)
{
  odometry_noise_.setZero();
  observation_noise_.setZero();

  int max_state_size = kRobotStateSize + kMaxNumObservations * kObservationSize;

  observation_jacobian_ = Eigen::MatrixXf::Zero(kObservationSize, max_state_size);

  means_ = Eigen::MatrixXf::Zero(max_state_size, 1);
  covariances_ = Eigen::MatrixXf::Zero(max_state_size, max_state_size);

  odom_position_.setZero();
  position_estimate_.setZero();

  setParameters(node);

  getLidarBaseLinkTransform();

  speed_from_odom_ = 0;
  last_odom_timestamp_ = ros::Time::now();

  observations_count_ = 0;

  visualization_ = true;

  cone_marker_pub_ = node.advertise<visualization_msgs::Marker>("/cone_detector/cone_marker", 50);
  position_pub_ = node.advertise<geometry_msgs::Pose2D>("/slam_pose", 50);

  //! Allow initial position from odometry - useful for testing
  odometryMeasurment();
  means_.block<kRobotStateSize, 1>(0, 0) = odom_position_;
}

void EkfSlam::processScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_DEBUG("Starting position update");

  Eigen::Vector3f odometry_measurement = odometryMeasurment();

  ROS_DEBUG_STREAM("Odometry obtained, position: " << odom_position_);

  std::vector<Observation> observations = cone_detector_.detectCones(scan);
  if (visualization_)
  {
    publishDetectedCones(observations);
  }

  ROS_DEBUG_STREAM("Detected: " << observations.size() << " cones");

  update(observations, odometry_measurement);

  publishPosition();
  if (visualization_)
  {
    publishConesMap();
  }
}

void EkfSlam::update(const std::vector<Observation>& observations, Eigen::Vector3f odometry_measurement)
{
  // Prediction

  // Mean prediction
  means_.block<kRobotStateSize, 1>(0, 0) += odometry_measurement;
  means_(2) = BoundToMinusPiPi(means_(2));

  // Covariance update
  Eigen::Matrix3f G = Eigen::Matrix3f::Identity();
  G(0, 2) -= odometry_measurement(1);
  G(1, 2) += odometry_measurement(0);

  covariances_.block<kRobotStateSize, kRobotStateSize>(0, 0) =
      G * covariances_.block<kRobotStateSize, kRobotStateSize>(0, 0) * G.transpose() + odometry_noise_;
  covariances_.block(0, kRobotStateSize, kRobotStateSize, observations_count_ * kObservationSize) =
      G * covariances_.block(0, kRobotStateSize, kRobotStateSize, observations_count_ * kObservationSize);

  for (auto obs : observations)
  {
    int matching_observation_index = -1;
    if (observations_count_ > 0)
    {
      matching_observation_index = findMatchingObservation(obs);
    }

    if (matching_observation_index < 0)
    {
      addNewObservation(obs);
    }
    else
    {
      updatePositionFromObservation(obs, matching_observation_index);
    }
  }

  position_estimate_ = means_.block<kRobotStateSize, 1>(0, 0);

  ROS_DEBUG_STREAM("Finished position update, new position: " << position_estimate_);
}

void EkfSlam::addNewObservation(const Observation& observation)
{
  int new_observation_index_x = kRobotStateSize + observations_count_ * kObservationSize;
  int new_observation_index_y = kRobotStateSize + observations_count_ * kObservationSize + 1;

  // Observation positions are stored in global coordinate frame (and first 3 values of means_
  // is current position of the robot).
  means_(new_observation_index_x, 0) = means_(0) + observation.r * cos(means_(2) + observation.angle);
  means_(new_observation_index_y, 0) = means_(1) + observation.r * sin(means_(2) + observation.angle);

  ROS_DEBUG_STREAM("Found new observation, coordinates x: " << means_(new_observation_index_x, 0)
                                                            << "y: " << means_(new_observation_index_y, 0));

  covariances_(new_observation_index_x, new_observation_index_x) = 1;
  covariances_(new_observation_index_y, new_observation_index_y) = 1;

  ++observations_count_;
}

void EkfSlam::updatePositionFromObservation(const Observation& cone, int matched_index)
{
  // Calculate Expected Measurment
  float x_expected = means_(kRobotStateSize + matched_index * kObservationSize) - means_(0);
  float y_expected = means_(kRobotStateSize + matched_index * kObservationSize + 1) - means_(1);

  float r_squared_expected = pow(x_expected, 2) + pow(y_expected, 2);
  float r_expected = sqrt(r_squared_expected);

  // If update distance is too small, then there are problems with division in observation_jacobian_robot_state_part and
  // observation_jacobian_observation_part matrices
  if (fabs(r_expected - cone.r) > 0.01)
  {
    float angle_expected = BoundToMinusPiPi(atan2(y_expected, x_expected) - means_(2));
    Eigen::Vector2f z_expected = Eigen::Vector2f(r_expected, angle_expected);
    Eigen::Vector2f z_real = Eigen::Vector2f(cone.r, cone.angle);

    // Observation Jacobian
    Eigen::MatrixXf observation_jacobian_robot_state_part = Eigen::MatrixXf::Zero(kObservationSize, kRobotStateSize);
    Eigen::MatrixXf observation_jacobian_observation_part = Eigen::MatrixXf::Zero(kObservationSize, kObservationSize);
    observation_jacobian_robot_state_part << -r_expected * x_expected / r_squared_expected,
        -r_expected * y_expected / r_squared_expected, 0, y_expected / r_squared_expected,
        -x_expected / r_squared_expected, -1;
    observation_jacobian_observation_part << r_expected * x_expected / r_squared_expected,
        r_expected * y_expected / r_squared_expected, -y_expected / r_squared_expected, x_expected / r_squared_expected;
    observation_jacobian_.block<kObservationSize, kRobotStateSize>(0, 0) = observation_jacobian_robot_state_part;
    observation_jacobian_.block<kObservationSize, kObservationSize>(
        0, kRobotStateSize + matched_index * kObservationSize) = observation_jacobian_observation_part;

    int current_size = kRobotStateSize + observations_count_ * kObservationSize;
    Eigen::MatrixXf covariances_block = covariances_.block(0, 0, current_size, current_size);
    Eigen::MatrixXf observation_jacobian_block = observation_jacobian_.block(0, 0, kObservationSize, current_size);
    Eigen::MatrixXf kalman_gain =
        covariances_block * observation_jacobian_block.transpose() *
        (observation_jacobian_block * covariances_block * observation_jacobian_block.transpose() + observation_noise_)
            .inverse();

    Eigen::Vector2f dz = z_real - z_expected;
    dz(1) = BoundToMinusPiPi(dz(1));

    means_.block(0, 0, current_size, 1) += kalman_gain * (dz);
    covariances_.block(0, 0, current_size, current_size) =
        (Eigen::MatrixXf::Identity(current_size, current_size) - kalman_gain * observation_jacobian_block) *
        covariances_block;

    observation_jacobian_.block<kObservationSize, kRobotStateSize>(0, 0) =
        Eigen::MatrixXf::Zero(kObservationSize, kRobotStateSize);
    observation_jacobian_.block<kObservationSize, kObservationSize>(0, kRobotStateSize +
                                                                           kObservationSize * matched_index) =
        Eigen::MatrixXf::Zero(kObservationSize, kObservationSize);
  }
}

int EkfSlam::findMatchingObservation(const Observation& position)
{
  float x_mean, y_mean, r_mean, angle_mean, distance;
  float x_detected = position.r * cos(position.angle);
  float y_detected = position.r * sin(position.angle);
  for (int index = 0; index < observations_count_; ++index)
  {
    x_mean = means_(kRobotStateSize + index * kObservationSize) - means_(0);
    y_mean = means_(kRobotStateSize + index * kObservationSize + 1) - means_(1);

    r_mean = sqrt(pow(x_mean, 2) + pow(y_mean, 2));
    angle_mean = BoundToMinusPiPi(atan2(y_mean, x_mean) - means_(2));

    x_mean = r_mean * cos(angle_mean);
    y_mean = r_mean * sin(angle_mean);

    distance = sqrt(pow(x_detected - x_mean, 2) + pow(y_detected - y_mean, 2));

    if (speed_from_odom_ < 0 || speed_from_odom_ > 10 || std::isnan(speed_from_odom_))
    {
      speed_from_odom_ = 0;
    }

    float observation_matchin_threshold = observation_matching_max_distance_ *
                                          (speed_from_odom_ > scaling_speed_ ? speed_from_odom_ / scaling_speed_ : 1);
    if (distance < observation_matchin_threshold)
    {
      return index;
    }
  }
  // Matching cone not found
  // TODO: change to exception
  return -1;
}

Eigen::Vector3f EkfSlam::odometryMeasurment()
{
  geometry_msgs::TransformStamped odom_base_footprint_transform;

  try
  {
    // Time(0) - latest available transform
    // Time::now() - current time
    // TODO: change to parameters
    odom_base_footprint_transform =
        tf_buffer_.lookupTransform("base_footprint", "odom", ros::Time(0), ros::Duration(10.0));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  Eigen::Quaternionf odom_base_footprint_quaternion(
      odom_base_footprint_transform.transform.rotation.w, odom_base_footprint_transform.transform.rotation.x,
      odom_base_footprint_transform.transform.rotation.y, odom_base_footprint_transform.transform.rotation.z);

  float odom_base_footprint_yaw = odom_base_footprint_quaternion.toRotationMatrix().eulerAngles(0, 1, 2)(2);

  Eigen::Vector3f odom_position_change;
  odom_position_change(0) = (odom_position_(0) - odom_base_footprint_transform.transform.translation.x);
  odom_position_change(1) = (odom_position_(1) - odom_base_footprint_transform.transform.translation.y);
  odom_position_change(2) = BoundToMinusPiPi(odom_position_(2) - odom_base_footprint_yaw);

  odom_position_(0) = odom_base_footprint_transform.transform.translation.x;
  odom_position_(1) = odom_base_footprint_transform.transform.translation.y;
  odom_position_(2) = BoundToMinusPiPi(odom_base_footprint_yaw);

  float time_diff = (odom_base_footprint_transform.header.stamp - last_odom_timestamp_).toSec();
  speed_from_odom_ = sqrt(pow(odom_position_change(0), 2) + pow(odom_position_change(1), 2)) / time_diff;
  last_odom_timestamp_ = odom_base_footprint_transform.header.stamp;

  ROS_DEBUG_STREAM("Speed from odom: " << speed_from_odom_);

  return odom_position_change;
}

void EkfSlam::publishPosition()
{
  geometry_msgs::TransformStamped map_base_link_transform;
  map_base_link_transform.header.stamp = ros::Time::now();
  map_base_link_transform.header.frame_id = "map";
  map_base_link_transform.child_frame_id = "base_link";
  map_base_link_transform.transform.translation.x = position_estimate_(0);
  map_base_link_transform.transform.translation.y = position_estimate_(1);
  map_base_link_transform.transform.translation.z = 0.0;

  map_base_link_transform.transform.rotation.w = cos(position_estimate_(2) * 0.5);
  map_base_link_transform.transform.rotation.x = 0;
  map_base_link_transform.transform.rotation.y = 0;
  map_base_link_transform.transform.rotation.z = sin(position_estimate_(2) * 0.5);

  tf_broadcaster_.sendTransform(map_base_link_transform);

  geometry_msgs::Pose2D robot_pose;
  robot_pose.x = position_estimate_(0);
  robot_pose.y = position_estimate_(1);
  robot_pose.theta = position_estimate_(2);

  position_pub_.publish(robot_pose);
}

void EkfSlam::publishDetectedCones(const std::vector<Observation>& cones)
{
  int i = 0;
  for (auto x : cones)
  {
    geometry_msgs::Point point;
    point.x = x.r * cos(x.angle);
    point.y = x.r * sin(x.angle);
    point.z = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "detected_cones";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(1);
    cone_marker_pub_.publish(marker);
    ++i;
  }
}

void EkfSlam::publishConesMap()
{
  for (int i = 0; i < observations_count_; ++i)
  {
    float x = means_(kRobotStateSize + i * kObservationSize);
    float y = means_(kRobotStateSize + i * kObservationSize + 1);

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cones";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.75;
    marker.color.g = 0.75;
    marker.color.b = 0.75;
    marker.lifetime = ros::Duration(1);
    cone_marker_pub_.publish(marker);
  }
}

void EkfSlam::getLidarBaseLinkTransform()
{
  geometry_msgs::TransformStamped lidar_base_link_transform;
  bool is_transform_valid = false;
  while (!is_transform_valid)
  {
    is_transform_valid = true;
    try
    {
      lidar_base_link_transform =
          tf_buffer_.lookupTransform("base_link", "velodyne", ros::Time(0), ros::Duration(10.0));
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Couldn't get lidar position: %s", ex.what());
      ros::Duration(1.0).sleep();
      is_transform_valid = false;
    }
  }

  ROS_DEBUG("velodyne->base_link transform obtained");

  Eigen::Quaternionf lidar_to_base_link_quaternion = Eigen::Quaternionf(
      lidar_base_link_transform.transform.rotation.w, lidar_base_link_transform.transform.rotation.x,
      lidar_base_link_transform.transform.rotation.y, lidar_base_link_transform.transform.rotation.z);

  Eigen::Vector3f lidar_to_base_link_translation;
  lidar_to_base_link_translation(0) = lidar_base_link_transform.transform.translation.x;
  lidar_to_base_link_translation(1) = lidar_base_link_transform.transform.translation.y;
  lidar_to_base_link_translation(2) = 0;

  cone_detector_.setLidarBaseLinkTransformation(lidar_to_base_link_translation, lidar_to_base_link_quaternion);
}

void EkfSlam::setParameters(ros::NodeHandle& node)
{
  if (!node.getParam("odometry_noise_x", odometry_noise_(0, 0)))
  {
    ROS_WARN("Couldn't get odometry_noise_x parameter, setting to default 0.05 [m]");
    odometry_noise_(0, 0) = 0.05;
  }

  if (!node.getParam("odometry_noise_y", odometry_noise_(1, 1)))
  {
    ROS_WARN("Couldn't get odometry_noise_y parameter, setting to default 0.05 [m]");
    odometry_noise_(1, 1) = 0.05;
  }
  float odometry_noise_theta_degrees;
  if (!node.getParam("odometry_noise_theta", odometry_noise_theta_degrees))
  {
    ROS_WARN("Couldn't get odometry_noise_theta parameter, setting to default 5 [degrees]");
    odometry_noise_(2, 2) = (5. * M_PI) / 180.;
  }
  else
  {
    odometry_noise_(2, 2) = (odometry_noise_theta_degrees * M_PI) / 180.;
  }

  ROS_DEBUG_STREAM("Odometry noise: " << odometry_noise_);

  if (!node.getParam("observation_noise_x", observation_noise_(0, 0)))
  {
    ROS_WARN("Couldn't get observation_noise_x parameter, setting to default 0.05 [m]");
    observation_noise_(0, 0) = 0.05;
  }
  if (!node.getParam("observation_noise_y", observation_noise_(1, 1)))
  {
    ROS_WARN("Couldn't get observation_noise_y parameter, setting to default 0.05 [m]");
    observation_noise_(1, 1) = 0.05;
  }

  ROS_DEBUG_STREAM("Observation noise: " << observation_noise_);

  if (!node.getParam("cone_matching_max_distance", observation_matching_max_distance_))
  {
    ROS_WARN("Couldn't get cone_matching_max_distance parameter, setting to default 0.5 [m]");
    observation_matching_max_distance_ = 0.5;
  }

  float min_lidar_range;
  if (!node.getParam("min_lidar_range", min_lidar_range))
  {
    ROS_WARN("Couldn't get min_lidar_range parameter, leaving default");
  }
  else
  {
    cone_detector_.setMinLidarRange(min_lidar_range);
  }

  float max_lidar_range;
  if (!node.getParam("max_lidar_range", max_lidar_range))
  {
    ROS_WARN("Couldn't get max_lidar_range parameter, leaving default");
  }
  else
  {
    cone_detector_.setMaxLidarRange(max_lidar_range);
  }

  int min_cluster_size;
  if (!node.getParam("min_cluster_size", min_cluster_size))
  {
    ROS_WARN("Couldn't get min_cluster_size parameter, leaving default");
  }
  else
  {
    cone_detector_.setMinClusterSize(min_cluster_size);
  }

  int max_cluster_size;
  if (!node.getParam("max_cluster_size", max_cluster_size))
  {
    ROS_WARN("Couldn't get max_cluster_size parameter, leaving default");
  }
  else
  {
    cone_detector_.setMaxClusterSize(max_cluster_size);
  }

  float cluster_tolerance_distance;
  if (!node.getParam("cluster_tolerance_distance", cluster_tolerance_distance))
  {
    ROS_WARN("Couldn't get cluster_tolerance_distance parameter, leaving default");
  }
  else
  {
    cone_detector_.setClusterToleranceDistance(cluster_tolerance_distance);
  }
}