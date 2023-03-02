#include <ekf_slam/ekf_slam.h>

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <limits>
#include <memory>
#include <cmath>

#include <tf/tf.h>

#include <ekf_slam/util.h>

EkfSlam::EkfSlam(ros::NodeHandle& node) : tf_listener_(tf_buffer_)
{
  last_odom_position_.setZero();

  parseParameters(node);

  getLidarBaseLinkTransform();

  speed_from_odom_ = 0;
  last_odom_timestamp_ = ros::Time::now();

  visualization_ = true;

  cone_marker_pub_ = node.advertise<visualization_msgs::Marker>("/cone_detector/cone_marker", 50);
  position_pub_ = node.advertise<geometry_msgs::Pose2D>("/slam_pose", 50);

  //! Allow initial position from odometry - useful for testing
  odometryMeasurment();
  ekf_ = std::make_unique<Ekf>(last_odom_position_, odometry_noise_, observation_noise_);
}

void EkfSlam::processScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_DEBUG("Starting position update");
  Eigen::Vector3f odometry_measurement = odometryMeasurment();
  ROS_DEBUG_STREAM("Odometry obtained, position: " << last_odom_position_);
  ROS_DEBUG_STREAM("Odometry change: " << odometry_measurement);

  ekf_->predict(odometry_measurement);

  std::vector<Observation> observations = cone_detector_.detectCones(scan);
  if (visualization_)
  {
    publishDetectedCones(observations);
  }
  ROS_DEBUG_STREAM("Detected: " << observations.size() << " cones");

  // std::vector<std::pair<Observation, int>> observation_pairs;
  for (auto obs : observations)
  {
    try
    {
      std::pair<int, float> matching_observation = ekf_->findClosestObservation(obs);

      float observation_matchin_threshold = observation_matching_max_distance_ *
                                            (speed_from_odom_ > scaling_speed_ ? speed_from_odom_ / scaling_speed_ : 1);
      if (matching_observation.second < observation_matchin_threshold)
      {
        ROS_DEBUG_STREAM("Found matching cones!");
        // observation_pairs.push_back(std::make_pair(obs, matching_observation.first));
        ekf_->updatePositionFromObservation(obs, matching_observation.first);
      }
      else
      {
        ekf_->addNewObservation(obs);
      }
    }
    catch (const std::runtime_error& e)
    {
      ekf_->addNewObservation(obs);
    }
  }

  // ekf_->update(observation_pairs);

  publishPosition();
  if (visualization_)
  {
    publishConesMap();
  }
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
        tf_buffer_.lookupTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));
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
  odom_position_change(0) = (odom_base_footprint_transform.transform.translation.x - last_odom_position_(0));
  odom_position_change(1) = (odom_base_footprint_transform.transform.translation.y - last_odom_position_(1));
  odom_position_change(2) = BoundToMinusPiPi(odom_base_footprint_yaw - last_odom_position_(2));

  last_odom_position_(0) = odom_base_footprint_transform.transform.translation.x;
  last_odom_position_(1) = odom_base_footprint_transform.transform.translation.y;
  last_odom_position_(2) = BoundToMinusPiPi(odom_base_footprint_yaw);

  float time_diff = (odom_base_footprint_transform.header.stamp - last_odom_timestamp_).toSec();
  speed_from_odom_ = sqrt(pow(odom_position_change(0), 2) + pow(odom_position_change(1), 2)) / time_diff;
  last_odom_timestamp_ = odom_base_footprint_transform.header.stamp;

  ROS_DEBUG_STREAM("Speed from odom: " << speed_from_odom_);
  if (speed_from_odom_ < 0 || speed_from_odom_ > 10 || std::isnan(speed_from_odom_))
  {
    speed_from_odom_ = 0;
  }

  return odom_position_change;
}

void EkfSlam::publishPosition()
{
  Eigen::Vector3f position_estimate = ekf_->getCurrentPositionEstimate();

  geometry_msgs::TransformStamped map_base_link_transform;
  map_base_link_transform.header.stamp = ros::Time::now();
  map_base_link_transform.header.frame_id = "map";
  map_base_link_transform.child_frame_id = "base_link";
  map_base_link_transform.transform.translation.x = position_estimate(0);
  map_base_link_transform.transform.translation.y = position_estimate(1);
  map_base_link_transform.transform.translation.z = 0.0;

  map_base_link_transform.transform.rotation.w = cos(position_estimate(2) * 0.5);
  map_base_link_transform.transform.rotation.x = 0;
  map_base_link_transform.transform.rotation.y = 0;
  map_base_link_transform.transform.rotation.z = sin(position_estimate(2) * 0.5);

  tf_broadcaster_.sendTransform(map_base_link_transform);

  geometry_msgs::Pose2D robot_pose;
  robot_pose.x = position_estimate(0);
  robot_pose.y = position_estimate(1);
  robot_pose.theta = position_estimate(2);

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
  Eigen::VectorXf observations = ekf_->getCurrentObservationsEstimate();
  for (int i = 0; i < observations.size() / 2; ++i)
  {
    float x = observations(i * 2);
    float y = observations(i * 2 + 1);

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

void EkfSlam::parseParameters(ros::NodeHandle& node)
{
  if (!node.getParam("odometry_noise_x", odometry_noise_(0)))
  {
    ROS_WARN("Couldn't get odometry_noise_x parameter, setting to default 0.05 [m]");
    odometry_noise_(0) = 0.05;
  }

  if (!node.getParam("odometry_noise_y", odometry_noise_(1)))
  {
    ROS_WARN("Couldn't get odometry_noise_y parameter, setting to default 0.05 [m]");
    odometry_noise_(1) = 0.05;
  }
  float odometry_noise_theta_degrees;
  if (!node.getParam("odometry_noise_theta", odometry_noise_theta_degrees))
  {
    ROS_WARN("Couldn't get odometry_noise_theta parameter, setting to default 5 [degrees]");
    odometry_noise_(2) = (5. * M_PI) / 180.;
  }
  else
  {
    odometry_noise_(2) = (odometry_noise_theta_degrees * M_PI) / 180.;
  }

  ROS_DEBUG_STREAM("Odometry noise: " << odometry_noise_);

  if (!node.getParam("observation_noise_x", observation_noise_(0)))
  {
    ROS_WARN("Couldn't get observation_noise_x parameter, setting to default 0.05 [m]");
    observation_noise_(0) = 0.05;
  }
  if (!node.getParam("observation_noise_y", observation_noise_(1)))
  {
    ROS_WARN("Couldn't get observation_noise_y parameter, setting to default 0.05 [m]");
    observation_noise_(1) = 0.05;
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