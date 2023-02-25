#include <ekf_slam/cone_detector.h>

#include <ros/ros.h>
#include <cmath>

ConeDetector::ConeDetector() : clusterization_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
  min_lidar_range_ = 0.1;
  max_lidar_range_ = 20.0;
  cluster_tolerance_distance_ = 0.1;
  min_cluster_size_ = 2;
  max_cluster_size_ = 250;

  cluster_extractor_.setClusterTolerance(cluster_tolerance_distance_);
  cluster_extractor_.setMinClusterSize(min_cluster_size_);
  cluster_extractor_.setMaxClusterSize(max_cluster_size_);

  lidar_to_base_link_quaternion_ = Eigen::Quaternionf(1, 0, 0, 0);
  lidar_to_base_link_translation_ = Eigen::Vector3f(0, 0, 0);
}

std::vector<Observation> ConeDetector::detectCones(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // convert laser scan points to 3D
  // https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/

  std::vector<pcl::PointXYZ> points;
  pcl::PointXYZ pt;
  int i = 0;
  for (auto x : scan->ranges)
  {
    if (x > min_lidar_range_ && x < max_lidar_range_)
    {
      pt.x = x * cos(scan->angle_min + i * scan->angle_increment);
      pt.y = x * sin(scan->angle_min + i * scan->angle_increment);
      pt.z = 0;

      // Use only points in front of the lidar
      if (pt.x > 0)
      {
        points.push_back(pt);
      }
    }
    ++i;
  }

  // Add points to point_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  point_cloud->width = points.size();
  point_cloud->height = 1;
  point_cloud->is_dense = false;
  point_cloud->points.resize(point_cloud->width * point_cloud->height);

  i = 0;
  for (auto x : points)
  {
    point_cloud->points[i] = x;
    ++i;
  }

  // Segmentation
  // http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

  clusterization_tree_->setInputCloud(point_cloud);

  std::vector<pcl::PointIndices> cluster_indices;

  cluster_extractor_.setSearchMethod(clusterization_tree_);
  cluster_extractor_.setInputCloud(point_cloud);
  cluster_extractor_.extract(cluster_indices);

  // Extract cone observations from groups
  std::vector<Observation> cones_observations;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    // Find bounding rectangle of points in XY plane
    float min_x = 1000;
    float max_x = -1000;
    float min_y = 1000;
    float max_y = -1000;
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      if (point_cloud->points[*pit].x < min_x)
      {
        min_x = point_cloud->points[*pit].x;
      }
      if (point_cloud->points[*pit].x > max_x)
      {
        max_x = point_cloud->points[*pit].x;
      }
      if (point_cloud->points[*pit].y < min_y)
      {
        min_y = point_cloud->points[*pit].y;
      }
      if (point_cloud->points[*pit].y > max_y)
      {
        max_y = point_cloud->points[*pit].y;
      }
    }

    // Set cone to be in the middle of the bouding rectangle
    Eigen::Vector3f v;
    v(0) = (min_x + max_x) / 2;
    v(1) = (min_y + max_y) / 2;
    v(2) = 0;

    // Transform to base_link coordinate frame
    v = lidar_to_base_link_quaternion_ * v + lidar_to_base_link_translation_;

    // Convert to polar coordinates
    Observation p;
    p.r = sqrt(pow(v(0), 2) + pow(v(1), 2));
    p.angle = BoundToMinusPiPi(atan2(v(1), v(0)));

    cones_observations.push_back(p);
  }

  return cones_observations;
}

void ConeDetector::setLidarBaseLinkTransformation(Eigen::Vector3f translation, Eigen::Quaternionf quaternion)
{
  lidar_to_base_link_translation_ = translation;
  lidar_to_base_link_quaternion_ = quaternion;
}

bool ConeDetector::setMinLidarRange(float min_range)
{
  if (min_range > 0.0)
  {
    min_lidar_range_ = min_range;
    if (min_range > 2.0)
    {
      ROS_WARN("min_range parameter is more than 2 [m]");
    }
    return true;
  }
  else
  {
    ROS_ERROR("min_range parameter should be positive, setting to defult 0.1 [m]");
    min_lidar_range_ = 0.1;
    return false;
  }
}

bool ConeDetector::setMaxLidarRange(float max_range)
{
  if (max_range > 0.0)
  {
    max_lidar_range_ = max_range;
    if (max_range < 2.0)
    {
      ROS_WARN("max_lidar_range_ parameter is less than 2 [m]");
    }
    return true;
  }
  else
  {
    ROS_ERROR("max_lidar_range_ parameter should be positive, setting to defult 20 [m]");
    max_lidar_range_ = 20.0;
    return false;
  }
}

bool ConeDetector::setMinClusterSize(int min_cluster_size)
{
  if (min_cluster_size > 0)
  {
    min_cluster_size_ = min_cluster_size;
    cluster_extractor_.setMinClusterSize(min_cluster_size_);
    if (min_cluster_size_ > 10)
    {
      ROS_WARN("min_cluster_size parameter could be too big (more than 10)");
    }
    return true;
  }
  else
  {
    ROS_ERROR("min_cluster_size_ parameter should be positive, setting to defult 2");
    min_cluster_size_ = 2;
    cluster_extractor_.setMinClusterSize(min_cluster_size_);
    return false;
  }
}

bool ConeDetector::setMaxClusterSize(int max_cluster_size)
{
  if (max_cluster_size > 0)
  {
    max_cluster_size_ = max_cluster_size;
    cluster_extractor_.setMaxClusterSize(max_cluster_size_);
    if (max_cluster_size_ < 10)
    {
      ROS_WARN("max_cluster_size_ parameter could be too small (less than 10)");
    }
    return true;
  }
  else
  {
    ROS_ERROR("max_cluster_size_ parameter should be positive, setting to defult 250");
    max_cluster_size_ = 250;
    cluster_extractor_.setMaxClusterSize(max_cluster_size_);
    return false;
  }
}

bool ConeDetector::setClusterToleranceDistance(float cluster_tolerance_distance)
{
  if (cluster_tolerance_distance > 0.0)
  {
    cluster_tolerance_distance_ = cluster_tolerance_distance;
    cluster_extractor_.setClusterTolerance(cluster_tolerance_distance_);
    if (cluster_tolerance_distance_ > 0.2)
    {
      ROS_WARN("cluster_tolerance_distance_ parameter could be too big (less than 0.2 [m])");
    }
    return true;
  }
  else
  {
    ROS_ERROR("cluster_tolerance_distance_ parameter should be positive, setting to defult 0.1 [m]");
    cluster_tolerance_distance_ = 0.1;
    cluster_extractor_.setClusterTolerance(cluster_tolerance_distance_);
    return false;
  }
}