#ifndef CONE_DETECTOR_H
#define CONE_DETECTOR_H

#include <vector>

#include <Eigen/Dense>

#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/LaserScan.h>

#include <fsd_ekf_slam/util.h>

class ConeDetector
{
private:
  float min_lidar_range_;
  float max_lidar_range_;
  int min_cluster_size_;
  int max_cluster_size_;
  float cluster_tolerance_distance_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr clusterization_tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor_;

  Eigen::Quaternionf lidar_to_base_link_quaternion_;
  Eigen::Vector3f lidar_to_base_link_translation_;

public:
  ConeDetector();
  void setLidarBaseLinkTransformation(Eigen::Vector3f translation, Eigen::Quaternionf quaternion);
  bool setMinLidarRange(float min_range);
  bool setMaxLidarRange(float max_range);
  bool setMinClusterSize(int min_cluster_size);
  bool setMaxClusterSize(int max_cluster_size);
  bool setClusterToleranceDistance(float cluster_tolerance_distance);
  std::vector<Observation> detectCones(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif