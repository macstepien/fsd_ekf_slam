#ifndef __CONE_DETECTOR__
#define __CONE_DETECTOR__

#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/geometry.h>
#include <pcl/impl/point_types.hpp>

#include "util.h"

class ConeDetector
{
    private:
        float minRange;
        float maxRange;
        int minClusterSize;
        int maxClusterSize;
        float clusterToleranceDistance;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr coneTree;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterExtractor;
        Eigen::Quaternionf velodyneBaseLinkQuat;
        Eigen::Vector3f velodyneBaseLinkTrans;
    public:
        ConeDetector();
        void setLidarBaseLinkTransformation(Eigen::Quaternionf quat, Eigen::Vector3f trans);
        bool setMinRange(float minRangeNew);
        bool setMaxRange(float maxRangeNew);
        bool setMinClusterSize(int minClusterSizeNew);
        bool setMaxClusterSize(int maxClusterSizeNew);
        bool setClusterToleranceDistance(float clusterToleranceDistanceNew);
        std::vector<ConePoint> detectCones(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif