#include "cone_detector.h"

#include <ros/ros.h>
#include <cmath>

ConeDetector::ConeDetector() : tree(new pcl::search::KdTree<pcl::PointXYZ>), coneTree(new pcl::search::KdTree<pcl::PointXYZ>)
{
    minRange = 0.1;
    maxRange = 20.0;
    clusterToleranceDistance = 0.1;
    minClusterSize = 2;
    maxClusterSize = 250;
    
    clusterExtractor.setClusterTolerance(clusterToleranceDistance); 
    clusterExtractor.setMinClusterSize(minClusterSize);
    clusterExtractor.setMaxClusterSize(maxClusterSize);

    velodyneBaseLinkQuat =  Eigen::Quaternionf(1, 0, 0, 0);
    velodyneBaseLinkTrans = Eigen::Vector3f(0, 0, 0);
}

void ConeDetector::setLidarBaseLinkTransformation(Eigen::Quaternionf quat, Eigen::Vector3f trans)
{
    velodyneBaseLinkQuat = quat;
    velodyneBaseLinkTrans = trans;
}

bool ConeDetector::setMinRange(float minRangeNew)
{
    if (minRangeNew > 0)
    {
        minRange = minRangeNew;
        if (minRangeNew > 2)
            ROS_WARN("minRange parameter is more than 2 [m]");
        return true;
    }
    else
    {
        ROS_ERROR("minRange parameter should be positive, setting to defult 0.1 [m]");
        minRange = 0.1;
        return false;
    }
}

bool ConeDetector::setMaxRange(float maxRangeNew)
{
    if (maxRangeNew > 0)
    {
        maxRange = maxRangeNew;
        if (maxRangeNew < 2)
            ROS_WARN("maxRange parameter is less than 2 [m]");
        return true;
    }
    else
    {
        ROS_ERROR("maxRange parameter should be positive, setting to defult 20 [m]");
        maxRange = 20.0;
        return false;
    }
}

bool ConeDetector::setMinClusterSize(int minClusterSizeNew)
{
    if (minClusterSizeNew > 0)
    {
        minClusterSize = minClusterSizeNew;
        clusterExtractor.setMinClusterSize(minClusterSize);
        if (minClusterSizeNew > 10)
            ROS_WARN("minClusterSize parameter could be too big (more than 10)");
        return true;
    }
    else
    {
        ROS_ERROR("minClusterSize parameter should be positive, setting to defult 2");
        minClusterSize = 2;
        clusterExtractor.setMinClusterSize(minClusterSize);
        return false;
    }
}

bool ConeDetector::setMaxClusterSize(int maxClusterSizeNew)
{
    if (maxClusterSizeNew > 0)
    {
        maxClusterSize = maxClusterSizeNew;
        clusterExtractor.setMaxClusterSize(maxClusterSize);
        if (maxClusterSizeNew < 10)
            ROS_WARN("maxClusterSize parameter could be too small (less than 10)");
        return true;
    }
    else
    {
        ROS_ERROR("maxClusterSize parameter should be positive, setting to defult 250");
        maxClusterSize = 250;
        clusterExtractor.setMaxClusterSize(maxClusterSize);
        return false;
    }
}

bool ConeDetector::setClusterToleranceDistance(float clusterToleranceDistanceNew)
{
    if (clusterToleranceDistanceNew > 0)
    {
        clusterToleranceDistance = clusterToleranceDistanceNew;
        clusterExtractor.setClusterTolerance(clusterToleranceDistance); 
        if (clusterToleranceDistance > 0.2)
            ROS_WARN("clusterToleranceDistance parameter could be too big (less than 0.2 [m])");
        return true;
    }
    else
    {
        ROS_ERROR("clusterToleranceDistance parameter should be positive, setting to defult 0.1 [m]");
        clusterToleranceDistance = 0.1;
        clusterExtractor.setClusterTolerance(clusterToleranceDistance); 
        return false;
    }
}

std::vector<ConePoint> ConeDetector::detectCones(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::vector<pcl::PointXYZ> points;
    // przeksztalcamy otrzymane range na punkty w 3D
    //https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/
    
    pcl::PointXYZ pt;
    int i = 0;
    for (auto x : scan->ranges)
    {
        if(x > minRange && x < maxRange)
        {
            pt.x = x * cos(scan->angle_min + i*scan->angle_increment);
            pt.y = x * sin(scan->angle_min + i*scan->angle_increment);
            pt.z = 0;

            if(pt.x > 0)
                points.push_back(pt);
        }
        ++i;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width    = points.size();
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    i = 0;
    for (auto x : points)
    {
        cloud->points[i] = x;
        ++i;
    }

    //segmentacja
    //http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
    
    
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    
    clusterExtractor.setSearchMethod(tree);
    clusterExtractor.setInputCloud(cloud);

    clusterExtractor.extract(cluster_indices);

    std::vector<ConePoint> observations;

    float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        minX = 1000;
        maxX = -1000;
        minY = 1000;
        maxY = -1000;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            if (cloud->points[*pit].x < minX)
                minX = cloud->points[*pit].x;
            if (cloud->points[*pit].x > maxX)
                maxX = cloud->points[*pit].x;
            if (cloud->points[*pit].y < minY)
                minY = cloud->points[*pit].y;
            if (cloud->points[*pit].y > maxY)
                maxY = cloud->points[*pit].y;
        }

        Eigen::Vector3f v;
        v(0) = (minX + maxX) / 2;
        v(1) = (minY + maxY) / 2;
        v(2) = 0;
        v = velodyneBaseLinkQuat * v + velodyneBaseLinkTrans;

        ConePoint p;
        p.r = sqrt(pow(v(0),2) + pow(v(1),2));
        p.angle = minus_pi_to_pi(atan2(v(1), v(0)));

        observations.push_back(p);
    }

    return observations;
}