#ifndef __EKF_SLAM__
#define __EKF_SLAM__

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include "cone_detector.h"

class EKF_SLAM
{
    private:
        Eigen::Matrix3f odometryNoise;
        Eigen::Matrix2f observationNoise;
        Eigen::MatrixXf H;

        Eigen::MatrixXf means;
        Eigen::MatrixXf covariances;

        Eigen::Vector3f odomPosition;
        Eigen::Vector3f positionEstimate;
        
        ConeDetector detector;

        float coneMatchingMaxDistance;
        float speedFromOdom;
        ros::Time lastOdomStamp;

        int conesNumber;

        bool visualization;

        ros::Publisher cone_pub;
        ros::Publisher cone_marker_pub;
        ros::Publisher position_pub;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener listener;
        tf2_ros::TransformBroadcaster positionTfPublisher;

        void setParameters(ros::NodeHandle& node);
        void getLidarBaseLinkTransform();
        
        std::stringstream debugMsg;
        
        void ekfSlam(std::vector<ConePoint> detectedCones, Eigen::Vector3f odomChange);

        Eigen::Vector3f odometryMeasurment();
        int findMatchingCone(ConePoint position);
        void addNewCone(ConePoint cone);
        void updatePositionFromObservation(ConePoint cone, int matchedIndex);

        //TF and /slam_pose publishing
        void publishPosition();

        //Visualization
        void publishConesMap();
        void publishDetectedCones(std::vector<ConePoint>);
        

    public:
        EKF_SLAM(ros::NodeHandle& node);
        void updatePosition(const sensor_msgs::LaserScan::ConstPtr& scan);
};



#endif