#include <ros/ros.h>

#include "ekf_slam.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ekf_slam");
    ros::NodeHandle node("~");

    EKF_SLAM poseEstimator(node);

    ros::Subscriber scanSub;
    scanSub = node.subscribe("/scan", 10, &EKF_SLAM::updatePosition, &poseEstimator);

    ros::spin();
}