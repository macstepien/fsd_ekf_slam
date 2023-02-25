#include <ros/ros.h>

#include <ekf_slam/ekf_slam.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_slam");
  ros::NodeHandle node("~");

  EkfSlam ekf_slam(node);

  ros::Subscriber scan_sub;
  scan_sub = node.subscribe("/scan", 10, &EkfSlam::processScan, &ekf_slam);

  ros::spin();
}