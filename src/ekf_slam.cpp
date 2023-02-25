#include "ekf_slam.h"

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <limits>
#include <cmath>

#include <tf/tf.h>
#include <sstream>

#include "util.h"

#define DEBUG_MESSAGES
#define MAX_NUM_CONES 1000

EKF_SLAM::EKF_SLAM(ros::NodeHandle& node) : listener(tfBuffer)
{
    odometryNoise.setZero();
    observationNoise.setZero();
    H = Eigen::MatrixXf::Zero(2, 3+MAX_NUM_CONES*2);

    means = Eigen::MatrixXf::Zero(3 + MAX_NUM_CONES*2,1);
    covariances = Eigen::MatrixXf::Zero(3 + MAX_NUM_CONES*2,3 + MAX_NUM_CONES*2);

    odomPosition.setZero();
    positionEstimate.setZero();

    setParameters(node);
    
    getLidarBaseLinkTransform();

    speedFromOdom = 0;
    lastOdomStamp = ros::Time::now();

    conesNumber = 0;

    visualization = true;

    cone_pub = node.advertise<geometry_msgs::Point>("/cone_detector/cone_position", 50);
    cone_marker_pub = node.advertise<visualization_msgs::Marker>("/cone_detector/cone_marker", 50);
    position_pub = node.advertise<geometry_msgs::Pose2D>("/slam_pose", 50);

    //! Allow initial position from odometry - useful for testing
    odometryMeasurment();
    means.block<3,1>(0,0) = odomPosition;
}

void EKF_SLAM::updatePosition(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    #ifdef DEBUG_MESSAGES
    ROS_INFO("Starting position update");
    #endif

    Eigen::Vector3f odomChange = odometryMeasurment();

    #ifdef DEBUG_MESSAGES
    debugMsg << "Odometry obtained, position: " << odomPosition;
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif

    std::vector<ConePoint> detectedCones = detector.detectCones(scan);
    if (visualization)
        publishDetectedCones(detectedCones);

    #ifdef DEBUG_MESSAGES
    debugMsg << "Detected: " << detectedCones.size() << " cones";
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif

    ekfSlam(detectedCones, odomChange);

    publishPosition();
    if (visualization)
        publishConesMap();
}

void EKF_SLAM::ekfSlam(std::vector<ConePoint> detectedCones, Eigen::Vector3f odomChange)
{
    //Prediction

    //Mean prediction
    means.block<3,1>(0,0) += odomChange;
    means(2) = minus_pi_to_pi(means(2));

    //Covariance update
    Eigen::Matrix3f G = Eigen::Matrix3f::Identity();
    G(0, 2) -= odomChange(1);
    G(1, 2) += odomChange(0);

    covariances.block<3,3>(0,0) = G*covariances.block<3,3>(0,0)*G.transpose() + odometryNoise;
    covariances.block(0,3,3,conesNumber*2) = G*covariances.block(0,3,3,conesNumber*2);

    for(auto cone : detectedCones)
    {
        int index = -1;
        if (conesNumber > 0)
            index = findMatchingCone(cone);
        
        if (index < 0)
        {
            addNewCone(cone);
            ++conesNumber;
        }
        else
        {
            updatePositionFromObservation(cone, index);
        }
    }

    positionEstimate = means.block<3,1>(0,0);

    #ifdef DEBUG_MESSAGES
    debugMsg << "Finished position update, new position: " << means.block<3,1>(0,0);
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif
}

void EKF_SLAM::addNewCone(ConePoint cone)
{
    means(3+conesNumber*2, 0) = means(0) + cone.r*cos(means(2) + cone.angle);
    means(3+conesNumber*2+1, 0) = means(1) + cone.r*sin(means(2) + cone.angle);

    #ifdef DEBUG_MESSAGES
    debugMsg << "Found new cone, coordinates x: " << means(3+conesNumber*2, 0) << "y: " << means(3+conesNumber*2+1, 0);
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif

    covariances(3+conesNumber*2, 3+conesNumber*2) = 1;
    covariances(3+conesNumber*2+1, 3+conesNumber*2+1) = 1;
}

void EKF_SLAM::updatePositionFromObservation(ConePoint cone, int matchedIndex)
{
    //Calculate Expected Measurment
    float xEM = means(3+matchedIndex*2) - means(0);
    float yEM = means(3+matchedIndex*2+1) - means(1);

    float r2EM = pow(xEM,2) + pow(yEM,2);
    float rEM = sqrt(r2EM);

    //If update distance is too small, then there are problems with division in H1 and H2 matrices
    if (fabs(rEM - cone.r) > 0.01)
    {
        float angleEM = minus_pi_to_pi(atan2(yEM, xEM) - means(2));
        Eigen::Vector2f zExpected = Eigen::Vector2f(rEM, angleEM);
        Eigen::Vector2f zReal = Eigen::Vector2f(cone.r, cone.angle);

        //Observation Jacobian
        Eigen::MatrixXf H1 = Eigen::MatrixXf::Zero(2,3);
        Eigen::MatrixXf H2 = Eigen::MatrixXf::Zero(2,2);
        H1 << -rEM*xEM/r2EM, -rEM*yEM/r2EM, 0, yEM/r2EM, -xEM/r2EM, -1;
        H2 << rEM*xEM/r2EM, rEM*yEM/r2EM, -yEM/r2EM, xEM/r2EM;
        H.block<2,3>(0,0) = H1;
        H.block<2,2>(0,3+2*matchedIndex) = H2;
        
        Eigen::MatrixXf covariancesBlock = covariances.block(0,0,3+conesNumber*2,3+conesNumber*2);
        Eigen::MatrixXf HBlock = H.block(0,0,2,3+2*conesNumber);
        Eigen::MatrixXf K = covariancesBlock*HBlock.transpose()*(HBlock*covariancesBlock*HBlock.transpose()+observationNoise).inverse();
        

        Eigen::Vector2f dz = zReal - zExpected;
        dz(1) = minus_pi_to_pi(dz(1));
        
        means.block(0,0,3+conesNumber*2,1) += K*(dz);
        covariances.block(0,0,3+conesNumber*2,3+conesNumber*2) = (Eigen::MatrixXf::Identity(3+conesNumber*2, 3+conesNumber*2) - K*HBlock)*covariancesBlock;
        
        H.block<2,3>(0,0) = Eigen::MatrixXf::Zero(2,3);
        H.block<2,2>(0,3+2*matchedIndex) = Eigen::MatrixXf::Zero(2,2);
    }
}

int EKF_SLAM::findMatchingCone(ConePoint position)
{
    float xMean, yMean, rMean, angleMean, distance;
    float xDetected = position.r*cos(position.angle);
    float yDetected = position.r*sin(position.angle);
    for (int index = 0; index < conesNumber; ++index)
    {
        xMean = means(3 + index*2) - means(0);
        yMean = means(3 + index*2 + 1) - means(1);

        rMean = sqrt(pow(xMean,2) + pow(yMean,2));
        angleMean = minus_pi_to_pi(atan2(yMean, xMean) - means(2));

        xMean = rMean*cos(angleMean);
        yMean = rMean*sin(angleMean);

        distance = sqrt(pow(xDetected - xMean,2) + pow(yDetected - yMean,2));
        
        float scalingSpeed = 0.8;
        if (speedFromOdom < 0 || speedFromOdom > 10 || std::isnan(speedFromOdom))
            speedFromOdom = 0;

        if (distance < coneMatchingMaxDistance * (speedFromOdom>scalingSpeed?speedFromOdom/scalingSpeed:1))
            return index;
    }
    //Matching cone not found
    return -1;
}



Eigen::Vector3f EKF_SLAM::odometryMeasurment()
{
    geometry_msgs::TransformStamped odomBaseFootprintTransform;

    try 
    {
        //Time(0) - latest available transform
        //Time::now() - current time
        odomBaseFootprintTransform = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time(0), ros::Duration(10.0));
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Quaternionf q(odomBaseFootprintTransform.transform.rotation.w, odomBaseFootprintTransform.transform.rotation.x,
                         odomBaseFootprintTransform.transform.rotation.y, odomBaseFootprintTransform.transform.rotation.z);

    
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Vector3f odomChange;
    odomChange(0) = (odomPosition(0) - odomBaseFootprintTransform.transform.translation.x);
    odomChange(1) = (odomPosition(1) - odomBaseFootprintTransform.transform.translation.y);
    odomChange(2) = minus_pi_to_pi(odomPosition(2) - euler(2));

    odomPosition(0) = odomBaseFootprintTransform.transform.translation.x;
    odomPosition(1) = odomBaseFootprintTransform.transform.translation.y;
    odomPosition(2) = minus_pi_to_pi(euler(2));

    float timeDiff = (odomBaseFootprintTransform.header.stamp - lastOdomStamp).toSec();
    speedFromOdom = sqrt(pow(odomChange(0), 2) + pow(odomChange(1), 2))/timeDiff;
    lastOdomStamp = odomBaseFootprintTransform.header.stamp;

    #ifdef DEBUG_MESSAGES
    debugMsg << "Speed from odom: " << speedFromOdom;
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif

    return odomChange;
}

void EKF_SLAM::publishPosition()
{
    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = positionEstimate(0);
    t.transform.translation.y = positionEstimate(1);
    t.transform.translation.z = 0.0;

    t.transform.rotation.w = cos(positionEstimate(2) * 0.5);
    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = sin(positionEstimate(2) * 0.5);

    positionTfPublisher.sendTransform(t);

    geometry_msgs::Pose2D position;
    position.x = positionEstimate(0);
    position.y = positionEstimate(1);
    position.theta = positionEstimate(2);
    
    position_pub.publish(position);
}

void EKF_SLAM::publishDetectedCones(std::vector<ConePoint> cones)
{
    int i = 0;
    for (auto x : cones)
    {
        geometry_msgs::Point pmsg;
        pmsg.x = x.r * cos(x.angle);
        pmsg.y = x.r * sin(x.angle);
        pmsg.z = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp =  ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = pmsg;
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
        cone_marker_pub.publish( marker );
        ++i;
    }
}

void EKF_SLAM::publishConesMap()
{
    for (int i = 0; i < conesNumber; ++i)
    {
        float x = means(3 + i*2);
        float y = means(3 + i*2 + 1);

        geometry_msgs::Point pmsg;
        pmsg.x = x;
        pmsg.y = y;
        pmsg.z = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp =  ros::Time::now();
        marker.ns = "cones";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = pmsg;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.75;
        marker.color.g = 0.75;
        marker.color.b = 0.75;
        marker.lifetime = ros::Duration(1);
        cone_marker_pub.publish( marker );

        
    }
}

void EKF_SLAM::getLidarBaseLinkTransform()
{
    geometry_msgs::TransformStamped velodyneBaselinkTransform;
    bool transformValid = false;
    while (!transformValid)
    {
        transformValid = true;
        try 
        {
            velodyneBaselinkTransform = tfBuffer.lookupTransform("base_link", "velodyne", ros::Time(0), ros::Duration(10.0));
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            transformValid = false;
        }
    }
    
    //? DEBUG
    ROS_INFO("velodyne->base_link transform obtained");

    Eigen::Quaternionf velodyneBaseLinkQuat =  Eigen::Quaternionf(velodyneBaselinkTransform.transform.rotation.w, velodyneBaselinkTransform.transform.rotation.x,
                                                velodyneBaselinkTransform.transform.rotation.y, velodyneBaselinkTransform.transform.rotation.z);

    Eigen::Vector3f velodyneBaseLinkTrans;
    velodyneBaseLinkTrans(0) = velodyneBaselinkTransform.transform.translation.x;
    velodyneBaseLinkTrans(1) = velodyneBaselinkTransform.transform.translation.y;
    velodyneBaseLinkTrans(2) = 0;
    
    detector.setLidarBaseLinkTransformation(velodyneBaseLinkQuat, velodyneBaseLinkTrans);
}

void EKF_SLAM::setParameters(ros::NodeHandle& node)
{
    if (!node.getParam("odometryNoiseX", odometryNoise(0, 0)))
    {
        ROS_WARN("Couldn't get odometryNoiseX parameter, setting to default 0.05 [m]");
        odometryNoise(0, 0) = 0.05;
    }

    if (!node.getParam("odometryNoiseY", odometryNoise(1, 1)))
    {
        ROS_WARN("Couldn't get odometryNoiseY parameter, setting to default 0.05 [m]");
        odometryNoise(1, 1) = 0.05;
    }
    float degrees;
    if (!node.getParam("odometryNoiseTheta", degrees))
    {
        ROS_WARN("Couldn't get odometryNoiseTheta parameter, setting to default 5 [degrees]");
        odometryNoise(2, 2) = (5.*M_PI)/180.;
    }
    else
    {
        odometryNoise(2, 2) = (degrees*M_PI)/180.;
    }
    
    #ifdef DEBUG_MESSAGES
    debugMsg << "Odometry noise: " << odometryNoise;
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif

    if (!node.getParam("observationNoiseX", observationNoise(0,0)))
    {
        ROS_WARN("Couldn't get observationNoiseX parameter, setting to default 0.05 [m]");
        observationNoise(0,0) = 0.05;
    }
    if (!node.getParam("observationNoiseY", observationNoise(1,1)))
    {
        ROS_WARN("Couldn't get observationNoiseY parameter, setting to default 0.05 [m]");
        observationNoise(1,1) = 0.05;
    }
    #ifdef DEBUG_MESSAGES
    debugMsg << "Observation noise: " << observationNoise;
    ROS_INFO(debugMsg.str().c_str());
    debugMsg.str(std::string());
    #endif
    
    if (!node.getParam("coneMatchingMaxDistance", coneMatchingMaxDistance))
    {
        ROS_WARN("Couldn't get coneMatchingMaxDistance parameter, setting to default 0.5 [m]");
        coneMatchingMaxDistance = 0.5;
    }
    

    float minRange;
    if (!node.getParam("minLidarRange", minRange))
    {
        ROS_WARN("Couldn't get minRange parameter, leaving default");
    }
    else
    {
        detector.setMinRange(minRange);
    }

    float maxRange;
    if (!node.getParam("maxLidarRange", maxRange))
    {
        ROS_WARN("Couldn't get maxRange parameter, leaving default");
    }
    else
    {
        detector.setMaxRange(maxRange);
    }

    int minClusterSize;
    if (!node.getParam("minClusterSize", minClusterSize))
    {
        ROS_WARN("Couldn't get minClusterSize parameter, leaving default");
    }
    else
    {
        detector.setMinClusterSize(minClusterSize);
    }

    int maxClusterSize;
    if (!node.getParam("maxClusterSize", maxClusterSize))
    {
        ROS_WARN("Couldn't get maxClusterSize parameter, leaving default");
    }
    else
    {
        detector.setMaxClusterSize(maxClusterSize);
    }

    float clusterToleranceDistance;
    if (!node.getParam("clusterToleranceDistance", clusterToleranceDistance))
    {
        ROS_WARN("Couldn't get clusterToleranceDistance parameter, leaving default");
    }
    else
    {
        detector.setClusterToleranceDistance(clusterToleranceDistance);
    }
}