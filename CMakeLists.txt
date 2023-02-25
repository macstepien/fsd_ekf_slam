cmake_minimum_required(VERSION 2.8.3)
project(ekf_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ekf_slam_node src/ekf_slam_node.cpp src/ekf_slam.h src/ekf_slam.cpp 
  src/cone_detector.h src/cone_detector.cpp src/util.h src/util.cpp)

target_link_libraries(ekf_slam_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_add_gtest(utest test/tests.cpp src/ekf_slam.h src/ekf_slam.cpp)
target_link_libraries(utest ${catkin_LIBRARIES})