cmake_minimum_required(VERSION 2.8.3)
project(ekf_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  std_msgs
)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ekf_slam_node src/ekf_slam_node.cpp src/ekf_slam.cpp 
  src/cone_detector.cpp src/util.cpp)

target_link_libraries(ekf_slam_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ekf_slam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)