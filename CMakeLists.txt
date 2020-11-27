cmake_minimum_required(VERSION 3.0.2)
project(hera_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
