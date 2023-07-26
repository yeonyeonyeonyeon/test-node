cmake_minimum_required(VERSION 3.0.2)
project(test_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs std_srvs
)

## Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/test_node1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)