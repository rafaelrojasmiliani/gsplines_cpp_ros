cmake_minimum_required(VERSION 3.0.2)
project(gsplines_follow_trajectory_control)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib
             actionlib_msgs
             control_msgs
             geometry_msgs
             gsplines_ros
             gsplines_msgs
             joint_trajectory_controller
             roscpp
             rospy
             std_msgs
             sensor_msgs
             trajectory_msgs
             message_generation)

find_package(Eigen3 REQUIRED)

add_action_files(DIRECTORY action FILES FollowJointGSpline.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs
                  sensor_msgs gsplines_msgs)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  gsplines_ros
  gsplines_msgs
  joint_trajectory_controller
  roscpp
  rospy
  std_msgs
  trajectory_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include/)

add_library(gsplines_follow_trajectory_control src/wrapper.cpp)

install()
