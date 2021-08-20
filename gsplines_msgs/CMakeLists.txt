cmake_minimum_required(VERSION 3.0.2)
project(gsplines_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS control_msgs
             geometry_msgs
             roscpp
             rospy
             sensor_msgs
             std_msgs
             trajectory_msgs
             message_generation)

add_message_files(DIRECTORY msg FILES GSpline.msg JointGSpline.msg)
add_action_files(DIRECTORY action FILES FollowJointGSpline.action)

generate_messages(
  DEPENDENCIES
  control_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  actionlib_msgs)

catkin_package(CATKIN_DEPENDS control_msgs geometry_msgs sensor_msgs std_msgs
               trajectory_msgs)

install()
