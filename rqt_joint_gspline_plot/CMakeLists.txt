cmake_minimum_required(VERSION 3.0.2)
project(rqt_joint_gspline_plot)

find_package(
  catkin REQUIRED
  COMPONENTS gsplines_msgs
             gsplines_ros
             joint_trajectory_controller
             roscpp
             rospy
             rqt_gui
             rqt_gui_cpp
             rqt_gui_py
             sensor_msgs
             std_msgs
             trajectory_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
