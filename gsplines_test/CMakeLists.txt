cmake_minimum_required(VERSION 3.0.2)
project(gsplines_test)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp rospy std_msgs gsplines)

find_package(Eigen3 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES gsplines_test CATKIN_DEPENDS genmsg roscpp
  # rospy std_msgs DEPENDS Eigen3
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  # TODO: Check names of system library include directories (Eigen3)
  ${Eigen3_INCLUDE_DIRS})
