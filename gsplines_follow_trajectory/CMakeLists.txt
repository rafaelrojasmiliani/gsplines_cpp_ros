cmake_minimum_required(VERSION 3.0.2)
project(gsplines_follow_trajectory)

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

add_library(gsplines_follow_joint_trajectory_action_wrapper
            src/follow_joint_trajectory_action_wrapper.cpp)

target_link_libraries(gsplines_follow_joint_trajectory_action_wrapper
                      ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

install(
  TARGETS gsplines_follow_joint_trajectory_action_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/gsplines_follow_trajectory
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)
