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

catkin_package(
  INCLUDE_DIRS
  include/
  LIBRARIES
  gsplines_follow_trajectory
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

add_library(gsplines_follow_trajectory
            src/follow_joint_trajectory_action_wrapper.cpp)

target_link_libraries(gsplines_follow_trajectory ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})

add_executable(follow_joint_trajectory_wrapper
               src/follow_joint_trajectory_action_wrapper_node.cpp)

target_link_libraries(follow_joint_trajectory_wrapper ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES} gsplines_follow_trajectory)

install(
  TARGETS gsplines_follow_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Install executable
install(TARGETS follow_joint_trajectory_wrapper
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/gsplines_follow_trajectory
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)

catkin_install_python(
  PROGRAMS scripts/random_minimum_jerk_trajectories
  scripts/action_transtion_overlay_text_publisher scripts/joint_states_to_float
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/random_minimum_jerk_gsplines DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
