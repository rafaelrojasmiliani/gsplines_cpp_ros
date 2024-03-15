cmake_minimum_required(VERSION 3.0.2)
project(gsplines_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS pybind11_catkin roscpp rospy
                                        gsplines_msgs)

find_package(Eigen3 REQUIRED)

find_package(GSplines REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  include/
  LIBRARIES
  gsplines_ros
  CATKIN_DEPENDS
  roscpp
  rospy)

add_library(gsplines_ros src/gsplines_ros.cpp)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
                    ${gsplines_INCLUDE_DIRS} include/)

target_link_libraries(gsplines_ros ${catkin_LIBRARIES} ${Eigen3_LIBRARIES}
                      ${gsplines_LIBRARIES})

install(
  TARGETS gsplines_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/gsplines_ros
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(
  PROGRAMS
  scripts/random_minimum_jerk_path_generator
  scripts/gspline_plotter
  scripts/gspline_dk_path_publisher
  scripts/gspline_to_joint_trajectory
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
