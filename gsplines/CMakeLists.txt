cmake_minimum_required(VERSION 3.0.2)
project(gsplines)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

find_package(Eigen3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/pybind11)
add_subdirectory(${CMAKE_SOURCE_DIR}/submodules/gsplines_cpp)

catkin_package(
  INCLUDE_DIRS
  submodules/gsplines_cpp/include
  LIBRARIES
  gsplines
  CATKIN_DEPENDS
  roscpp
  rospy)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
