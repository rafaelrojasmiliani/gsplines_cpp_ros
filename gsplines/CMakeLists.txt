cmake_minimum_required(VERSION 3.0.2)
project(gsplines_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp rospy gsplines_msgs)

find_package(Eigen3 REQUIRED)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/submodules/gsplines_cpp/include/
  include/
  LIBRARIES
  gsplines
  gsplines_ros
  CATKIN_DEPENDS
  roscpp
  rospy)

add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/pybind11)
add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/gsplines_cpp)
include_directories(${PROJECT_SOURCE_DIR}/submodules/gsplines_cpp/include)

add_library(gsplines_ros src/gsplines_ros.cpp)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include/)

target_link_libraries(gsplines_ros ${catkin_LIBRARIES} ${Eigen3_LIBRARIES}
                      gsplines)

pybind11_add_module(pygsplines_ros ${PROJECT_SOURCE_DIR}/bindings/bindings.cpp)
target_link_libraries(pygsplines_ros PUBLIC ${catkin_LIBRARIES}
                                            ${Eigen3_LIBRARIES} gsplines)

set_target_properties(pygsplines_ros PROPERTIES OUTPUT_NAME gsplines_ros)

install(TARGETS pygsplines pygsplines_ros
        LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(
  TARGETS gsplines gsplines_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/submodules/gsplines_cpp/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/gsplines_ros
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)
