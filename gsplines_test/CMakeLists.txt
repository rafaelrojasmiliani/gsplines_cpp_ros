cmake_minimum_required(VERSION 3.0.2)
project(gsplines_test)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp rospy std_msgs
                                        gsplines_ros gsplines_follow_trajectory)

find_package(Eigen3 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES gsplines_test CATKIN_DEPENDS genmsg roscpp
  # rospy std_msgs DEPENDS Eigen3
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
                    /usr/include/eigen3/)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
