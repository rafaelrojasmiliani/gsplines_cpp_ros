include(GoogleTest)
link_directories(${PROJECT_SOURCE_DIR}/build)

file(GLOB_RECURSE test_list ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(file_path ${test_list})
  get_filename_component(barename ${file_path} NAME)
  string(REPLACE ".cpp" "" new_name ${barename})
  catkin_add_gtest(${new_name} ${file_path})
  target_link_libraries(${new_name} gsplines_ros ${gsplines_LIBRARIES} gtest)
endforeach()
