include(GoogleTest)
find_package(rostest REQUIRED)
link_directories(${PROJECT_SOURCE_DIR}/build)

file(GLOB_RECURSE test_list ${PROJECT_SOURCE_DIR}/test/*.cpp)
file(GLOB_RECURSE python_test_list ${PROJECT_SOURCE_DIR}/test/*.py)
file(GLOB_RECURSE node_list ${PROJECT_SOURCE_DIR}/test/node*.cpp)
file(GLOB_RECURSE python_node_list ${PROJECT_SOURCE_DIR}/test/node*.py)

# remove nodes from test list
foreach(file_path ${node_list})
  list(REMOVE_ITEM test_list ${file_path})
endforeach()

# remove python nodes from test list
foreach(file_path ${python_node_list})
  list(REMOVE_ITEM python_test_list ${file_path})
endforeach()

# add catking gtest for each cpp in tests/
foreach(file_path ${test_list})
  get_filename_component(barename ${file_path} NAME)
  string(REPLACE ".cpp" "" new_name ${barename})
  catkin_add_gtest(${new_name} ${file_path})
  target_link_libraries(${new_name} gsplines_ros ${gsplines_LIBRARIES} gtest)
endforeach()

foreach(file_path ${node_list})
  get_filename_component(barename ${file_path} NAME)
  string(REPLACE ".cpp" "" new_name ${barename})
  add_rostest_gtest(${new_name} ${new_name}.test ${file_path})
  target_link_libraries(${new_name} gsplines_ros ${gsplines_LIBRARIES} gtest)
endforeach()

foreach(file_path ${python_test_list})
  # get_filename_component(barename ${file_path} NAME) string(REPLACE ".py"
  # "_py" new_name ${barename}) string(REPLACE ".py" "" mod_name ${barename})
  catkin_add_nosetests(${file_path} WORKING_DIRECTORY
                       ${CMAKE_SOURCE_DIR}/python/src)
endforeach()
