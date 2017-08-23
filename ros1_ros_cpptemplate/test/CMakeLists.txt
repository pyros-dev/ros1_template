##############################################################################
# Tests
##############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##############################################################################
# Macro
##############################################################################

# tests are supposed to have <name>.test and <name>.cpp file
macro(${PROJECT_NAME}_rostest_gtest_macro test_name)
  add_rostest_gtest(test_${test_name} ${test_name}.test ${test_name}.cpp)
  target_link_libraries(test_${test_name} ${catkin_LIBRARIES})
endmacro()

##############################################################################
# Tests
##############################################################################

ros1_ros_cpptemplate_rostest_gtest_macro(publishers)
ros1_ros_cpptemplate_rostest_gtest_macro(subscribers)
ros1_ros_cpptemplate_rostest_gtest_macro(service)
