##############################################################################
# Tests
##############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

############################################################################
# Sources
##############################################################################

# Using GLOB means a re-check of this CMakeLists has to be triggered
# every time a new file is added. E.g. with `touch CMakelists.txt`
file(GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

############################################################################
# Add Tests and Linking
##############################################################################

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
