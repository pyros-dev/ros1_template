###############################################################################
# Applications
###############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") #for multithreading

add_executable(multithread_fibonacci multithread_fibonacci_call.cpp)
target_link_libraries(multithread_fibonacci ${catkin_LIBRARIES} ${PROJECT_NAME})
install(TARGETS multithread_fibonacci RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
