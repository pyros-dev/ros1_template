##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(launchers/fibonacci_launch.test)
add_rostest(launchers/answer_launch.test)
add_rostest(launchers/all_launch.test)
add_rostest(answer_server/answer_server.test)
add_rostest(fibonacci_pubsub/fibonacci_pub.test)
add_rostest(fibonacci_pubsub/fibonacci_sub.test)

