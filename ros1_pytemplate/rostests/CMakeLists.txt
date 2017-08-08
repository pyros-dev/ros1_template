##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(answer_server/answer_server.test)
add_rostest(fibonacci_pubsub/fibonacci_pub.test)
add_rostest(fibonacci_pubsub/fibonacci_sub.test)
