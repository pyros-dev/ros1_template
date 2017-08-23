##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)

# Unit (Node) tests
add_rostest(answer_server/answer_server.test)
add_rostest(fibonacci_pubsub/fibonacci_pub.test)
add_rostest(fibonacci_pubsub/fibonacci_sub.test)

# Integration (launchers + config) Tests
add_rostest(integration/fibonacci_pubsub.test)
add_rostest(integration/answer_server_cfg.test)
add_rostest(integration/all.test)

