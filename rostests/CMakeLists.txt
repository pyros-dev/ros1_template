##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(oracle/oracle.test)
add_rostest(prophet/prophet.test)
add_rostest(prophet/follower.test)
