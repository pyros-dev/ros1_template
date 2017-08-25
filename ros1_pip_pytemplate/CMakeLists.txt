##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.12)
project(ros1_pip_pytemplate)

##############################################################################
# Dependencies (we are using catkin_pip here)
##############################################################################

find_package(catkin REQUIRED)
find_package(catkin_pip REQUIRED)

##############################################################################
# Catkin Pip
##############################################################################

# Note : although catkin_pip can ask pip to install packages from requirements.txt
# We usually dont want to do this from CMake, to make sure all mandatory (non-dev) dependencies
# are properly listed in setup.py

catkin_pip_target(ros1_pip_pytemplate)
catkin_package()


##############################################################################
# Installs
##############################################################################

# Should these be handled by setup.py and pip ??
install(
    PROGRAMS
        scripts/cli.py
        scripts/node_reactive.py
        scripts/node_proactive.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)

    ##########################################################################
    # User Embedded tests
    ##########################################################################

    catkin_add_pytests(ros1_pip_pytemplate/tests)

    ##########################################################################
    # Dev tests (including ROS stuff)
    ##########################################################################

    catkin_add_pytests(tests)

    ##########################################################################
    # ROS tests : possible
    # but might hit some of rostest issues
    # https://github.com/ros/ros_comm/issues?utf8=%E2%9C%93&q=is%3Aissue%20rostest
    # and do not add any value compared to the previous Dev tests.
    ##########################################################################
    find_package(rostest)
    if (rostest_FOUND)
        add_rostest(tests/node_reactive.test)
        add_rostest(tests/node_proactive.test)
    endif()

endif()
