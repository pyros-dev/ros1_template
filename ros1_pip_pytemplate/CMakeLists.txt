##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_pip_pytemplate)

##############################################################################
# Dependencies (we are using catkin_pip here)
##############################################################################

find_package(catkin REQUIRED
            COMPONENTS
            catkin_pip
)

##############################################################################
# Catkin Pip
##############################################################################

# Note : although catkin_pip can ask pip to install packages from requirements.txt
# We usually dont want to do this from CMake, to make sure all mandatory (non-dev) dependencies
# are properly listed in setup.py

catkin_pip_package(ros1_pip_pytemplate)


##############################################################################
# Installs
##############################################################################

#  Working around https://github.com/pyros-dev/catkin_pip/issues/119 for now
#
# # Should these be handled by setup.py and pip ??
# install(
#     PROGRAMS
#         scripts/httpbin_cli.py
#         scripts/httpbin_node.py
#         scripts/httpbin_launch.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )
#
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)

    ##########################################################################
    # User Embedded tests
    ##########################################################################

    catkin_add_pytests(ros1_pip_pytemplate/tests)

    ##########################################################################
    # Dev tests
    ##########################################################################

    catkin_add_pytests(tests)

endif()
