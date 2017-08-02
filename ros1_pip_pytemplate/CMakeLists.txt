##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_pip_template)

##############################################################################
# Dependencies (we are using catkin_pip here)
##############################################################################

find_package(catkin REQUIRED
            COMPONENTS
            catkin_pip
            message_generation
)


##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY
    msg
  FILES
    Arg.msg
)

##############################################################################
# Services
##############################################################################

add_service_files(
  DIRECTORY
    srv
  FILES
    Get.srv
)

generate_messages()

##############################################################################
# Catkin Pip
##############################################################################

# Note : although catkin_pip can ask pip to install packages from requirements.txt
# We usually dont want to do this from CMake, to make sure all mandatory (non-dev) dependencies
# are properly listed in setup.py

catkin_pip_package(ros1_pip_template)


##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS
        scripts/httpbin_cli.py
        scripts/httpbin_node.py
        scripts/httpbin_launch.py
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

    catkin_add_pytests(ros1_pip_template/tests)

    ##########################################################################
    # Dev tests
    ##########################################################################

    catkin_add_pytests(tests)

endif()
