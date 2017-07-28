##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_template)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslint)
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
endif()
catkin_package()

catkin_python_setup()

##############################################################################
# Messages
##############################################################################

# TODO


##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS
        nodes/template_node.py
        scripts/question.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)

    ##############################################################################
    # Nose Tests
    ##############################################################################

    catkin_add_nosetests(tests)

    ##############################################################################
    # Ros Tests
    ##############################################################################

    add_subdirectory(rostests)

endif()