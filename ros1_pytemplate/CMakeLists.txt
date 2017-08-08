##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_pytemplate)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED
            COMPONENTS
            roslint
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
            COMPONENTS
             rostest
  )
endif()


##############################################################################
# Python
##############################################################################

catkin_python_setup()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*.py nodes/*.py ros1_pytemplate/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

##############################################################################
# Catkin
##############################################################################

catkin_package()


##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS
        nodes/answer_server_node.py
        nodes/fibonacci_sub_node.py
        nodes/fibonacci_pub_node.py
        scripts/question_cli.py
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
    # Nose Tests
    ##########################################################################

    catkin_add_nosetests(tests)

    ##########################################################################
    # Ros Tests
    ##########################################################################

    add_subdirectory(rostests)

endif()
