##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_template)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED
            COMPONENTS
            message_generation
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
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*.py nodes/*.py ros1_template/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY
    msg
  FILES
    Answer.msg
)

##############################################################################
# Services
##############################################################################

add_service_files(
  DIRECTORY
    srv
  FILES
    Question.srv
)

generate_messages()

##############################################################################
# Catkin
##############################################################################

catkin_package(CATKIN_DEPENDS
             message_runtime
)


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

    ##########################################################################
    # Nose Tests
    ##########################################################################

    catkin_add_nosetests(tests)

    ##########################################################################
    # Ros Tests
    ##########################################################################

    add_subdirectory(rostests)

endif()
