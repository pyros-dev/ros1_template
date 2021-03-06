##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ros1_cpptemplate)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED
            COMPONENTS
              roslint
              rosconsole
)

##############################################################################
# Catkin
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        rosconsole
)

##############################################################################
# Project Configuration
##############################################################################

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_definitions(${catkin_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Subdirectories includes
##############################################################################

add_subdirectory(src)

##############################################################################
# Installs
##############################################################################
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

##############################################################################
# Linter
##############################################################################

# Run with catkin_make roslint_${PROJECT_NAME}
set(ROSLINT_CUSTOM_FILTER "--filter=-legal/copyright,-build/c++11,-runtime/references,-readability/namespace")
set(ROSLINT_CUSTOM_EXTENSIONS "--extensions=h,hpp,cpp")
set(ROSLINT_CPP_OPTS ${ROSLINT_CUSTOM_FILTER} ${ROSLINT_CUSTOM_EXTENSIONS})

file(GLOB_RECURSE LINT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp *.cpp)
roslint_cpp(${LINT_FILES})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
    roslint_add_test()
    add_subdirectory(test)
endif()
