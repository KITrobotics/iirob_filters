cmake_minimum_required(VERSION 2.8.3)
project(iirob_filters)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  roscpp
  tf2_ros
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iirob_filters
  CATKIN_DEPENDS eigen_conversions geometry_msgs roscpp
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gravity_compensation
  src/gravity_compensation.cpp
)

add_library(low_pass_filter
  src/low_pass_filter.cpp
)

add_library(median_filter
  src/median_filter.cpp
)

add_library(threshold_filter
  src/threshold_filter.cpp
)

target_link_libraries(gravity_compensation ${catkin_LIBRARIES})
target_link_libraries(low_pass_filter ${catkin_LIBRARIES})
target_link_libraries(median_filter ${catkin_LIBRARIES})
target_link_libraries(threshold_filter ${catkin_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(iirob_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(iirob_filters_node src/iirob_filters_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(iirob_filters_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS iirob_filters iirob_filters_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_iirob_filters.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
