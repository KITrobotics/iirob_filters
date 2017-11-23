cmake_minimum_required(VERSION 2.8.3)
project(iirob_filters)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure 
  eigen_conversions
  geometry_msgs
  roscpp
  rosparam_handler
  tf2_ros
  filters
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
else()
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES}) 
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (Cpp11CompilerFlag)
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

endif()

generate_ros_parameter_files(
    cfg/Threshold.params
    cfg/LowPassFilter.params
    cfg/GravityCompensation.params
    cfg/MovingMean.params
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS eigen_conversions geometry_msgs roscpp filters
  DEPENDS Eigen
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/gravity_compensation.cpp
  src/low_pass_filter.cpp  
  src/threshold_filter.cpp
  src/moving_mean_filter.cpp
  
   include/${PROJECT_NAME}/low_pass_filter.h
   include/${PROJECT_NAME}/iirob_filter_base.h
   include/${PROJECT_NAME}/gravity_compensation.h
   include/${PROJECT_NAME}/threshold_filter.h
   include/${PROJECT_NAME}/moving_mean_filter.h
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg  # For dynamic_reconfigure
  ${PROJECT_NAME}_genparam  # For rosparam_handler
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(FILES filters_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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
