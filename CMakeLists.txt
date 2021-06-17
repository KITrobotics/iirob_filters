cmake_minimum_required(VERSION 3.16.3)
project(iirob_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  filters
  geometry_msgs
  rclcpp
  rosparam_shortcuts
  tf2_geometry_msgs
  tf2_ros
  pluginlib
)

add_library(low_pass_filter SHARED
  src/low_pass_filter.cpp
)

target_include_directories(low_pass_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(low_pass_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_options(low_pass_filter PRIVATE -Wall -Wextra)


add_library(gravity_compensation SHARED
  src/gravity_compensation.cpp
)

target_include_directories(gravity_compensation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gravity_compensation ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_options(gravity_compensation PRIVATE -Wall -Wextra)

# TODO(sjahr) Re-enable low_pass_filter, threshold_filter, kalman_filter, moving_mean_filter

# Install Libraries
install(
  TARGETS low_pass_filter gravity_compensation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(gravity_compensation low_pass_filter)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_gravity_compensation test/test_gravity_compensation.cpp)
  ament_target_dependencies(test_gravity_compensation ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_gravity_compensation gravity_compensation)

  ament_add_gtest(test_low_pass_filter test/test_low_pass_filter.cpp)
  ament_target_dependencies(test_low_pass_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_low_pass_filter low_pass_filter)

endif()

# Install pluginlib xml
pluginlib_export_plugin_description_file(iirob_filters filters_plugin.xml)

ament_package()
