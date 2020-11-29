cmake_minimum_required(VERSION 2.8.3)
project(monte_carlo_localisation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_filters
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

catkin_package(INCLUDE_DIRS include)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_library
  src/monte_carlo_localisation/measurement_models/likelihood_field_model.cpp
  src/monte_carlo_localisation/motion_models/diff_odom_motion_model.cpp
  src/monte_carlo_localisation/map_utils.cpp
  src/monte_carlo_localisation/mcl_node.cpp
  src/monte_carlo_localisation/motion_utils.cpp
  src/monte_carlo_localisation/particle.cpp
  src/monte_carlo_localisation/particle_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_library ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/monte_carlo_localisation/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME}_library)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()