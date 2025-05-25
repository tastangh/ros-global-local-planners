cmake_minimum_required(VERSION 3.0.2)
project(blm6191_coverage_planner)

## Compile as C++14 or C++17 (Recommended)
add_compile_options(-std=c++14) # or use c++17

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  tf2_ros # tf2_ros ekledik
  tf2_geometry_msgs # tf2_geometry_msgs ekledik
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME} # Library name is implicitly added by add_library
  CATKIN_DEPENDS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  tf2_ros # Depend added
  tf2_geometry_msgs # Depend added
  # DEPENDS # System dependencies like boost, Eigen etc.
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build C++ libraries
add_library(${PROJECT_NAME}
  src/coverage_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install
# Binary files (libraries and executables)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # Typically not needed for libraries, but harmless
)

# Header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Plugin description file
install(FILES blm6191_coverage_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Launch files (Install your launch files)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Config files (Install your config files)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)