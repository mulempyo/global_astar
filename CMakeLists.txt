cmake_minimum_required(VERSION 3.0.2)
project(kwj_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  geometry_msgs
  message_generation
  nav_core
  nav_msgs
  pluginlib
  rosconsole
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
     DEPENDENCIES
          geometry_msgs
)
## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kwj_global_planner
  CATKIN_DEPENDS cmake_modules costmap_2d geometry_msgs nav_core nav_msgs pluginlib rosconsole roscpp sensor_msgs tf2_ros visualization_msgs 
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(kwj_global_planner
  src/kwj_global.cpp
  src/kwj_ros.cpp
)

target_link_libraries(kwj_global_planner ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
add_dependencies(kwj_global_planner ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(kwj_node src/kwj_node.cpp)

## Link the executable to the library
target_link_libraries(kwj_node kwj_global_planner ${catkin_LIBRARIES})

## Install targets
install(TARGETS kwj_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS kwj_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/kwj_global/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

