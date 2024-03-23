cmake_minimum_required(VERSION 3.0.2)
project(car_navigator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoints_move
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs tf2_geometry_msgs 
  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/odom_path_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_navigator_node src/car_navigator_node.cpp)
add_dependencies(car_navigator_node ${car_navigator_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_navigator_node ${catkin_LIBRARIES} )
