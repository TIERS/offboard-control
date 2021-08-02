cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  # PCL
  sensor_msgs 
  pcl_ros
  tf2_geometry_msgs
  tiers_ros_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES offboard_control
 CATKIN_DEPENDS message_runtime sensor_msgs pcl_ros tiers_ros_msgs
)

###########
## Build ##
###########

include_directories(${YAMLCPP_INCLUDE_DIRS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## Declare C++ executables
add_executable(position_to_mavros_node src/position_to_mavros.cpp)
add_executable(safe_offboard_node src/safe_offboard.cpp)
add_executable(pc_filter_node src/pc_filter.cpp)

target_link_libraries(
  position_to_mavros_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  safe_offboard_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pc_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBS}
)


