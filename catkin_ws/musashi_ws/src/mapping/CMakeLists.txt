cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  eigen_conversions
  tf_conversions
)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapping
  DEPENDS Eigen3
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(inflate_static_map_node src/inflate_static_map_node.cpp)
target_link_libraries(inflate_static_map_node ${catkin_LIBRARIES})

add_executable(pc_to_local_map_node src/pc_to_local_map_node.cpp)
target_link_libraries(pc_to_local_map_node ${catkin_LIBRARIES})

add_executable(pc_to_local_map_no_inflation_node src/pc_to_local_map_no_inflation_node.cpp)
target_link_libraries(pc_to_local_map_no_inflation_node ${catkin_LIBRARIES})
