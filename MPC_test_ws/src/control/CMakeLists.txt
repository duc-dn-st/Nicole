cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sdv_msgs
  tf
  utilities
  drivers
  dynamic_reconfigure
)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/ControlGains.cfg
)

###########
## Build ##
###########


set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/controller_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/open_loop.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpc_ackerman.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/controller_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/simple_controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/open_loop.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/mpc_ackerman.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/pure_pursuit.h
    )

add_library(control_libs ${SOURCES} ${HEADERS})
target_link_libraries(control_libs ${catkin_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} control_libs)

add_executable(low_level_controller src/low_level_controller.cpp)
target_link_libraries(low_level_controller ${catkin_LIBRARIES})

add_executable(controller_node_test src/controller_node_test.cpp)
target_link_libraries(controller_node_test ${catkin_LIBRARIES} control_libs)
