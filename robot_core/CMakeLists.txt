cmake_minimum_required(VERSION 3.0.2)
project(robot_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  panda_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_core panda_msgs
  CATKIN_DEPENDS generate_messages geometry_msgs roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/robot_main.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
