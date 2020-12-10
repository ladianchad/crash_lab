cmake_minimum_required(VERSION 3.0.2)
project(panda_msgs)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
  sensor_msgs
)


add_message_files(
  FILES
  Yolo.msg
)

add_service_files(
  FILES
  Depth.srv
  tilt.srv
  motion.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs  std_msgs sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

