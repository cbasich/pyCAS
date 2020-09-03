cmake_minimum_required(VERSION 2.8.3)
project(pyCAS)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  RobotStatus.msg
  DoorStatus.msg
  SSPState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)