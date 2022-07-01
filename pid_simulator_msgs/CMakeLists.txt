cmake_minimum_required(VERSION 3.0.2)
project(pid_simulator_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Feedback.msg
  PidFeedback.msg
  PidTuner.msg
  PositionFeedback.msg
  Target.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)