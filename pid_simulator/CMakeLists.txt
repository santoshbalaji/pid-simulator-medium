cmake_minimum_required(VERSION 3.0.2)
project(pid_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  turtlesim
  pid_simulator_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(position_controller src/position_controller.cpp src/pid.cpp)
target_link_libraries(position_controller ${catkin_LIBRARIES})