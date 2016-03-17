cmake_minimum_required(VERSION 2.8.3)
project(psm_haptics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  roscpp
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(psm_haptics_node src/psm_haptics.cpp)

target_link_libraries(psm_haptics_node ${catkin_LIBRARIES})
