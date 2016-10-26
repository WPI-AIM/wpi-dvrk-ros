cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  roscpp
  tf
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(psmIK_node src/psmIK.cpp)
add_executable(psmMP_node src/psmMP.cpp)
add_executable(mtmMP_node src/mtmMP.cpp)

target_link_libraries(psmIK_node ${catkin_LIBRARIES})
target_link_libraries(psmMP_node ${catkin_LIBRARIES})
target_link_libraries(mtmMP_node ${catkin_LIBRARIES})

