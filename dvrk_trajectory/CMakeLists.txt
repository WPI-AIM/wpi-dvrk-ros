cmake_minimum_required(VERSION 2.8.3)
project(dvrk_trajectory)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dvrk_trajectory_node src/dvrk_trajectory.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(dvrk_trajectory_node
   ${catkin_LIBRARIES}
 )

