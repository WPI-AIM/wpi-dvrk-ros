cmake_minimum_required(VERSION 2.8.3)
project(mtm_haptics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cisst_msgs
  cisst_ros_bridge
  geometry_msgs
  moveit_core
  moveit_msgs
  roscpp
  roslib
  sensor_msgs
  tf
)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstRobot
  cisstNumerical
  cisstQt)
# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mtm_haptics
  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge geometry_msgs moveit_core moveit_msgs roscpp roslib sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_library(force_filter src/force_filter.cpp include/mtm_haptics/force_filter.h)

add_executable(test_mtm_haptics src/test_mtm_haptics.cpp
                                 include/mtm_haptics/haptic_position_kinematics.h
                                 src/haptic_position_kinematics.cpp)
add_executable(mtm_haptics_teleop src/mtm_haptics_teleop.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(force_filter
   ${catkin_LIBRARIES}
)
target_link_libraries(test_mtm_haptics
   ${catkin_LIBRARIES}
)
target_link_libraries(mtm_haptics_teleop
   ${catkin_LIBRARIES}
)
cisst_target_link_libraries (test_mtm_haptics ${REQUIRED_CISST_LIBRARIES})
cisst_target_link_libraries (mtm_haptics_teleop ${REQUIRED_CISST_LIBRARIES})

