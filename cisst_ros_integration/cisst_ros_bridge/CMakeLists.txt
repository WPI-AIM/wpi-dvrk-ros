cmake_minimum_required(VERSION 2.8.3)
project(cisst_ros_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cisst_msgs
  )

set (REQUIRED_CISST_LIBRARIES 
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cisst_ros_bridge
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs cisst_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# header
set (HEADER_FILES
  include/cisst_ros_bridge/mtsROSBridge.h
  include/cisst_ros_bridge/mtsCISSTToROS.h
  include/cisst_ros_bridge/mtsROSToCISST.h)

# source
set (SOURCE_FILES
  src/mtsROSBridge.cpp
  src/mtsCISSTToROS.cpp
  src/mtsROSToCISST.cpp)

add_library(cisst_ros_bridge ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(cisst_ros_bridge ${catkin_LIBRARIES})
cisst_target_link_libraries (cisst_ros_bridge ${REQUIRED_CISST_LIBRARIES})

## Declare a cpp executable
add_executable(example_bridge example/main.cpp)
target_link_libraries (example_bridge cisst_ros_bridge ${catkin_LIBRARIES})
cisst_target_link_libraries (example_bridge ${REQUIRED_CISST_LIBRARIES})

#############
## Install ##
#############

install(TARGETS cisst_ros_bridge example_bridge
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

