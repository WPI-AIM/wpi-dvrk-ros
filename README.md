# Author: Adnan Munawar
## AIM Labs, Worcester Polytechnic Institute
### amunawar@wpi.edu

### Description: ###
This repository contains code for Motion Planning Interfaces, Haptics, and Semi-Virtual TeleOperations of the dVRK components with the actual components.

### Pre-Requisites: ###
These packages require the installation of the dvrk-ros packages from here:

https://github.com/WPI-AIM/dvrk-ros.git

Additionally, the installation of cisst-saw packages is also required from here

https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros


### Packages: ###
**dvrk_trajectory:** Contains Code for MTM Pose capture and simulation in RViz. Also containts code for capturing footpedal tray events

**kinematics:** Contains code for MTM and PSM motion planning tasks, collision detection of PSM in simulation and Virtual Teleoperation of PSM in simulation using actual MTM

**mtm_haptics:** Contains code for MTM Haptic Feedback (In development phase)

**mtmr_rviz:** MoveIt interfaces for MTMR

**psm1_rviz:** MoveIt interfaces for PSM1
