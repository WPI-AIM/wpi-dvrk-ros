dVRK Interface for CHAI-3D
====================
This package enables support for dVRK MTMs in CHAI-3D 

# Install 
* download and compile cisst-saw from
  https://github.com/jhu-cisst/cisst-saw
* download and compile dvrk-ros from
  https://github.com/jhu-dvrk/dvrk-ros
* download and compile CHAI-3D from 
  https://github.com/adnanmunawar/chai3d.git
* download and compile this package. CHAI-3D will use the library from this package for MTM interface

# Run
* run dvrk_console application for MTMR for now.
* Home the MTM.
* Set the MTM in **DVRK_EFFORT_CARTESIAN** mode.
* Set the topic **set_wrench_body_orientation_absolute** to **True**.
* Launch chai multi-device demo to see if the mtm device is available.
* If it is, launch any CHAI-3D demo. 

