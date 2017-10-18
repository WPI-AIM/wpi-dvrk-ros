# Description
This package is a wrapper for easy integration of dVRK manipulators without having to remake 
the topics and publishers from scratch. Functions are overloaded to provide seamless integration 
from many different data types. The library is designed to be used with non-ROS programs as well.

## Author
Adnan Munawar: amunawar@wpi.edu

## Dependencies
Standalone, however, you do need a working **dvrk-ros** for anything useful from this package

## Usage

### Adding library to CMakeLists File
    find_package(catkin REQUIRED COMPONENTS dvrk_arm)
    include_directories(${catkin_INCLUDE_DIRS})
    ...
### Origin and Tip Tranfroms
Keeping in mind the intended use of the library for different applications and UI devices, we can 
set the **Origin Transform**, such that all the **End Effector Transforms** (+ Position and Orientaiton, Forces and Moments)
using the numerous overloaded functions are w.r.t the defined w.r.t Origin Transfrom. 

Not only that, one might wants the self defined directional coordinates of the End Effector (i.e. where should the x,y and z unit vectors of the tip be pointing). For this purpose, the Tip Tranform can be set places another transform on the tip and form here on, each Tranfrom component is reported w.r.t to the tip tranfrom.

When setting the positions or forces on the dvrk Manipulators, the frames are already handeled so you just specify
the values w.r.t to the frames you set.

### Using the library (Example)
    DVRK_Arm arm("MTMR");
    sleep(1);
    arm.set_mode(arm._m_effort_mode);
    sleep(1);
    arm.set_force(1,0,0);
    double x,y,z;
    arm.get_position(x,y,z);
    arm.set_mode(arm._m_cart_pos_mode);
    sleep(1);
    arm.set_position(-0.5,-0.25,-0.3);
    
    //Check if arm is available
    
    bool check = arm._is_available();
