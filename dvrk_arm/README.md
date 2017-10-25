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

### Conversion Function
The class **Dvrk_Bridge** handles all the ros-communication. The **cartesian pose callback** from the **cisst-saw** is reported via **geometry_msgs/PoseStamped** msg. The class **Dvrk_Bridge** accepts a conversion callback *member-function* that takes in **const geometry_msgs/PoseStamped &** as an argument as does all the conversion inside of it. 

    assign_conversion_fcn(&DVRK_Arm::cisstPose_to_userTransform, this);
    
#### IMPORTANT
While setting just the position of **Origin Tranfrom** or **Tip Transform**, the orientation is not altered, it remains whatever it was set before, or if it wasn't set before, it remains identity matrix. Likewise, when the only the orientation is set, the position remains un-altered.

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
    
### Setting Origin and Tip Frames
    DVRK_Arm arm("MTML");
    // Lets say we want to know what the current Transform between tip and origin is
    tf::Transform ee_tran_cur;
    arm.get_cur_trans(ee_tran_cur);
    // Now, we might want the origin trans to be placed at the tip, so all position and angular
    // offsets are zero. Just take the inverse of the cur_trans and set it as origin trans;
    arm.set_origin_frame(ee_tran_cur.inverse());
    // Now, lets say, we want the EE trans to be orientated differently.
    tf::Quaternion tip_quat;
    tip_quat.setRPY(0, M_PI/2, 0);
    arm.affix_tip_frame_rot(tip_quat);
    // We can also move the tip frame anywhere we want.
    arm.affix_tip_frame_pos(-1,0,2);
