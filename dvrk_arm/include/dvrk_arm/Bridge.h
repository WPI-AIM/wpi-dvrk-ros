#ifndef CDVRK_BRIDGEH
#define CDVRK_BRIDGEH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Wrench.h"
#include "FootPedals.h"
#include "Console.h"
#include "string.h"
#include <functional>

class DVRK_Arm;

class DVRK_Bridge: public DVRK_FootPedals{

    friend class DVRK_Arm;
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name);
    ~DVRK_Bridge();
    void _rate_sleep();

    void (DVRK_Arm::*my_func)(const geometry_msgs::PoseStamped &pose) = NULL;

    template <class T, class U>
    void assign_conversion_fcn(void (T::*conversion_fcn)(U), T *obj);
    DVRK_Arm *my_obj;

private:
    std::string arm_name;

    ros::NodeHandle *n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_safety_pub;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
    ros::Publisher joint_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Rate *rate;
    double scale;
    bool _is_cnvFcn_set;
    std::vector<std::string> valid_arms;
    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state, state_cmd;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;
};

template <class T, class U>
void DVRK_Bridge::assign_conversion_fcn(void (T::*conversion_fcn)(U), T *obj){
    //cnv_fcn = conversion_fcn;
    //my_func = boost::bind(conversion_fcn, obj);
    my_func = (conversion_fcn);
    my_obj = obj;
    _is_cnvFcn_set = true;
}

#endif
