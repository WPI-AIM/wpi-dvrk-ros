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
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "ros/callback_queue.h"

class DVRK_Bridge: public DVRK_FootPedals{
public:
    friend class DVRK_Arm;
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name);
    ~DVRK_Bridge();
    void _rate_sleep();

    template <class T, class U>
    void assign_conversion_fcn(void (T::*conversion_fcn)(U), T *obj);

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
    ros::CallbackQueue cb_queue;
    ros::Timer timer;
    ros::AsyncSpinner* aspin;

    ros::Rate *rate;
    double scale;
    bool _is_cnvFcn_set;
    std::vector<std::string> valid_arms;
    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void timer_cb(const ros::TimerEvent&);

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state, state_cmd;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;

    boost::function<void (const geometry_msgs::PoseStamped &pose)> conversion_function;
};

template <class T, class U>
void DVRK_Bridge::assign_conversion_fcn(void (T::*conversion_fcn)(U), T *obj){
    conversion_function = boost::bind(conversion_fcn, obj, _1);
    _is_cnvFcn_set = true;
}

#endif
