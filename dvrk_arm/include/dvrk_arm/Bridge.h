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

struct States{
public:
    States(){
        mStates[DVRK_UNINITIALIZED] = "DVRK_UNINITIALIZED";
        mStates[DVRK_POSITION_JOINT]= "DVRK_POSITION_JOINT";
        mStates[DVRK_POSITION_CARTESIAN] = "DVRK_POSITION_CARTESIAN";
        mStates[DVRK_EFFORT_CARTESIAN] = "DVRK_EFFORT_CARTESIAN";


        _m_effort_mode = mStates[DVRK_EFFORT_CARTESIAN];
        _m_jnt_pos_mode = mStates[DVRK_POSITION_JOINT];
        _m_cart_pos_mode = mStates[DVRK_POSITION_CARTESIAN];

        activeState = DVRK_UNINITIALIZED;
    }

    enum eSTATES{DVRK_UNINITIALIZED,
                DVRK_POSITION_CARTESIAN,
                DVRK_POSITION_JOINT,
                DVRK_EFFORT_CARTESIAN};
    eSTATES activeState;

    std::map<eSTATES, std::string> mStates;

    std::string _m_effort_mode;
    std::string _m_jnt_pos_mode;
    std::string _m_cart_pos_mode;
};

class DVRK_Bridge: public States, public DVRK_FootPedals{
public:
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name);
    ~DVRK_Bridge();

    void _rate_sleep();

    template <class T, class U>
    void assign_conversion_fcn(void (T::*conversion_fcn)(U), T *obj);
    void set_cur_pose(const geometry_msgs::PoseStamped &pose);
    void set_cur_wrench(const geometry_msgs::Wrench &wrench);
    void set_cur_joint(const sensor_msgs::JointState &jnt_state);
    void set_cur_mode(const std::string &state, bool lock_ori);

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    bool _start_pubs;

private:
    std::string arm_name;

    ros::NodeHandle *n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_lock_pub;
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
    sensor_msgs::JointState cur_joint, pre_joint, cmd_joint;
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
