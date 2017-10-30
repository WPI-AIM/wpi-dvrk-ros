#ifndef CDVRK_BRIDGEH
#define CDVRK_BRIDGEH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/WrenchStamped.h"
#include "FootPedals.h"
#include "Console.h"
#include "string.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "ros/callback_queue.h"
#include "dvrk_arm/States.h"
#include "Conversion.h"

class DVRK_Bridge: public States, public DVRK_FootPedals{
public:
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name, int bridge_frequnce = 1000);
    ~DVRK_Bridge();

    void set_cur_pose(const geometry_msgs::PoseStamped &pose);
    void set_cur_wrench(const geometry_msgs::Wrench &wrench);
    void set_cur_joint(const sensor_msgs::JointState &jnt_state);
    void set_cur_mode(const std::string &state, bool lock_ori);

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    bool _start_pubs;

    typedef boost::shared_ptr<ros::NodeHandle> NodePtr;
    typedef boost::shared_ptr<ros::Rate> RatePtr;
    typedef boost::shared_ptr<ros::AsyncSpinner> AspinPtr;

    bool shutDown();

    Conversion<const geometry_msgs::PoseStamped&> poseConversion;
    Conversion<const sensor_msgs::JointState&> jointConversion;
    Conversion<const geometry_msgs::WrenchStamped&> wrenchConversion;

private:
    std::string arm_name;

    NodePtr n, nTimer;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_lock_pub;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
    ros::Publisher joint_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Subscriber wrench_sub;
    ros::CallbackQueue cb_queue, cb_queue_timer;
    ros::Timer timer;
    AspinPtr aspin;
    RatePtr rate;
    int _freq;

    double scale;
    std::vector<std::string> valid_arms;
    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void wrench_sub_cb(const geometry_msgs::WrenchStampedConstPtr &wrench);
    void timer_cb(const ros::TimerEvent&);
    void _rate_sleep();

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint, cmd_joint;
    std_msgs::String cur_state, state_cmd;
    geometry_msgs::WrenchStamped cur_wrench, cmd_wrench;
};

#endif
