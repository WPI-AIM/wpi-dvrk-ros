#ifndef CGEOMAGIC_TELEOP_H
#define CGEOMAGIC_TELEOP_H
#include "geomagic_teleop/geomagic_psm_teleop.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Transform.h"
#include <string>
#include <geomagic_control/OmniFeedback.h>
#include <ros/package.h>
#include <dvrk_arm/Arm.h>
#include <dvrk_arm/Frame.h>

typedef boost::shared_ptr<Frame> FramePtr;

class Geomagic_Teleop{
public:
    Geomagic_Teleop(boost::shared_ptr<ros::NodeHandle> node);
    ~Geomagic_Teleop();

    geomagic_control::OmniFeedback omni_feedback;
    sensor_msgs::Joy geomagic_joy_cur, geomagic_joy_pre, geomagic_joy_cmd;
    geometry_msgs::Pose geomagic_pose_cur, geomagic_pose_pre, geomagic_pose_cmd;

    ros::Subscriber geomagic_joy_sub, geomagic_pose_sub;
    ros::Publisher  geomagic_force_pub;
    boost::shared_ptr<ros::NodeHandle> _node;
    std::string _slave_name, _device_name;
    boost::shared_ptr<ros::Rate> rate;
    int _pub_rate;

    tf::Quaternion R_geoTopsm;
    tf::Matrix3x3 mat_geoTopsm;

    double db = 0.0001;
    double precision = 1000;
    double scale = 0.01;
    bool _clutch, _coag, _first_trigger;

    void geomagic_pose_cb(const geometry_msgs::PoseStampedConstPtr msg);
    void geomagic_joy_cb(const sensor_msgs::JoyConstPtr msg);
    void dead_band(sensor_msgs::Joy &msg);
    void clip(sensor_msgs::Joy &msg);
    void PosetoFrame(const geometry_msgs::Pose &msg, FramePtr frame);
    void align_end_effectors();
    void run();

    boost::shared_ptr<DVRK_Arm> arm_psm;
    FramePtr cur_gFrame, pre_gFrame; //Current Frame and previous Frame
    tf::Quaternion rotGeo2Psm;
};


#endif
