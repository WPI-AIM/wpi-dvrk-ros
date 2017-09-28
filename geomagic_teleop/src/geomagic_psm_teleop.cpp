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

geometry_msgs::Pose psm_pose_cur;
sensor_msgs::Joy geomagic_joy_cur;
sensor_msgs::Joy geomagic_joy_pre;
sensor_msgs::Joy geomagic_joy_cmd;
geometry_msgs::Pose psm_pose_cmd;
geometry_msgs::Pose geomagic_pose_cur;
sensor_msgs::JointState psm_jnt_cur;


std_msgs::String psm_state_cur;
geomagic_control::OmniFeedback omni_feedback;

bool _coag_pressed = false;
bool _clutch_pressed = false;


double db = 0.0001;
double precision = 1000;
double scale = 0.01;


void psm_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    psm_pose_cur = msg->pose;
}

void psm_jnt_cb(const sensor_msgs::JointStatePtr msg){
}

void geomagic_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    geomagic_pose_cur = msg->pose;
}

void geomagic_joy_cb(const sensor_msgs::JoyConstPtr msg){
        geomagic_joy_pre = geomagic_joy_cur;
        geomagic_joy_cur = *msg;
}

void psm_state_cb(const std_msgs::StringConstPtr msg){
    psm_state_cur = *msg;
}

void dead_band(sensor_msgs::Joy &msg){
    for(unsigned int i = 0 ; i<msg.axes.size() ; i++){
        double abs_pos = std::abs(msg.axes[i]);
        if(abs_pos < db){
            msg.axes[i] = 0;
        }
        else{
            msg.axes[i] = msg.axes[i] - ((abs_pos/msg.axes[i]) * db);
        }
    }
}

void clip(sensor_msgs::Joy &msg){
    for(unsigned int i = 0 ; i<msg.axes.size() ; i++){
        msg.axes[i] = (double)((int)(msg.axes[i] * precision))/precision;
    }
}



int main(int argc, char **argv){

    ros::init(argc, argv, "geomagic_teleop");
    ros::NodeHandle node;
    std::string slave_name= "";
    std::string device_name= "";
    int pub_rate;
    node.param(std::string("arm"), slave_name, std::string("PSM1"));
    node.param(std::string("rate"), pub_rate, 1000);
    node.param(std::string("device_name"), device_name, std::string("Geomagic"));
    ros::Rate rate(pub_rate);
    tf::Vector3 pos_geomagic, pos_psm;
    tf::Transform trans_geomagic, trans_psm;
    tf::TransformBroadcaster t_br;

    std::string req_state = "DVRK_POSITION_CARTESIAN";


    ros::Subscriber psm_pose_sub;
    ros::Subscriber geomagic_joy_sub, geomagic_pose_sub;
    ros::Subscriber psm_state_sub;
    ros::Subscriber psm_jnt_state_sub;
    ros::Publisher  psm_teleop;
    ros::Publisher  psm_state_pub;
    ros::Publisher  geomagic_force_pub;

    tf::Quaternion R_geoTopsm;
    tf::Matrix3x3 mat_geoTopsm;

    // initialize joint current/command/msg_js
    psm_jnt_cur.position.resize(7);

    psm_pose_sub = node.subscribe("dvrk/" + slave_name + "/position_cartesian_current",10,psm_pose_cb);
    psm_state_sub = node.subscribe("dvrk/" + slave_name + "/robot_state",10,psm_state_cb);
    psm_jnt_state_sub = node.subscribe("dvrk/" + slave_name + "/state_joint_current",10,psm_jnt_cb);
    psm_teleop = node.advertise<geometry_msgs::Pose>("/dvrk/" + slave_name + "/set_position_cartesian",10);
    psm_state_pub = node.advertise<std_msgs::String>("/dvrk/" + slave_name + "/set_robot_state",10);

    geomagic_joy_sub = node.subscribe("/" + device_name + "/joy",10,geomagic_joy_cb);
    geomagic_pose_sub = node.subscribe("/" + device_name + "/pose",10,geomagic_pose_cb);
    geomagic_force_pub = node.advertise<geomagic_control::OmniFeedback>("/" + device_name + "/force_feedback",10);

    std_msgs::String state_msg;
    state_msg.data = req_state.c_str();
    ROS_INFO("Setting %s to %s state", slave_name.c_str(), req_state.c_str());
    psm_state_pub.publish(state_msg);
    ros::spinOnce();
    rate.sleep();


    geomagic_joy_cmd.axes.resize(6);
    geomagic_joy_cur.axes.resize(6);
    geomagic_joy_pre.axes.resize(6);
    geomagic_joy_cur.buttons.resize(2);
    omni_feedback.lock.resize(3);

    mat_geoTopsm.setValue(0 , 1 , 0,
                          1 , 0 , 0,
                          0 , 0 ,-1);
    mat_geoTopsm.getRotation(R_geoTopsm);

    sleep(2);



    int counter = 0;

    while(node.ok()){
        if (!strcmp(psm_state_cur.data.c_str(), req_state.c_str()))
        {
            geomagic_joy_cmd.axes[0] = geomagic_joy_cur.axes[0] - geomagic_joy_pre.axes[0];
            geomagic_joy_cmd.axes[1] = geomagic_joy_cur.axes[1] - geomagic_joy_pre.axes[1];
            geomagic_joy_cmd.axes[2] = geomagic_joy_cur.axes[2] - geomagic_joy_pre.axes[2];

            //geomagic_joy_cmd.axes[0] = (geomagic_joy_cmd.axes[0]*precision);
            //geomagic_joy_cmd.axes[0] = geomagic_joy_cmd.axes[0]/precision;
            //dead_band(geomagic_joy_cmd);
            //clip(geomagic_joy_cmd);
            pos_geomagic.setX(-geomagic_joy_cur.axes[0] * scale);
            pos_geomagic.setY(geomagic_joy_cur.axes[2] * scale);
            pos_geomagic.setZ(geomagic_joy_cur.axes[1] * scale);


            trans_geomagic.setOrigin(pos_geomagic);
            tf::Quaternion geo_rot, quat;
            geo_rot.setRPY(geomagic_joy_cur.axes[3], geomagic_joy_cur.axes[4], geomagic_joy_cur.axes[5]);
            trans_geomagic.setRotation(geo_rot);

            psm_pose_cmd.position.x = psm_pose_cur.position.x + (scale * geomagic_joy_cmd.axes[0]);
            psm_pose_cmd.position.y = psm_pose_cur.position.y + (scale * geomagic_joy_cmd.axes[1]);
            psm_pose_cmd.position.z = psm_pose_cur.position.z + (scale * geomagic_joy_cmd.axes[2]);


            quat = geo_rot * R_geoTopsm;
            psm_pose_cmd.orientation.x = quat.getX();
            psm_pose_cmd.orientation.y = quat.getY();
            psm_pose_cmd.orientation.z = quat.getZ();
            psm_pose_cmd.orientation.w = quat.getW();

            pos_psm.setX(psm_pose_cmd.position.x);
            pos_psm.setY(psm_pose_cmd.position.y);
            pos_psm.setZ(psm_pose_cmd.position.z);
            trans_psm.setOrigin(pos_psm);
            trans_psm.setRotation(quat);


            // Check if COAG is pressed. Grey Button

            if(geomagic_joy_cur.buttons[0] == 1){
                if(_coag_pressed == false){
                    ROS_INFO("Coag Pressed");
                    _coag_pressed = true;
                    for(u_int i = 0 ; i < 3 ; i++){
                        omni_feedback.lock[i] = false;
                    }
                }
                else{
                    omni_feedback.position.x = geomagic_joy_cur.axes[0];
                    omni_feedback.position.y = geomagic_joy_cur.axes[1];
                    omni_feedback.position.z = geomagic_joy_cur.axes[2];

                    omni_feedback.force.x = 0;
                    omni_feedback.force.y = 0;
                    omni_feedback.force.z = 0;

                    for(u_int i = 0 ; i < 3 ; i++){
                        omni_feedback.lock[i] = true;
                    }
                }
                psm_teleop.publish(psm_pose_cmd);
            }
            else{
                _coag_pressed = false;
                tf::Quaternion temp_quat;
                temp_quat.setX(psm_pose_cur.orientation.x);
                temp_quat.setY(psm_pose_cur.orientation.y);
                temp_quat.setZ(psm_pose_cur.orientation.z);
                temp_quat.setW(psm_pose_cur.orientation.w);

                R_geoTopsm = geo_rot.inverse() * temp_quat;
            }

            // Check if Clutch is Pressed. White Button
            if(geomagic_joy_cur.buttons[1] == 1){
                if(_clutch_pressed == false){
                    ROS_INFO("Clutch Pressed");
                    _clutch_pressed = true;

                    for(u_int i = 0 ; i < 3 ; i++){
                        omni_feedback.lock[i] = false;
                    }
                }
                else{
                    omni_feedback.position.x = geomagic_joy_cur.axes[0];
                    omni_feedback.position.y = geomagic_joy_cur.axes[1];
                    omni_feedback.position.z = geomagic_joy_cur.axes[2];

                    omni_feedback.force.x = 0;
                    omni_feedback.force.y = 0;
                    omni_feedback.force.z = 0;

                    for(u_int i = 0 ; i < 3 ; i++){
                        omni_feedback.lock[i] = true;
                    }
                }
            }
            else{
                _clutch_pressed = false;
            }

            geomagic_force_pub.publish(omni_feedback);
            t_br.sendTransform(tf::StampedTransform(trans_geomagic,ros::Time::now(),"/world","/geomagic_position"));
            t_br.sendTransform(tf::StampedTransform(trans_psm,ros::Time::now(),"/world","/teleoped_psm"));

        }
        else{
            ROS_INFO("DVRK %s in %s, CURRENT STATE IS %s ", slave_name.c_str(), req_state.c_str(), psm_state_cur.data.c_str());
            psm_state_pub.publish(state_msg);
            sleep(2);
        }
        counter++;
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
