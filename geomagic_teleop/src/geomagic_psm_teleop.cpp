#include "geomagic_teleop/geomagic_psm_teleop.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geomagic_teleop/geomagic_psm_teleop.h"

Geomagic_Teleop::Geomagic_Teleop(boost::shared_ptr<ros::NodeHandle> node):
    _node(node)
{
    _node->param(std::string("device_name"), _device_name, std::string("Geomagic"));
    _node->param(std::string("arm"), _slave_name, std::string("PSM1"));
    _node->param(std::string("rate"), _pub_rate, 1000);

    rate.reset(new ros::Rate(_pub_rate));
    arm_psm.reset(new DVRK_Arm(_slave_name));
    cur_gFrame.reset(new Frame);
    pre_gFrame.reset(new Frame);
    rotPsm2Geo.setRPY(-M_PI, 0, M_PI/2);

    geomagic_joy_sub = _node->subscribe("/" + _device_name + "/joy",10,&Geomagic_Teleop::geomagic_joy_cb, this);
    geomagic_pose_sub = _node->subscribe("/" + _device_name + "/pose",10,&Geomagic_Teleop::geomagic_pose_cb, this);
    geomagic_force_pub = _node->advertise<geomagic_control::OmniFeedback>("/" + _device_name + "/force_feedback",10);

    geomagic_joy_cmd.axes.resize(6);
    geomagic_joy_cur.axes.resize(6);
    geomagic_joy_pre.axes.resize(6);
    geomagic_joy_cur.buttons.resize(2);
    omni_feedback.lock.resize(3);
    _clutch = false; _coag = false; _first_trigger = false;
    ros::spinOnce();
    arm_psm->set_mode(arm_psm->_m_cart_pos_mode);
    sleep(1);
    scale = 0.01;
    align_end_effectors();
}

void Geomagic_Teleop::align_end_effectors(){
    ros::spinOnce();
    tf::Quaternion rot_psm;
    arm_psm->get_cur_orientation(rot_psm);
    rotPsm2Geo = cur_gFrame->rot_quat.inverse() * rot_psm;
//    tf::Transform psm_trans;
//    tf::Quaternion psm_rot, geo_rot, affix_psm_rot;
//    arm_psm->get_cur_transform(psm_trans);
//    arm_psm->set_origin_frame(psm_trans);
//    arm_psm->get_cur_orientation(psm_rot);
//    tf::quaternionMsgToTF(geomagic_pose_cur.orientation, geo_rot);
//    affix_psm_rot = psm_rot.inverse() * geo_rot;
//    if(std::abs(affix_psm_rot.length() - 1) < 1e-6 ){
//        arm_psm->affix_tip_frame_rot(affix_psm_rot);
////        arm_psm->get_cur_transform(psm_trans);
////        arm_psm->set_transform(psm_trans);
//    }
//    else{
//        ROS_ERROR("GEO ROT NON VALID TO AFFIX TO PSM FRAME %f", affix_psm_rot.length());
//    }
}

void Geomagic_Teleop::PosetoFrame(const geometry_msgs::Pose &pose, FramePtr frame){
    frame->trans.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    frame->trans.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    frame->pos = frame->trans.getOrigin();
    frame->rot_quat = frame->trans.getRotation();
    frame->rot_mat.setRotation(frame->rot_quat);
}

void Geomagic_Teleop::geomagic_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    PosetoFrame(geomagic_pose_cur, pre_gFrame);
    geomagic_pose_pre = geomagic_pose_cur;
    geomagic_pose_cur = msg->pose;
    PosetoFrame(geomagic_pose_cur, cur_gFrame);

}

void Geomagic_Teleop::geomagic_joy_cb(const sensor_msgs::JoyConstPtr msg){
        geomagic_joy_pre = geomagic_joy_cur;
        geomagic_joy_cur = *msg;
        _coag = geomagic_joy_cur.buttons[0];
        _clutch = geomagic_joy_cur.buttons[1];
}


void Geomagic_Teleop::dead_band(sensor_msgs::Joy &msg){
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

void Geomagic_Teleop::clip(sensor_msgs::Joy &msg){
    for(unsigned int i = 0 ; i<msg.axes.size() ; i++){
        msg.axes[i] = (double)((int)(msg.axes[i] * precision))/precision;
    }
}

void Geomagic_Teleop::run(){
    tf::Transform psmTrans;
    arm_psm->get_cur_transform(psmTrans);
    psmTrans.setOrigin(psmTrans.getOrigin() + scale*(cur_gFrame->trans.getOrigin() - pre_gFrame->trans.getOrigin()));
    psmTrans.setRotation(cur_gFrame->trans.getRotation() * rotPsm2Geo);

    if(_clutch || _coag){
        if(_first_trigger == false){
            _first_trigger = true;
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
        if(_coag){
            arm_psm->set_transform(psmTrans);
        }
    }
    else{
        _first_trigger = false;
        align_end_effectors();
    }
    geomagic_force_pub.publish(omni_feedback);
    ros::spinOnce();
    rate->sleep();
}

Geomagic_Teleop::~Geomagic_Teleop(){

}


int main(int argc, char **argv){

    ros::init(argc, argv, "geomagic_teleop");
    boost::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle);
    Geomagic_Teleop geo_teleop(node);

    while(node->ok()){
        geo_teleop.run();
    }
    return 0;
}
