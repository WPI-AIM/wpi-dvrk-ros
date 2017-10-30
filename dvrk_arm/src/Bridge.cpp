#include "dvrk_arm/Bridge.h"

DVRK_Bridge::DVRK_Bridge(const std::string &arm_name, int bridge_frequency): _freq(bridge_frequency){
    valid_arms.push_back("MTML");
    valid_arms.push_back("MTMR");
    valid_arms.push_back("PSM1");
    valid_arms.push_back("PSM2");
    valid_arms.push_back("PSM3");

    bool _valid_arm = false;

    for(size_t i = 0; i < valid_arms.size(); i ++){
        if (strcmp(arm_name.c_str(), valid_arms[i].c_str()) == 0){
            this->arm_name = valid_arms[i];
           _valid_arm = true;
        }
    }

    if(_valid_arm){
        ROS_INFO("Specified arm is %s:", arm_name.c_str());
        init();
    }
    else{
        ROS_ERROR("%s Invalid Arm Specified", arm_name.c_str());
    }
}

void DVRK_Bridge::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, arm_name + "_interface_node");
    n.reset(new ros::NodeHandle);
    nTimer.reset(new ros::NodeHandle);
    n->setCallbackQueue(&cb_queue);
    nTimer->setCallbackQueue(&cb_queue_timer);
    rate.reset(new ros::Rate(1000));
    timer = nTimer->createTimer(ros::Duration(1/(double)_freq), &DVRK_Bridge::timer_cb, this);
    aspin.reset(new ros::AsyncSpinner(0, &cb_queue_timer));

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_Bridge::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_Bridge::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_Bridge::joint_sub_cb, this);
    wrench_sub = n->subscribe("/dvrk/" + arm_name + "/wrench_body_current", 10, &DVRK_Bridge::wrench_sub_cb, this);

    joint_pub = n->advertise<sensor_msgs::JointState>("/dvrk/" + arm_name + "/set_position_joint", 10);
    pose_pub  = n->advertise<geometry_msgs::Pose>("/dvrk/" + arm_name + "/set_position_cartesian", 10);
    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_lock_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);

    activeState = DVRK_UNINITIALIZED;

    cmd_pose.pose.position.x = 0; cmd_pose.pose.position.y = 0; cmd_pose.pose.position.z = 0;
    cmd_pose.pose.orientation.x = 0; cmd_pose.pose.orientation.y = 0; cmd_pose.pose.orientation.z = 0; cmd_pose.pose.orientation.w = 1;
    cmd_wrench.wrench.force.x = 0; cmd_wrench.wrench.force.y = 0; cmd_wrench.wrench.force.z = 0;
    cmd_wrench.wrench.torque.x = 0; cmd_wrench.wrench.torque.y = 0; cmd_wrench.wrench.torque.z = 0;

    DVRK_FootPedals::init(n);
    _is_pose_cnvFcn_set = false; _is_joint_cnvFcn_set = false; _is_wrench_cnvFcn_set = false;
    _start_pubs = false;
    sleep(1);
    aspin->start();
    scale = 0.1;
}

void DVRK_Bridge::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
    if(_is_joint_cnvFcn_set){
        conversion_function_joint(cur_joint);
    }
}

void DVRK_Bridge::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;
    if(_is_pose_cnvFcn_set){
        conversion_function_pose(cur_pose);
    }
}

void DVRK_Bridge::wrench_sub_cb(const geometry_msgs::WrenchStampedConstPtr &msg){
    cur_wrench = *msg;
    if(_is_wrench_cnvFcn_set){
        conversion_function_wrench(cur_wrench);
    }
}

void DVRK_Bridge::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
    for(std::map<eSTATES, std::string>::iterator it = mStates.begin(); it != mStates.end() ; ++it){
        if(strcmp(cur_state.data.c_str(), it->second.c_str()) == 0){
            activeState = it->first;
        }
    }
}

void DVRK_Bridge::timer_cb(const ros::TimerEvent& event){
    cb_queue.callAvailable();
    if(_start_pubs == true){
        switch (activeState) {
        case DVRK_POSITION_JOINT:
            joint_pub.publish(cmd_joint);
            break;
        case DVRK_POSITION_CARTESIAN:
            pose_pub.publish(cmd_pose.pose);
            break;
        case DVRK_EFFORT_CARTESIAN:
            force_pub.publish(cmd_wrench.wrench);
            break;
        default:
            break;
        }
    }
}

void DVRK_Bridge::set_cur_mode(const std::string &state, bool lock_ori){
    for(std::map<eSTATES, std::string>::iterator it = mStates.begin(); it != mStates.end() ; ++it){
        if(strcmp(state.c_str(), it->second.c_str()) == 0){
            state_cmd.data = state;
            state_pub.publish(state_cmd);
            if(it->first == DVRK_EFFORT_CARTESIAN){
                std_msgs::Bool lock;
                lock.data = lock_ori;
                force_orientation_lock_pub.publish(lock);
            }
            _rate_sleep();
        }
    }
    _start_pubs = false;
}

void DVRK_Bridge::set_cur_pose(const geometry_msgs::PoseStamped &pose){
    cmd_pose = pose;
    _start_pubs = true;
}

void DVRK_Bridge::set_cur_wrench(const geometry_msgs::Wrench &wrench){
    cmd_wrench.wrench = wrench;
    _start_pubs = true;
}

void DVRK_Bridge::set_cur_joint(const sensor_msgs::JointState &jnt_state){
    cmd_joint = jnt_state;
    _start_pubs = true;
}

void DVRK_Bridge::_rate_sleep(){
    rate->sleep();
}

bool DVRK_Bridge::_is_available(){
    if (state_pub.getNumSubscribers() > 0 && state_sub.getNumPublishers() > 0 && pose_sub.getNumPublishers() > 0){
        // If there are listeners to the state_publisher and pose publisher and subscribers to state msg, most likely the Arm is available
        // Doing 3 seperate topic checks for redundancy
        return true;
    }
    else{
        return false;
    }

}

bool DVRK_Bridge::_in_effort_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_effort_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::_in_cart_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_cart_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::_in_jnt_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_jnt_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::shutDown(){
    ROS_WARN("Shutdown called");
    cb_queue.clear();
    aspin->stop();
    ros::shutdown();
    return true;
}

DVRK_Bridge::~DVRK_Bridge(){
}


