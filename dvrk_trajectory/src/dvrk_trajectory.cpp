#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/JointState.h>
#include<vector>
#include<std_msgs/Bool.h>
#include<std_msgs/Empty.h>
#include<iostream>

class Traj
{
public:
    Traj();
    void pose_cb(const geometry_msgs::PoseConstPtr & pose);
    void coag_cb(const std_msgs::BoolConstPtr & state);
    void jointstate_cb(const sensor_msgs::JointStateConstPtr & js);
    void clutch_cb(const std_msgs::BoolConstPtr & state);

protected:

    std::size_t quene_size;

    std::vector<geometry_msgs::Pose> dvrk_pose;
    std::vector<sensor_msgs::JointState> dvrk_js;
    std::vector<geometry_msgs::Pose> dvrk_traj_pose;
    std::vector<sensor_msgs::JointState> dvrk_traj_js;

    ros::NodeHandle node;
    ros::Rate *rate;

    ros::Publisher traj_pub;
    ros::Subscriber caog_sub;
//    ros::Publisher traj_length_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber js_sub;
    ros::Subscriber clutch_sub;
};

Traj::Traj()
{
    this->quene_size = 50;
    this->rate = new ros::Rate(1000);
    this->caog_sub = node.subscribe("/dvrk_footpedal/coag_state",1000, &Traj::coag_cb, this);
    this->pose_sub = node.subscribe("/dvrk_mtm/cartesian_pose_current",1000, &Traj::pose_cb,this);
    this->js_sub = node.subscribe("/dvrk_mtm/joint_position_current",1000,&Traj::jointstate_cb,this);
    this->traj_pub = node.advertise<geometry_msgs::Pose>("/mtm/trajectory_poses",1);
//    this->traj_length_pub = node.advertise<std_msgs::UInt64ConstPtr>("/mtm/trajectory_poses_size",1);
    this->clutch_sub = node.subscribe("/dvrk_footpedal/clutch_state",1000,&Traj::clutch_cb,this);
}

void Traj::pose_cb(const geometry_msgs::PoseConstPtr & pose)
{
    if (this->dvrk_pose.size() >= quene_size){
        this->dvrk_pose.clear();
    }
    this->dvrk_pose.push_back(*pose.get());
}

void Traj::jointstate_cb(const sensor_msgs::JointStateConstPtr &js)
{
    if (this->dvrk_pose.size() >= quene_size){
        this->dvrk_pose.clear();
    }
    this->dvrk_js.push_back(*js.get());
}

void Traj::coag_cb(const std_msgs::BoolConstPtr & state)
{
    if (state->data == true){
        if(this->dvrk_pose.size() > 0)
        {
            this->dvrk_traj_pose.push_back(this->dvrk_pose.back());
            this->dvrk_traj_js.push_back(this->dvrk_js.back());
            ROS_INFO("Catching Pose: Size of Trajectory: %lu Poses", dvrk_traj_pose.size());
        }
        else{
            ROS_ERROR("dvrk_traj vector's size is 0, something is wrong");
        }
    }
}

void Traj::clutch_cb(const std_msgs::BoolConstPtr & state)
{
    if( state->data == true)
    {
    if (this->dvrk_traj_pose.size()>0){
        ROS_INFO("Clutch Pressed: Publishing %lu Trajectory Poses",dvrk_traj_pose.size());
        for(size_t i=0 ; i < dvrk_traj_pose.size() ; i++)
        {
            this->traj_pub.publish(dvrk_traj_pose.at(i));
            ros::spinOnce();
            this->rate->sleep();
        }
    }
    else{
        ROS_INFO("No Poses captured yet, capture MTM poses first by pressing COAG/MONO");
    }
    }

}




int main(int argc,char **argv )
{
    ros::init(argc,argv,"dvrk_trajectory_node");
    Traj dvrk;
    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
