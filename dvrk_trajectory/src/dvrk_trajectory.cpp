#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/JointState.h>
#include<vector>
#include<std_msgs/Bool.h>
#include<iostream>

class Traj
{
public:
    void pose_cb(geometry_msgs::Pose pose);
    void coag_cb(const std_msgs::BoolConstPtr & state);

    std::vector<geometry_msgs::Pose> dvrk_pose;
    std::vector<geometry_msgs::Pose> dvrk_traj;
    std::vector<sensor_msgs::JointState> dvrk_js;
};

void Traj::pose_cb(geometry_msgs::Pose pose)
{
    if (this->dvrk_pose.size() >= 5){
        this->dvrk_pose.clear();
    }
    this->dvrk_pose.push_back(pose);
}

void Traj::coag_cb(const std_msgs::BoolConstPtr & state)
{
    ROS_INFO("Clutch Callback");
    std::cout << "Get in" << std::endl;
    if (state->data == true){
        if(this->dvrk_pose.size() > 0)
        {
            this->dvrk_traj.push_back(this->dvrk_pose.back());
        }
        else{
            ROS_ERROR("dvrk_traj vector's size is 0, something is wrong");
        }
    }

}

int main(int argc,char **argv )
{
    ros::init(argc,argv,"dvrk_trajectory_node");
    ros::NodeHandle node;
    Traj dvrk;
    ros::Subscriber coag_sub = node.subscribe("/dvrk_footpedal/coag_state",1000, &Traj::coag_cb, &dvrk);
    ros::Subscriber pose_sub = node.subscribe("/dvrk_mtm/cartesian_pose_current",1000, &Traj::pose_cb, &dvrk);
    ros::Publisher  pose_pub = node.advertise<geometry_msgs::Pose>("/dvrk_trajectory/Pose",1);

    while(ros::ok())
    {
        if (dvrk.dvrk_traj.size()>0)
            for(size_t i=0 ; i < dvrk.dvrk_traj.size() ; i++)
            {
                pose_pub.publish(dvrk.dvrk_traj.at(i));
            }
    }

    ros::spin();

    return 0;
}
