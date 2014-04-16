#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/JointState.h>
#include<vector>
#include<std_msgs/Bool.h>
#include<iostream>

class Traj
{
public:
    void pose_cb(const geometry_msgs::PoseConstPtr & pose);
    void coag_cb(const std_msgs::BoolConstPtr & state);
    void jointstate_cb(const sensor_msgs::JointStateConstPtr & js);

    std::vector<geometry_msgs::Pose> dvrk_pose;
    std::vector<sensor_msgs::JointState> dvrk_js;
    std::vector<geometry_msgs::Pose> dvrk_traj_pose;
    std::vector<sensor_msgs::JointState> dvrk_traj_js;

};

void Traj::pose_cb(const geometry_msgs::PoseConstPtr & pose)
{
    if (this->dvrk_pose.size() >= 5){
        this->dvrk_pose.clear();
    }
    this->dvrk_pose.push_back(*pose.get());
}

void Traj::jointstate_cb(const sensor_msgs::JointStateConstPtr &js)
{
    if (this->dvrk_pose.size() >= 5){
        this->dvrk_pose.clear();
    }
    this->dvrk_js.push_back(*js.get());
}

void Traj::coag_cb(const std_msgs::BoolConstPtr & state)
{
    if (state->data == true){
        if(this->dvrk_pose.size() > 0)
        {
            ROS_INFO("Catching Pose to add to trajectory");
            this->dvrk_traj_pose.push_back(this->dvrk_pose.back());
            this->dvrk_traj_js.push_back(this->dvrk_js.back());
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
    ros::Subscriber js_sub = node.subscribe("/dvrk_mtm/joint_position_current",1000,&Traj::jointstate_cb,&dvrk);
    ros::Publisher  pose_pub = node.advertise<geometry_msgs::Pose>("/dvrk_trajectory/Pose",1);
    ros::Publisher js_pub = node.advertise<sensor_msgs::JointState>("dvrk_trajectory/JointState",1);
    ros::Rate rate(200);
    while(ros::ok())
    {
        if (dvrk.dvrk_traj_pose.size()>0)
            for(int i=0 ; i < dvrk.dvrk_traj_pose.size() ; i++)
            {
                pose_pub.publish(dvrk.dvrk_traj_pose.at(i));
            }
        if (dvrk.dvrk_traj_js.size()>0)
            for(int i=0 ; i < dvrk.dvrk_traj_js.size() ; i++)
            {
                js_pub.publish(dvrk.dvrk_traj_js.at(i));
            }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
