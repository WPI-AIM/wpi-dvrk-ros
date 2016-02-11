//Author : Adnan Munawar
//Email : amunawar@wpi.edu


#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/JointState.h>
#include<vector>
#include<std_msgs/Bool.h>
#include<std_msgs/Empty.h>
#include<std_msgs/UInt64.h>
#include<iostream>
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>

// Class for handling the capturing of poses from the MTM via Coag/Mono Presses and Publishing them on pressing the
// Clutch. I plan to change that in the future to some other pedal or maybe the gripper pinch event
class Traj
{
public:
    Traj();
    void mtm_ee_pose_cb(const geometry_msgs::PoseConstPtr & pose);
    void coag_foot_pedal_cb(const std_msgs::BoolConstPtr & state);
    void mtm_jointstate_cb(const sensor_msgs::JointStateConstPtr & js);
    void clutch_foot_pedal_cb(const std_msgs::BoolConstPtr & state);

protected:
// quene_size decided the number of poses or joint_state messages to store before starting to store new ones and discarding old ones
    std::size_t quene_size;
// len is the number of poses that are being published in the trajectory publisher
    std_msgs::UInt64 len;
    sensor_msgs::PointCloud pc;
    geometry_msgs::Point32 pc_point;

    std::vector<geometry_msgs::Pose> mtm_ee_poses;
    std::vector<sensor_msgs::JointState> mtm_joint_states;
    std::vector<geometry_msgs::Pose> clicked_mtm_ee_pose;
    std::vector<sensor_msgs::JointState> clicked_mtm_joint_states;

    ros::NodeHandle node;
    ros::Rate *rate;

    ros::Publisher traj_pub;
    ros::Publisher traj_length_pub;
    ros::Publisher traj_pc_pub;

    ros::Subscriber caog_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber js_sub;
    ros::Subscriber clutch_sub;
};

Traj::Traj()
{
    this->quene_size = 50;
    this->rate = new ros::Rate(1000);
    this->caog_sub = node.subscribe("/dvrk_footpedal/coag_state",1000, &Traj::coag_foot_pedal_cb, this);
    this->pose_sub = node.subscribe("/dvrk_mtm/cartesian_pose_current",1000, &Traj::mtm_ee_pose_cb,this);
    this->js_sub = node.subscribe("/dvrk_mtm/joint_position_current",1000,&Traj::mtm_jointstate_cb,this);
    this->clutch_sub = node.subscribe("/dvrk_footpedal/clutch_state",1000,&Traj::clutch_foot_pedal_cb,this);
    this->traj_pub = node.advertise<geometry_msgs::Pose>("/mtm/trajectory_poses",1);
    this->traj_length_pub = node.advertise<std_msgs::UInt64>("/mtm/trajectory_poses_size",1);
    this->traj_pc_pub = node.advertise<sensor_msgs::PointCloud>("/mtm/trajectory_points_pointcloud",1);

    this->pc.header.frame_id = "right_base";
}

//This cb is called whenever a new pose is recieved.
void Traj::mtm_ee_pose_cb(const geometry_msgs::PoseConstPtr & pose)
{
    if (this->mtm_ee_poses.size() >= quene_size){
        // Not deleting the entire vector since the calls to ros subscriber do not seem to blocking,
        // thus when this vector is used to fecth the last item in coag_foot_pedal_cb when its empty
        // and just before the new value is filled in, error occurs.
        this->mtm_ee_poses.erase(mtm_ee_poses.begin(),mtm_ee_poses.end()-1);
    }
    this->mtm_ee_poses.push_back(*pose.get());
}
//This cb is called whenever a new JointState message is recieved.
void Traj::mtm_jointstate_cb(const sensor_msgs::JointStateConstPtr &js)
{
    if (this->mtm_ee_poses.size() >= quene_size){
        this->mtm_ee_poses.clear();
    }
    this->mtm_joint_states.push_back(*js.get());
}

//This cb is called whenever the coag/mono footpedal is pressed. The last pose and JointState message in quene is pushed
//back to the corresponding vectors for storage.
void Traj::coag_foot_pedal_cb(const std_msgs::BoolConstPtr & clutch_pressed)
{
    if (clutch_pressed->data == true){
        if(this->mtm_ee_poses.size() > 0)
        {
            this->clicked_mtm_ee_pose.push_back(this->mtm_ee_poses.back());
            this->clicked_mtm_joint_states.push_back(this->mtm_joint_states.back());
            //this->pc_point.__connection_header = this->mtm_ee_poses.back().__connection_header;
            this->pc_point.x = this->mtm_ee_poses.back().position.x;
            this->pc_point.y = this->mtm_ee_poses.back().position.y;
            this->pc_point.z = this->mtm_ee_poses.back().position.z;

            this->pc.points.push_back(this->pc_point);
            this->traj_pc_pub.publish(this->pc);
            ROS_INFO("Catching Pose: Size of Trajectory: %lu Poses", clicked_mtm_ee_pose.size());
        }
        else{
            ROS_ERROR("dvrk_traj vector's size is 0, something is wrong");
        }
    }
}

//This cb is called whenver the clutch is pressed. This functions is used to trigger the publishing of the poses and JointStates stored
//in the coag_foot_pedal_cb funciton.

void Traj::clutch_foot_pedal_cb(const std_msgs::BoolConstPtr & state)
{
    if( state->data == true)
    {
    if (this->clicked_mtm_ee_pose.size()>0){
        ROS_INFO("Clutch Pressed: Publishing %lu Trajectory Poses",clicked_mtm_ee_pose.size());
        this->len.data = this->clicked_mtm_ee_pose.size();
        this->traj_length_pub.publish(len);
        this->rate->sleep();
        for(size_t i=0 ; i < clicked_mtm_ee_pose.size() ; i++)
        {
            this->traj_pub.publish(clicked_mtm_ee_pose.at(i));
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
