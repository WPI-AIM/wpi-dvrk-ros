#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>

class cartesian_pose{
public:
    cartesian_pose();
    void listen();
    tf::TransformListener listener;
    tf::StampedTransform tran;
    ros::Publisher pub;
    ros::NodeHandle node;
};

cartesian_pose::cartesian_pose()
{
    this->pub = this->node.advertise<geometry_msgs::PoseStamped>("/simulated_mtm/end_effector_pose",1);
}

void cartesian_pose::listen()
{
    try{
        this->listener.lookupTransform("right_back_parallel_link","right_wrist_roll_link",ros::Time(0),this->tran);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"end_effector_pose_node");
    ros::Rate rate(500);
    cartesian_pose cart_pos;
    geometry_msgs::PoseStamped pose_stamped;
    while(ros::ok())
    {
        pose_stamped.header.stamp = cart_pos.tran.stamp_;
        pose_stamped.pose.position.x = cart_pos.tran.getOrigin().getX();
        pose_stamped.pose.position.y = cart_pos.tran.getOrigin().getY();
        pose_stamped.pose.position.z = cart_pos.tran.getOrigin().getZ();
        pose_stamped.pose.orientation.x = cart_pos.tran.getRotation().getX();
        pose_stamped.pose.orientation.y = cart_pos.tran.getRotation().getY();
        pose_stamped.pose.orientation.z = cart_pos.tran.getRotation().getZ();
        pose_stamped.pose.orientation.w = cart_pos.tran.getRotation().getW();

        cart_pos.pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }
}
