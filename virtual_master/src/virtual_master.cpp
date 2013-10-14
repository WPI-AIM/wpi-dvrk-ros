#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sstream>

int main(int argc, char **argv)
{
ros::init(argc, argv, "Talker");
ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("/psm1/joint_state",1000);

ros::Rate loop_rate(1);

int count = 0;
sensor_msgs::JointState jstate;
jstate.name.resize(1);
jstate.name[0] = "left_arm_outer_yaw_joint";
jstate.position.resize(1);
jstate.position[0]  = 0;

while (ros::ok())
{
jstate.position[0] +=0.1;
chatter_pub.publish(jstate);
ros::spinOnce();
printf("What the F\n");
loop_rate.sleep();
++count;
}

return 0;
}

