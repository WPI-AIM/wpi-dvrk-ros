#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "daVinciGazeboJointController.h"
#include "DaVinci_IK.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

using namespace std;
using namespace KDL;
float joint_0,joint_1,joint_2;
float dx = 0.1;
float dy = 0.05;

void mykeyboardfunc(unsigned char val)
{
    switch (val)
    {
    case 'a':
        joint_0+=dx;
        break;
    case 'A':
        joint_0-=dx;
        break;
    case 's':
        joint_1+=dx;
        break;
    case 'S':
         joint_1-=dx;
        break;
    case 'd':
         joint_2+=dy;
        break;
    case 'D':
         joint_2-=dy;
         break;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "virtual_master_service_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<daVinciGazeboPlugins::SetJointState>("/psm1/set_joint_states");

    daVinciGazeboPlugins::SetJointState srv;
    srv.request.joint_state.name.resize(3);
    srv.request.joint_state.position.resize(3);
    srv.request.joint_state.name[0] = "right_arm_outer_yaw_joint";
    srv.request.joint_state.name[1] = "right_arm_outer_pitch_base_joint";
    srv.request.joint_state.name[2] = "right_arm_tool_insertion_joint";
    float x = 0.01;
    srv.request.joint_state.position[0] = x;
    srv.request.joint_state.position[1] = x;
    srv.request.joint_state.position[2] = x;
    ros::Rate looprate(5);
    looprate.sleep();
    client.call(srv);
    unsigned char val;
    //DaVinci_IK solver;
    //while(1)
    //{
    //solver.read_urdf(argv[1]);
    //}

    while (1)
    {
        cin>>val;
        mykeyboardfunc(val);
        srv.request.joint_state.position[0]  = joint_0;
        srv.request.joint_state.position[1]  = joint_1;
        srv.request.joint_state.position[2]  = joint_2;

        client.call(srv);
    }
    return 0;
}


