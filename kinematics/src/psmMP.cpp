//Author : Adnan Munawar
//Email : amunawar@wpi.edu

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <ros/time.h>
#include <geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>

class Kinematic_group{

public:
    Kinematic_group();
    moveit::planning_interface::MoveGroup * group;
    moveit::planning_interface::MoveGroup::Plan plan_srv;
    std::vector<geometry_msgs::Pose> poses_list;
    moveit_msgs::RobotTrajectory trajectory;
    collision_detection::CollisionRequest coll_req;
    collision_detection::CollisionResult coll_res;
    planning_scene::PlanningScene * psm_planning_scene;
    robot_model_loader::RobotModelLoader * robotModel;
    robot_model::RobotModelPtr kinematic_model;
    visualization_msgs::MarkerArray psm_collision_points;
    moveit_msgs::PlanningScene planning_scene_msg;
    moveit_msgs::PlanningSceneWorld world_msg;
    geometry_msgs::Point32 pc_point;
    geometry_msgs::Pose cur_psm_pose;
    geometry_msgs::Pose pre_psm_pose;
    geometry_msgs::Vector3 cur_psm_tip_vel;
    sensor_msgs::PointCloud pc;
    ros::Rate *rate_;
    bool pose_cb_switch;
    bool do_planning_;

    ros::Publisher pose_pub;
    ros::Publisher coll_marker_pub;
    ros::Publisher traj_pc_pub;


    ros::Subscriber coag_sub;
    ros::Subscriber clutch_sub;
    ros::Subscriber planning_scene_msg_sub;
    ros::Subscriber psm_pose_sub;

    void footpedal_cam_minus_cb(const std_msgs::BoolConstPtr &state);
    void footpedal_camera_cb(const std_msgs::BoolConstPtr &state);
    void compute_cart_path();
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void planning_scene_cb(moveit_msgs::PlanningScene scene);
    void check_collison();
    void psm_pose_cb(const geometry_msgs::PoseConstPtr &msg);


protected:
    ros::NodeHandle node_;
    moveit_msgs::RobotTrajectory path_;
    std_msgs::UInt64 traj_size_;
};

Kinematic_group::Kinematic_group()
{
    this->rate_ = new ros::Rate(50);
    this->group = new moveit::planning_interface::MoveGroup("full_chain");
    this->robotModel = new robot_model_loader::RobotModelLoader("robot_description");
    this->kinematic_model = robotModel->getModel();
    this->psm_planning_scene = new planning_scene::PlanningScene(this->kinematic_model);
    this->coll_req.group_name = "full_chain";
    this->coll_req.contacts = true;
    this->coll_req.max_contacts = 100;
    this->coll_req.verbose = false;
    this->pose_cb_switch = false;
    this->do_planning_ = false;
    this->pc.header.frame_id = "world";

    this->coag_sub = node_.subscribe(
                "/dvrk_footpedal/cam_minus_state",10,
                                     &Kinematic_group::footpedal_cam_minus_cb, this);
    this->clutch_sub = node_.subscribe(
                "/dvrk_footpedal/camera_state",10,
                                       &Kinematic_group::footpedal_camera_cb, this);
    this->planning_scene_msg_sub = node_.subscribe(
                "/move_group/monitored_planning_scene",10,&Kinematic_group::planning_scene_cb,this);
    this->psm_pose_sub = node_.subscribe(
                "/dvrk_psm/cartesian_pose_current",10,&Kinematic_group::psm_pose_cb, this);


    this->pose_pub = node_.advertise<geometry_msgs::PoseStamped>
            ("/mp_psm/cartesian_pose_current",1);
    this->coll_marker_pub = node_.advertise<visualization_msgs::MarkerArray>
            ("/interactive_robot_array",100);
    this->traj_pc_pub = node_.advertise<sensor_msgs::PointCloud>
            ("/psm/trajectory_points_pointcloud",1);
}

void Kinematic_group::footpedal_cam_minus_cb(const std_msgs::BoolConstPtr & state)
{
    if(state->data == true){
        this->poses_list.push_back(this->group->getCurrentPose().pose);

        if(this->pose_cb_switch == false){
            ROS_INFO("Setting Start State");
            this->group->setStartState(*this->group->getCurrentState());
            pose_cb_switch = true;
        }
        else{
            ROS_INFO("Setting Goal State");
            this->group->setJointValueTarget(*this->group->getCurrentState());
            pose_cb_switch = false;
        }
        //this->pc_point.__connection_header = this->group->getCurrentPose().pose.__connection_header;
        this->pc_point.x = this->group->getCurrentPose().pose.position.x;
        this->pc_point.y = this->group->getCurrentPose().pose.position.y;
        this->pc_point.z = this->group->getCurrentPose().pose.position.z;
        if (this->pc.points.size() >= 2){
            this->pc.points.clear();
        }
        this->pc.points.push_back(this->pc_point);
        ROS_INFO("Publishing recorded point");

        this->traj_pc_pub.publish(this->pc);
    }


}


void Kinematic_group::footpedal_camera_cb(const std_msgs::BoolConstPtr & state)
{
    if(state->data == true){
        ROS_INFO("Planning Requested");
        //this->compute_cart_path();
        this->do_planning_ = true;
    }
}

void Kinematic_group::compute_cart_path(){

    double fraction = group->computeCartesianPath(this->poses_list,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 this->trajectory);

    ROS_INFO("Cartesian path is (%.2f%% acheived)",
          fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

}

void Kinematic_group::planning_scene_cb(moveit_msgs::PlanningScene scene){
    this->world_msg = scene.world;
    //ROS_INFO("Processing Planning Scene Msg");
    this->psm_planning_scene->processPlanningSceneWorldMsg(this->world_msg);
}

void Kinematic_group::publishMarkers(visualization_msgs::MarkerArray& markers)
 {
   // delete old markers
   if (psm_collision_points.markers.size())
   {
     for (int i=0; i<psm_collision_points.markers.size(); i++)
       psm_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

     coll_marker_pub.publish(psm_collision_points);
   }

   // move new markers into g_collision_points
   std::swap(psm_collision_points.markers, markers.markers);

   // draw new markers (if there are any)
   if (psm_collision_points.markers.size())
     coll_marker_pub.publish(psm_collision_points);
 }

void Kinematic_group::check_collison(){

    this->coll_res.clear();
    this->psm_planning_scene->checkCollisionUnpadded(coll_req,coll_res,*group->getCurrentState(),
                                            psm_planning_scene->getAllowedCollisionMatrix());
    if (coll_res.collision)
       {
         ROS_INFO("COLLIDING contact_point_count=%d",(int)coll_res.contact_count);

         if (coll_res.contact_count > 0)
         {
           std_msgs::ColorRGBA color;
           color.r = 1.0;
           color.g = 0.0;
           color.b = 0.2;
           color.a = 0.8;
           visualization_msgs::MarkerArray markers;
           collision_detection::getCollisionMarkersFromContacts(markers,
                                                                group->getPlanningFrame().c_str(),
                                                                coll_res.contacts,
                                                                color,
                                                                ros::Duration(), // remain until deleted
                                                                0.002);
           publishMarkers(markers);
           }
         }
    else
      {
        //ROS_INFO("Not colliding");

        // delete the old collision point markers
        visualization_msgs::MarkerArray empty_marker_array;
        publishMarkers(empty_marker_array);
      }

}

void Kinematic_group::psm_pose_cb(const geometry_msgs::PoseConstPtr &msg){
    pre_psm_pose = cur_psm_pose;
    cur_psm_pose = *msg;

    cur_psm_tip_vel.x = cur_psm_pose.position.x - pre_psm_pose.position.x;
    cur_psm_tip_vel.y = cur_psm_pose.position.y - pre_psm_pose.position.y;
    cur_psm_tip_vel.z = cur_psm_pose.position.z - pre_psm_pose.position.z;
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"psm_MotionPlanning");
    Kinematic_group traj;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while(ros::ok())
    {
        traj.check_collison();
        if(traj.do_planning_ == true){
           bool success =  traj.group->plan(traj.plan_srv);
           ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
           /* Sleep to give Rviz time to visualize the plan. */
           sleep(5.0);
           traj.do_planning_ = false;
        }
        //ros::spinOnce();
        traj.rate_->sleep();
    }

    ros::shutdown();
    return 0;
}
