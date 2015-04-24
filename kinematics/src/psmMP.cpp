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
    ros::Rate *rate_;
    bool pose_cb_switch;
    bool do_planning_;

    ros::Publisher pose_pub;
    ros::Publisher coll_marker_pub;


    ros::Subscriber coag_sub;
    ros::Subscriber clutch_sub;
    ros::Subscriber planning_scene_msg_sub;

    void coag_cb(const std_msgs::BoolConstPtr &state);
    void clutch_cb(const std_msgs::BoolConstPtr &state);
    void compute_cart_path();
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void planning_scene_cb(moveit_msgs::PlanningScene scene);
    void check_collison();


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

    this->coag_sub = node_.subscribe(
                "/dvrk_footpedal/coag_state",10,
                                     &Kinematic_group::coag_cb, this);
    this->clutch_sub = node_.subscribe(
                "/dvrk_footpedal/clutch_state",10,
                                       &Kinematic_group::clutch_cb, this);
    this->planning_scene_msg_sub = node_.subscribe(
                "/move_group/monitored_planning_scene",10,&Kinematic_group::planning_scene_cb,this);


    this->pose_pub = node_.advertise<geometry_msgs::PoseStamped>
            ("/mp_psm/cartesian_pose_current",1);
    this->coll_marker_pub = node_.advertise<visualization_msgs::MarkerArray>
            ("/interactive_robot_array",100);
}

void Kinematic_group::coag_cb(const std_msgs::BoolConstPtr & state)
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
    }


}


void Kinematic_group::clutch_cb(const std_msgs::BoolConstPtr & state)
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
           color.b = 1.0;
           color.a = 0.5;
           visualization_msgs::MarkerArray markers;
           collision_detection::getCollisionMarkersFromContacts(markers,
                                                                group->getPlanningFrame().c_str(),
                                                                coll_res.contacts,
                                                                color,
                                                                ros::Duration(), // remain until deleted
                                                                0.01);
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
