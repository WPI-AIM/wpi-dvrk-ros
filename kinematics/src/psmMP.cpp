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
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
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
    geometry_msgs::Vector3 spr_collision_direction;
    double radius_SPR; //Three dimensional radii of spherical proxy region
    geometry_msgs::WrenchStamped spr_haptic_force;
    double norm, deflection_norm;
    double force_magnitude;
    tf::Matrix3x3 rot_mat6wrt0;
    tf::Vector3 tf_vec3;
    sensor_msgs::PointCloud pc;
    std::vector<tf::Vector3> fcl_normals;
    std::vector<double> collision_depth;
    ros::Rate *rate_;
    bool pose_cb_switch;
    bool do_planning_;

    ros::Publisher pose_pub;
    ros::Publisher coll_marker_pub;
    ros::Publisher traj_pc_pub;
    ros::Publisher spr_haptic_pub;


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
    void calculate_spr_collision_direction(visualization_msgs::MarkerArray &markers);
    void normalize(geometry_msgs::Vector3 &v);
    void compute_deflection_force(geometry_msgs::Wrench &wrench);
    void compute_norm(geometry_msgs::Vector3 &v, double &n);
    void compute_force_in_tip_frame(geometry_msgs::Wrench &wrench);
    void get_collision_normals(const collision_detection::CollisionResult::ContactMap& con, std::vector<tf::Vector3> &n, std::vector<double> &d);


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
    this->spr_haptic_pub = node_.advertise<geometry_msgs::WrenchStamped>
            ("/dvrk_psm/haptics_feedback_force",1);
    spr_haptic_force.header.frame_id = "one_tool_wrist_sca_ee_link_1";

    rot_mat6wrt0.setRPY(-M_PI/2,0,0);
    radius_SPR = 0.01; //If using a SPR of 20 mm Diameter
    //radius_SPR = 0.0125; //If using a SPR of 25 mm Diameter
    //radius_SPR = 0.015; //If using a SPR of 30 mm Diameter

    group->setEndEffectorLink("one_tool_wrist_sca_ee_link_1");
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
           get_collision_normals(coll_res.contacts, fcl_normals, collision_depth);
           ROS_INFO("Size of FCL Normals = %d", fcl_normals.size());
           ROS_INFO("Normal x: %f y: %f z:%f",fcl_normals.at(0).getX(),fcl_normals.at(0).getY(),fcl_normals.at(0).getZ());
           ROS_INFO("Depth: %f",collision_depth.at(0));
           calculate_spr_collision_direction(markers);
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

void Kinematic_group::calculate_spr_collision_direction(visualization_msgs::MarkerArray &markers){
    spr_collision_direction.x = 0;
    spr_collision_direction.y = 0;
    spr_collision_direction.z = 0;
    //If more than 1 marker, add all the deflection vectors to get an average vector
    for(size_t i = 0 ; i < markers.markers.size() ; i++){
    spr_collision_direction.x += (markers.markers.at(i).pose.position.x - group->getCurrentPose().pose.position.x);
    spr_collision_direction.y += (markers.markers.at(i).pose.position.y - group->getCurrentPose().pose.position.y);
    spr_collision_direction.z += (markers.markers.at(i).pose.position.z - group->getCurrentPose().pose.position.z);
    }
    spr_collision_direction.x = spr_collision_direction.x/markers.markers.size();
    spr_collision_direction.y = spr_collision_direction.y/markers.markers.size();
    spr_collision_direction.z = spr_collision_direction.z/markers.markers.size();

    compute_deflection_force(spr_haptic_force.wrench);
    compute_force_in_tip_frame(spr_haptic_force.wrench);
    spr_haptic_pub.publish(spr_haptic_force);

    ROS_INFO("SPR Collision x:%f y:%f z:%f", spr_collision_direction.x,
             spr_collision_direction.y,
             spr_collision_direction.z);
}

void Kinematic_group::normalize(geometry_msgs::Vector3 &v){
    compute_norm(v,norm);
    v.x = v.x/norm;
    v.y = v.y/norm;
    v.z = v.z/norm;

}

void Kinematic_group::compute_norm(geometry_msgs::Vector3 &v, double &n){
    n = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    n = sqrt(n);
}

void Kinematic_group::compute_deflection_force(geometry_msgs::Wrench &wrench){
    compute_norm(spr_collision_direction,deflection_norm);
    force_magnitude = radius_SPR - deflection_norm;
    normalize(spr_collision_direction);
    wrench.force.x = spr_collision_direction.x * force_magnitude;
    wrench.force.y = spr_collision_direction.y * force_magnitude;
    wrench.force.z = spr_collision_direction.z * force_magnitude;
}

void Kinematic_group::compute_force_in_tip_frame(geometry_msgs::Wrench &wrench){
    rot_mat6wrt0.setRPY(group->getCurrentRPY().at(0),
                        group->getCurrentRPY().at(1),
                        group->getCurrentRPY().at(2));
    tf_vec3.setValue(wrench.force.x,wrench.force.y,wrench.force.z);
    tf_vec3 = rot_mat6wrt0.transpose() * tf_vec3;
    wrench.force.x = tf_vec3.getX();
    wrench.force.y = tf_vec3.getY();
    wrench.force.z = tf_vec3.getZ();
}


// Trying this out to see if we can extract FCL collision normals for generic collision
void Kinematic_group::get_collision_normals(const collision_detection::CollisionResult::ContactMap &con, std::vector<tf::Vector3> &n, std::vector<double> &d){
    n.clear();
    d.clear();
    std::map<std::string, unsigned> ns_counts;
    tf::Vector3 temp;
   for(collision_detection::CollisionResult::ContactMap::const_iterator it = con.begin(); it != con.end(); ++it)
    {
      for(unsigned int i = 0; i < it->second.size(); ++i)
      {
        std::string ns_name = it->second[i].body_name_1 + "=" + it->second[i].body_name_2;
        if (ns_counts.find(ns_name) == ns_counts.end())
          ns_counts[ns_name] = 0;
        else
        ns_counts[ns_name]++;
        temp.setValue(it->second[i].normal.x(),it->second[i].normal.y(),it->second[i].normal.z());
        d.push_back(it->second[i].depth);
        n.push_back(temp);
      }
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
