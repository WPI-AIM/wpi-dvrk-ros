#include "mtm_haptics/haptic_position_kinematics.h"



MTM_pos_kinematics::MTM_pos_kinematics(){
    std::string filename = ros::package::getPath("mtm_haptics");
    filename.append("/config/dvmtm_position.rob");
    result = mtm_manip.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        ROS_ERROR("failed to load manipulator config file: %s", filename.c_str());
    } else {
        ROS_INFO("loaded mtm manipulator");
    }

    cur_jnt_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &MTM_pos_kinematics::jnt_pos_cb, this);

}

void MTM_pos_kinematics::jnt_pos_cb(const sensor_msgs::JointStateConstPtr &msg){
    mtm_joint_current[0] = msg->position[0];   // outer_yaw_joint
    mtm_joint_current[1] = msg->position[1];   // shoulder_pitch_joint
    mtm_joint_current[2] = msg->position[2];   // elbow_pitch_joint
}

void MTM_pos_kinematics::compute_torques(const geometry_msgs::Wrench &F, sensor_msgs::JointState torque_msg){
   mtsROSToCISST(F,this->newForce);
   vctDoubleVec jointDesired( 3, 0.0 );
        for ( size_t i=0; i<jointDesired.size(); i++ ){
            jointDesired[i] = mtm_joint_current[i];
        }

        mtm_manip.JacobianBody( jointDesired );
        vctDynamicMatrix<double> J( 3, mtm_manip.links.size(), VCT_COL_MAJOR );
        for( size_t r=0; r<3; r++ ){
            for( size_t c=0; c<mtm_manip.links.size(); c++ ){
                J[r][c] = mtm_manip.Jn[c][r];
            }
        }

        prmForceCartesianSet tmp = newForce;
        prmForceCartesianSet::ForceType tmpft;
        tmp.GetForce( tmpft );
        vctDynamicMatrix<double> ft( tmpft.size(), 1, 0.0, VCT_COL_MAJOR );
        for( size_t i=0; i<ft.size(); i++ ){
            ft[i][0] = tmpft[i];
        }
        vctDynamicMatrix<double> t = nmrLSMinNorm( J, ft );


        vctDoubleVec torqueDesired(8, 0.0);
        for( size_t i=0; i<3; i++ ){
            torqueDesired[i] = t[i][0];
        }

        if( torqueDesired[0] < -2.0 ) { torqueDesired[0] = -2.0; }
        if( 2.0 < torqueDesired[0]  ) { torqueDesired[0] =  2.0; }
        if( torqueDesired[1] < -2.0 ) { torqueDesired[1] = -2.0; }
        if( 2.0 < torqueDesired[1]  ) { torqueDesired[1] =  2.0; }
        if( torqueDesired[2] < -2.0 ) { torqueDesired[2] = -2.0; }
        if( 2.0 < torqueDesired[2]  ) { torqueDesired[2] =  2.0; }

        if( torqueDesired[3] < -1.0 ) { torqueDesired[3] =  0; }
        if( 1.0 < torqueDesired[3]  ) { torqueDesired[3] =  0; }
        if( torqueDesired[4] < -1.0 ) { torqueDesired[4] =  0; }
        if( 1.0 < torqueDesired[4]  ) { torqueDesired[4] =  0; }
        if( torqueDesired[5] < -1.0 ) { torqueDesired[5] =  0; }
        if( 1.0 < torqueDesired[5]  ) { torqueDesired[5] =  0; }
        if( torqueDesired[6] < -1.0 ) { torqueDesired[6] =  0; }
        if( 1.0 < torqueDesired[6]  ) { torqueDesired[6] =  0; }

           for( size_t i=0; i<7; i++){
           torque_msg.effort[i] = torqueDesired[i];
           }

}

