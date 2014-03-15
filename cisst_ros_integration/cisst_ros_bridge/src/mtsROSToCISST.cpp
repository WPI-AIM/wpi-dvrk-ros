/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSToCISST.cpp 4375 2013-07-26 16:13:06Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-03-15

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "cisst_ros_bridge/mtsROSToCISST.h"

void mtsROSToCISST(const std_msgs::Float32 &rosData, double &cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::Bool &rosData, bool &cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::String &rosData, std::string &cisstData)
{
    cisstData = rosData.data;
}


void mtsROSToCISST(const geometry_msgs::Pose &rosData, prmPositionCartesianGet &cisstData)
{
    cisstData.Position().Translation().X() = rosData.position.x;
    cisstData.Position().Translation().Y() = rosData.position.y;
    cisstData.Position().Translation().Z() = rosData.position.z;
    vctQuatRot3 quat;
    quat.X() = rosData.orientation.x;
    quat.Y() = rosData.orientation.y;
    quat.Z() = rosData.orientation.z;
    quat.W() = rosData.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Position().Rotation().Assign(rotation);
}

void mtsROSToCISST(const geometry_msgs::Pose &rosData, vctFrm4x4 &cisstData)
{
    cisstData.Translation().X() = rosData.position.x;
    cisstData.Translation().Y() = rosData.position.y;
    cisstData.Translation().Z() = rosData.position.z;
    vctQuatRot3 quat;
    quat.X() = rosData.orientation.x;
    quat.Y() = rosData.orientation.y;
    quat.Z() = rosData.orientation.z;
    quat.W() = rosData.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Rotation().Assign(rotation);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, prmPositionCartesianGet & cisstData)
{
    cisstData.Position().Translation().X() = rosData.translation.x;
    cisstData.Position().Translation().Y() = rosData.translation.y;
    cisstData.Position().Translation().Z() = rosData.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosData.rotation.x;
    quat.Y() = rosData.rotation.y;
    quat.Z() = rosData.rotation.z;
    quat.W() = rosData.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Position().Rotation().Assign(rotation);
}

// This function has been implemented as a TEST BY ADNAN MUNAWAR
void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmPositionJointSet &cisstData)
{
    vctDoubleVec DesiredPosition;
    DesiredPosition.SetSize(rosData.position.size());
    DesiredPosition.SetAll(0.0);
    for (unsigned int i = 0; i < rosData.position.size(); ++i) {
        DesiredPosition.at(i) = rosData.position[i];
    }
    cisstData.SetGoal(DesiredPosition);
}

void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, vctDoubleVec & cisstData)
{
    cisstData.resize(rosData.data.size());
    for (size_t i = 0; i < cisstData.size(); ++i) {
        cisstData.Element(i) = rosData.data[i];
    }    
}
