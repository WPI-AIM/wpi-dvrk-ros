/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsCISSTToROS.cpp 4375 2013-07-26 16:13:06Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "cisst_ros_bridge/mtsCISSTToROS.h"

void mtsCISSTToROS(const double &cisstData, std_msgs::Float32 &rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool &cisstData, std_msgs::Bool &rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const std::string &cisstData, std_msgs::String &rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const prmEventButton &cisstData, std_msgs::Bool &rosData)
{
  if (cisstData.Type() == prmEventButton::PRESSED)
      rosData.data = true;
  else if (cisstData.Type() == prmEventButton::RELEASED)
      rosData.data = false;
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData)
{
    vctQuatRot3 quat(cisstData.Position().Rotation(), VCT_NORMALIZE);
    rosData.rotation.x = quat.X();
    rosData.rotation.y = quat.Y();
    rosData.rotation.z = quat.Z();
    rosData.rotation.w = quat.W();
    rosData.translation.x = cisstData.Position().Translation().X();
    rosData.translation.y = cisstData.Position().Translation().Y();
    rosData.translation.z = cisstData.Position().Translation().Z();
}

void mtsCISSTToROS(const prmPositionCartesianGet &cisstData, geometry_msgs::Pose &rosData)
{
    vctQuatRot3 quat(cisstData.Position().Rotation(), VCT_NORMALIZE);
    rosData.orientation.x = quat.X();
    rosData.orientation.y = quat.Y();
    rosData.orientation.z = quat.Z();
    rosData.orientation.w = quat.W();
    rosData.position.x = cisstData.Position().Translation().X();
    rosData.position.y = cisstData.Position().Translation().Y();
    rosData.position.z = cisstData.Position().Translation().Z();
}

void mtsCISSTToROS(const vctFrm4x4 &cisstData, geometry_msgs::Pose &rosData)
{
    vctQuatRot3 quat(cisstData.Rotation(), VCT_NORMALIZE);
    rosData.orientation.x = quat.X();
    rosData.orientation.y = quat.Y();
    rosData.orientation.z = quat.Z();
    rosData.orientation.w = quat.W();
    rosData.position.x = cisstData.Translation().X();
    rosData.position.y = cisstData.Translation().Y();
    rosData.position.z = cisstData.Translation().Z();
}

void mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData)
{
    rosData.position.resize(cisstData.Position().size());
    for (unsigned int i = 0; i < cisstData.Position().size(); ++i) {
        rosData.position[i] = cisstData.Position().Element(i);
    }
}

void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData)
{
    rosData.data.resize(cisstData.size());
    for (size_t i = 0; i < cisstData.size(); ++i) {
        rosData.data[i] = cisstData.Element(i);
    }
}
