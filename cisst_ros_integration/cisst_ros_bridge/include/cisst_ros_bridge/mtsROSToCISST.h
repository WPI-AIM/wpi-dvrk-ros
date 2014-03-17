/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSToCISST.h 4375 2013-07-26 16:13:06Z zchen24 $

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

#ifndef _mtsROSToCISST_h
#define _mtsROSToCISST_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// non standard messages
#include "cisst_msgs/vctDoubleVec.h"

// std_msgs
void mtsROSToCISST(const std_msgs::Float32 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::Bool & rosData, bool & cisstData);
void mtsROSToCISST(const std_msgs::String &rosData, std::string &cisstData);

// geometry_msgs
// This Function has been modified by Adnan Munawar and now works to set
// the cartesian Positions of the MTM. Still under work for setting cartesian
// Position of the PSMs
void mtsROSToCISST(const geometry_msgs::Pose &rosData, prmPositionCartesianSet &cisstData);
void mtsROSToCISST(const geometry_msgs::Pose &rosData, vctFrm4x4 &cisstData);
void mtsROSToCISST(const geometry_msgs::Transform & rosData, prmPositionCartesianGet & cisstData);

// This Function has been implemented as a Test by Adnan Munawar
// This Functions works for the PSM now, however still under work for working
// on the MTMs
void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmPositionJointSet & cisstData);
// vctDoubleVec
void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, vctDoubleVec & cisstData);

#endif // _mtsROSToCISST_h
