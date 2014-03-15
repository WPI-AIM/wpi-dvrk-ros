/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSBridge.cpp 4363 2013-07-16 20:32:30Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar(Edits)
  Created on: 2014-03-15

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include "cisst_ros_bridge/mtsROSBridge.h"

CMN_IMPLEMENT_SERVICES(mtsROSBridge);

mtsROSBridge::mtsROSBridge(const std::string & componentName, double periodInSeconds, bool spin):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mSpin(spin)
{
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[strlen("mtsROSBridge") + 1];
    strcpy(argv[0], "mtsROSBridge");
    int argc = 1;
    ros::init(argc, argv, "mtsROSBridge");
    Node = new ros::NodeHandle;
}

void mtsROSBridge::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsROSBridge::Startup(void)
{
}

void mtsROSBridge::Cleanup(void)
{
}

void mtsROSBridge::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    const PublishersType::iterator end = Publishers.end();
    PublishersType::iterator iter;
    for (iter = Publishers.begin();
         iter != end;
         ++iter) {
        (*iter)->Execute();
    }

    if (mSpin) ros::spinOnce();
}
