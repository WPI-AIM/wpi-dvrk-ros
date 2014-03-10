/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: main.cpp 4363 2013-07-16 20:32:30Z zchen24 $

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

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include "cisst_ros_bridge/mtsROSBridge.h"

class TestComponent: public mtsTaskPeriodic
{
public:
    TestComponent(void):
        mtsTaskPeriodic("testComponent", 5.0 * cmn_ms, 256),
        Value1(4, 0.0),
        Value2(4, 0.0) {
        StateTable.AddData(Value1, "value1");
        StateTable.AddData(Value2, "value2");
        mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("provided");
        interfaceProvided->AddCommandReadState(StateTable, Value1, "GetValue1");
        interfaceProvided->AddCommandReadState(StateTable, Value2, "GetValue2");
        interfaceProvided->AddCommandWrite(&TestComponent::SetValue1, this,
                                           "SetValue1", Value1);
        interfaceProvided->AddCommandWrite(&TestComponent::SetValue2, this,
                                           "SetValue2", Value2);
    }

    void Configure(const std::string &) {
    }

    void Startup(void) {
    }

    void Run(void) {
        ProcessQueuedCommands();
    }

    void Cleanup(void) {
    }

protected:
    void SetValue1(const vctDoubleVec & newValue) {
        Value1.ForceAssign(newValue);
    }

    void SetValue2(const vctDoubleVec & newValue) {
        Value2.ForceAssign(newValue);
    }

    vctDoubleVec Value1;
    vctDoubleVec Value2;
};

int main(int argc, char ** argv)
{
    mtsComponentManager * manager = mtsManagerLocal::GetInstance();

    TestComponent testComponent;
    manager->AddComponent(&testComponent);

    mtsROSBridge publisher("publisher", 20.0 * cmn_ms);
    publisher.AddPublisherFromReadCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>("required",
                                                                                  "GetValue1",
                                                                                  "/sawROSExample/get_value_1");
    publisher.AddPublisherFromReadCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>("required",
                                                                                  "GetValue2",
                                                                                  "/sawROSExample/get_value_2");
    manager->AddComponent(&publisher);
    manager->Connect(publisher.GetName(), "required",
                     testComponent.GetName(), "provided");
    manager->Connect(publisher.GetName(), "ExecIn",
                     testComponent.GetName(), "ExecOut");

    mtsROSBridge sub1("sub1", 1 * cmn_ms);
    sub1.AddSubscriberToWriteCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>("required",
                                                                             "SetValue1",
                                                                             "/sawROSExample/set_value_1");
    manager->AddComponent(&sub1);
    manager->Connect(sub1.GetName(), "required",
                     testComponent.GetName(), "provided");

    mtsROSBridge sub2("sub2", 1 * cmn_ms);
    sub2.AddSubscriberToWriteCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>("required",
                                                                             "SetValue2",
                                                                             "/sawROSExample/set_value_2");
    manager->AddComponent(&sub2);
    manager->Connect(sub2.GetName(), "required",
                     testComponent.GetName(), "provided");


    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    // ros::spin() callback for subscribers
    std::cout << "Hit Ctrl-c to quit" << std::endl;
    ros::spin();

    manager->KillAllAndWait(2.0 * cmn_s);
    manager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
