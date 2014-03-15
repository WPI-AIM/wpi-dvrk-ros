/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSBridge.h 4367 2013-07-17 02:47:21Z zchen24 $

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

#ifndef _mtsROSBridge_h
#define _mtsROSBridge_h

// cisst include
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

// ros include
#include <ros/ros.h>

// conversion methods
#include "cisst_ros_bridge/mtsCISSTToROS.h"
#include "cisst_ros_bridge/mtsROSToCISST.h"

class mtsROSPublisherBase
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionRead Function;
    //! ROS publisher to publish the converted data
    ros::Publisher Publisher;

    virtual bool Execute(void) = 0;
};

template <typename _mtsType, typename _rosType>
class mtsROSPublisher: public mtsROSPublisherBase
{
public:
    mtsROSPublisher(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSPublisher() {
        // \todo, how to remove the topic from the node?
    }

    bool Execute(void) {
        mtsExecutionResult result = Function(CISSTData);

        if (result) {
            mtsCISSTToROS(CISSTData, ROSData);
            Publisher.publish(ROSData);
            return true;
        }
//        std::cerr << "we need to log with ROS and cisst?" << std::endl;
    }

protected:
    _mtsType CISSTData;
    _rosType ROSData;
};


class mtsROSSubscriberBase
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionWrite Function;
    //! ROS publisher to publish the converted data
    ros::Subscriber Subscriber;
};

template <typename _mtsType, typename _rosType>
class mtsROSSubscriber: public mtsROSSubscriberBase
{
public:
    typedef mtsROSSubscriber<_mtsType, _rosType> ThisType;
    mtsROSSubscriber(const std::string & rosTopicName, ros::NodeHandle & node) {
        Subscriber = node.subscribe(rosTopicName, 1, &ThisType::Callback, this);
//        this->numberOfJoints = numberOfAxis;
        std::cout << "topic: " << rosTopicName << std::endl;
    }
    ~mtsROSSubscriber() {
        // \todo, how to remove the subscriber from the node?
    }

    void Callback(const _rosType & rosData) {
        mtsROSToCISST(rosData, CISSTData);

        std::cout << "cisstdata = " << CISSTData << std::endl;

        mtsExecutionResult result = Function(CISSTData);
        if (!result) {
            std::cerr << result << std::endl;
        }
    }

protected:
    _mtsType CISSTData;
    _rosType ROSData;
    int numberOfJoints;
};


class mtsROSBridge: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsROSBridge(const std::string & componentName, double periodInSeconds, bool spin = false);
    inline ~mtsROSBridge() {}

    // taskPeriodic
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromReadCommand(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToWriteCommand(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

protected:
    //! list of publishers
    typedef std::list<mtsROSPublisherBase*> PublishersType;
    PublishersType Publishers;

    //! list of subscribers
    typedef std::list<mtsROSSubscriberBase*> SubscribersType;
    SubscribersType Subscribers;

    //! ros node
    ros::NodeHandle * Node;

    //! spin flag, if set call spinOnce() in run
    bool mSpin;
    //! Number of Axes for the Arm Type
    int numberOfAxis;
};

template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromReadCommand(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommand: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommand: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * newPublisher = new mtsROSPublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommand: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToWriteCommand(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddSubscribeToWriteCommand: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscribeToWriteCommand: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
//    switch (arm_name)
//    {
//    case "PSM1":
//        this->numberOfAxis = 7;
//        break;
//    case "PSM2":
//        this->numberOfAxis = 7;
//        break;
//    case "PSM":
//        this->numberOfAxis = 7;
//        break;
//    case "psm1":
//        this->numberOfAxis = 7;
//        break;
//    case "psm2":
//        this->numberOfAxis = 7;
//        break;
//    case "psm":
//        this->numberOfAxis = 7;
//        break;
//    case "MTML":
//        this->numberOfAxis = 8;
//        break;
//    case "MTMR":
//        this->numberOfAxis = 8;
//        break;
//    case "MTM":
//        this->numberOfAxis = 8;
//        break;
//    case "mtml":
//        this->numberOfAxis = 8;
//        break;
//    case "mtmr":
//        this->numberOfAxis = 8;
//        break;
//    case "mrm":
//        this->numberOfAxis = 8;
//        break;
//    default:
//        ROS_ERROR("Unknow Arm Type ? (MTM/PSM), Unable to specify number of axes");
//        return false;
//        break;
//    }
    mtsROSSubscriberBase * newSubscriber = new mtsROSSubscriber<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->Function)) {
        ROS_ERROR("mtsROS::AddSubscriberToWriteCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToWriteCommand: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    Subscribers.push_back(newSubscriber);
    return true;
}

CMN_DECLARE_SERVICES_INSTANTIATION(mtsROSBridge);

#endif // _mtsROSBridge_h
