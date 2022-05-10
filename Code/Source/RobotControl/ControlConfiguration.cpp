/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "RobotControl/ControlConfiguration.h"
#include "RobotControl/TwistControl/TwistBus.h"

namespace ROS2
{

bool ControlConfiguration::IsBroadcastBusModeDisabled() const
{
    return !IsBroadcastBusMode();
}

void ControlConfiguration::Reflect(AZ::ReflectContext* context)
{
    if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serializeContext->Class<ControlConfiguration>()
            ->Version(1)
            ->Field("Topic", &ControlConfiguration::m_topic)
            ->Field("Qos", &ControlConfiguration::m_qos)
            ->Field("Steering", &ControlConfiguration::m_steering)
            ->Field("BroadcastBusMode", &ControlConfiguration::m_broadcastBusMode)
            ->Field("RobotConfiguration", &ControlConfiguration::m_robotConfiguration)
            ;

        if (AZ::EditContext* ec = serializeContext->GetEditContext())
        {
            ec->Class<ControlConfiguration>("Robot control", "Handles robot control")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ControlConfiguration::m_topic,
                              "Topic", "ROS2 topic to subscribe to")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ControlConfiguration::m_qos, "QoS",
                              "Quality of Service settings for subscriber")
                ->DataElement(AZ::Edit::UIHandlers::ComboBox, &ControlConfiguration::m_steering,
                        "Steering", "Determines how robot is controlled.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->EnumAttribute(ControlConfiguration::Steering::Twist, "Twist")
                    ->EnumAttribute(ControlConfiguration::Steering::Ackermann, "Ackermann")
                ->DataElement(AZ::Edit::UIHandlers::CheckBox, &ControlConfiguration::m_broadcastBusMode,
                              "Broadcast bus mode",
                              "Broadcast messages on a notification bus instead of running built-in solution.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                ->DataElement(AZ::Edit::UIHandlers::Default, &ControlConfiguration::m_robotConfiguration,
                              "Robot configuration",
                              "Description of the robot")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ControlConfiguration::IsBroadcastBusModeDisabled)
            ;
        }
    }

    // Enable twist control notification bus
    TwistNotificationHandler::Reflect(context);
}

AZStd::string ControlConfiguration::GetTopic() const
{
    return m_topic;
}

ControlConfiguration::Steering ControlConfiguration::GetSteering() const
{
    return m_steering;
}

bool ControlConfiguration::IsBroadcastBusMode() const
{
    return m_broadcastBusMode;
}

QoS ControlConfiguration::GetControlTopicQoS() const
{
    return m_qos;
}

RobotConfiguration ControlConfiguration::GetRobotConfiguration() const
{
    return m_robotConfiguration;
}

}  // namespace ROS2

