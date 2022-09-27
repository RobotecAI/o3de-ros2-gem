/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/ROS2JoySubscriberComponent.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2JoySubscriberComponent::Activate()
    {
        if (!joy_sub)
        {
            AZ_Printf("Joy", "Subscribing to joy\n");
            auto ros2Node = ROS2Interface::Get()->GetNode();
            joy_sub = ros2Node->create_subscription<sensor_msgs::msg::Joy>(
                "/joy",
                10,
                [](const sensor_msgs::msg::Joy& message)
                {
                    AZStd::array<float, 6> axes{ 0.f };
                    AZStd::array<bool, 8> button{ false };

                    for (int i = 0; i < axes.size() && i < message.axes.size(); i++)
                    {
                        axes[i] = message.axes[i];
                    }

                    bool b = false;
                    for (int i = 0; i < button.size() && i < message.buttons.size(); i++)
                    {
                        button[i] = message.buttons[i];
                        b = b | message.buttons[i];
                    }

                    JoyNotificationBus::Broadcast(
                        &JoyNotifications::JoyReceived,
                        axes[0],
                        axes[1],
                        axes[2],
                        axes[3],
                        axes[4],
                        axes[5],
                        button[0],
                        button[1],
                        button[2],
                        button[3],
                        button[4],
                        button[5],
                        button[6],
                        button[7]);
                    if (b)
                    {
                        AZ_Printf("Joy", "JOY recv : %f %f %f %f\n", message.axes[0], message.axes[1], message.axes[2], message.axes[3]);
                    }
                });
        }
    }

    void ROS2JoySubscriberComponent::Deactivate()
    {
        joy_sub = nullptr;
    }

    void ROS2JoySubscriberComponent::Reflect(AZ::ReflectContext* context)
    {
        ControlConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2JoySubscriberComponent, AZ::Component>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2JoySubscriberComponent>("ROS2JoySubscriberComponent", "ROS2JoySubscriberComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
            }
        }
        // Enable twist control notification bus
        //       TwistNotificationHandler::Reflect(context);
        JoyNotificationHandler::Reflect(context);
    }

    void ROS2JoySubscriberComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        //       required.push_back(AZ_CRC("VehicleModelService"));
    }

    void ROS2JoySubscriberComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        // provided.push_back(AZ_CRC_CE("ROS2RobotControl"));
    }
} // namespace ROS2
