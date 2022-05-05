/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Imu/ROS2ImuSensorComponent.h"
#include "Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Names.h"

#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void ROS2ImuSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2ImuSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2ImuSensorComponent>("ROS2 Imu Sensor", "Imu sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                ;
            }
        }
    }

    ROS2ImuSensorComponent::ROS2ImuSensorComponent()
    {
        auto pc = AZStd::make_shared<PublisherConfiguration>();
        auto type = "sensor_msgs::msg::Imu";
        pc->m_type = type;
        pc->m_topic = "imu";
        m_sensorConfiguration.m_frequency = 50;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }

    void ROS2ImuSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for IMU sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations["sensor_msgs::msg::Imu"];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig->m_topic);
        m_imuPublisher = ros2Node->create_publisher<sensor_msgs::msg::Imu>(fullTopic.data(), publisherConfig->GetQoS());
    }

    void ROS2ImuSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_imuPublisher.reset();
    }

    void ROS2ImuSensorComponent::FrequencyTick()
    {
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        AZ::Vector3 start = entityTransform->GetWorldTM().GetTranslation();
        start.SetZ(start.GetZ() + 1.0f);

        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto message = sensor_msgs::msg::Imu();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();

        // TODO:: fill message fields
        // message.orientation = ;
        // message.orientation_covariance = ;
        // message.angular_velocity = ;
        // message.angular_velocity_covariance = ;
        // message.linear_acceleration = ;
        // message.linear_acceleration_covariance = ;

        m_imuPublisher->publish(message);
    }
} // namespace ROS2
