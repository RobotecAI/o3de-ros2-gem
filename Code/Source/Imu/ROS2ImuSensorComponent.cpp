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
#include "Utilities/ROS2Conversions.h"

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

        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        m_previousPose = entityTransform->GetWorldTM();
    }

    void ROS2ImuSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_imuPublisher.reset();
    }

    void ROS2ImuSensorComponent::FrequencyTick()
    {
        // Get current pose
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        const auto & currentPose = entityTransform->GetWorldTM();
        const auto & frequency = m_sensorConfiguration.m_frequency;

        // Angular velocity calculations
        const auto & currentRotation = currentPose.GetRotation();
        const auto & lastRotation = m_previousPose.GetRotation();
        const auto deltaRotation = currentRotation * lastRotation.GetInverseFull();
        AZ::Vector3 axis;
        float angle;
        deltaRotation.ConvertToAxisAngle(axis, angle);
        const auto angularVelocity = frequency * angle * axis;

        // Linear acceleration calculations
        const auto & currentPosition = currentPose.GetTranslation();
        const auto & previousPosition = m_previousPose.GetTranslation();
        const auto velocity = (currentPosition - previousPosition) * frequency;
        const auto acceleration = (velocity - m_previousLinearVelocity) * frequency;

        m_previousPose = currentPose;
        m_previousLinearVelocity = velocity;

        // Fill message fields
        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto message = sensor_msgs::msg::Imu();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();

        message.angular_velocity = ROS2Conversions::ToROS2Vector3(angularVelocity);
        message.linear_acceleration = ROS2Conversions::ToROS2Vector3(acceleration);

        // Set neutral orientation
        message.orientation.x = 0.0;
        message.orientation.y = 0.0;
        message.orientation.z = 0.0;
        message.orientation.w = 1.0;

        // Set covariances to 0
        for (auto & e : message.orientation_covariance)
        {
            e = 0.0;
        }

        for (auto & e : message.angular_velocity_covariance)
        {
            e = 0.0;
        }

        for (auto & e : message.linear_acceleration_covariance)
        {
            e = 0.0;
        }

        m_imuPublisher->publish(message);
    }
} // namespace ROS2
