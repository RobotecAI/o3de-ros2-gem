/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include "Sensor/ROS2SensorComponent.h"

namespace ROS2
{
    class ROS2ImuSensorComponent
        : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, "{502A955E-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2ImuSensorComponent();
        ~ROS2ImuSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        void Activate() override;
        void Deactivate() override;

    private:
        void FrequencyTick() override;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;

        double m_previousTime;
        AZ::Quaternion m_previousRotation;
        AZ::Vector3 m_previousLocalPosition;
        AZ::Vector3 m_previousLinearVelocity;
    };
}  // namespace ROS2
