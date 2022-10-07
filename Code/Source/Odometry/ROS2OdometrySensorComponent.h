/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include "Sensor/ROS2SensorComponent.h"
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2
{
    //! An Odometry sensor Component.
    //! Odometry message contains information about vehicle velocity and position in space.
    class ROS2OdometrySensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2OdometrySensorComponent, "{61387448-63AA-4563-AF87-60C72B05B863}", ROS2SensorComponent);
        ROS2OdometrySensorComponent();
        ~ROS2OdometrySensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        void Activate() override;
        void Deactivate() override;

    private:
        void FrequencyTick() override;

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
    };
} // namespace ROS2
