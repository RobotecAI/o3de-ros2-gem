/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ControlConfiguration.h"
#include "Joy/JoyBus.h"
#include "ROS2/ROS2Bus.h"
#include "rclcpp/rclcpp.hpp"
#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <sensor_msgs/msg/joy.hpp>

namespace ROS2
{

    class ROS2JoySubscriberComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2JoySubscriberComponent, "{528D8675-F2A2-42ED-AC04-21F7C8CD9B13}", AZ::Component);
        ROS2JoySubscriberComponent() = default;

        // AZ::Component interface implementation.
        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    };
} // namespace ROS2
