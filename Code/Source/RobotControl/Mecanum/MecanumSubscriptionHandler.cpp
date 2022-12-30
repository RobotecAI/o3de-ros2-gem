/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "MecanumSubscriptionHandler.h"
#include "ROS2/RobotControl/Mecanum/MecanumBus.h"
#include "ROS2/Utilities/ROS2Conversions.h"

namespace ROS2
{
    void MecanumSubscriptionHandler::SendToBus(const geometry_msgs::msg::Twist& message)
    {
        const AZ::Vector3 linearVelocity = ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2Conversions::FromROS2Vector3(message.angular);
        MecanumNotificationBus::Event(GetEntityId(), &MecanumNotifications::TwistReceived, linearVelocity, angularVelocity);
    }
} // namespace ROS2
