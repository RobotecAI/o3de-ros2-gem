/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannSubscriptionHandler.h"
#include "AckermannBus.h"
#include "AckermannCommandStruct.h"
#include "VehicleDynamics/InputControlBus.h"

namespace ROS2
{
    void AckermannSubscriptionHandler::BroadcastBus(const ackermann_msgs::msg::AckermannDrive& message)
    {
        AckermannCommandStruct acs;
        acs.m_acceleration = message.acceleration;
        acs.m_jerk = message.jerk;
        acs.m_speed = message.speed;
        acs.m_steeringAngle = message.steering_angle;
        acs.m_steeringAngleVelocity = message.steering_angle_velocity;
        AckermannNotificationBus::Broadcast(&AckermannNotifications::AckermannReceived, acs);
    }
} // namespace ROS2
