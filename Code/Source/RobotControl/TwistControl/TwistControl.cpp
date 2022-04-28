/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>

#include "RobotControl/TwistControl/TwistControl.h"
#include "Utilities/ROS2Conversions.h"

namespace ROS2
{
    void TwistControl::BroadcastBus(const geometry_msgs::msg::Twist& message)
    {
        TwistNotificationBus::Broadcast(&TwistNotifications::TwistReceived,
                                        ROS2Conversions::FromROS2Vector3(message.linear),
                                        ROS2Conversions::FromROS2Vector3(message.angular))
                ;
    }

    void TwistControl::ApplyControl(const geometry_msgs::msg::Twist& message)
    {
        const float gravityForce = -9.81;
        auto body = m_controlConfiguration.GetRobotConfiguration().GetBody();

        if (!body.IsValid()) {
            AZ_ErrorOnce("ROS2RobotControl", false, "Invalid body component for twist control. Nothing to move.");
        }

        const AZ::Vector3 linearVelocity = ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2Conversions::FromROS2Vector3(message.angular);

        // Simulate body
        AZ::Transform robotTransform;
        AZ::TransformBus::EventResult(robotTransform, body, &AZ::TransformBus::Events::GetWorldTM);
        auto transformedLinearVelocity = robotTransform.TransformVector(linearVelocity);

        Physics::RigidBodyRequestBus::Event(body, &Physics::RigidBodyRequests::SetLinearVelocity, transformedLinearVelocity);
        Physics::RigidBodyRequestBus::Event(body, &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocity);

        // Simulate gravity
        Physics::RigidBodyRequestBus::Event(body, &Physics::RigidBodyRequests::ApplyLinearImpulse, AZ::Vector3(0, 0, gravityForce));

        // TODO: handle wheels
    }
}  // namespace ROS2