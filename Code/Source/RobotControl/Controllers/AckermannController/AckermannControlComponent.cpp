/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/TwistControl/TwistBus.h"
#include "RobotControl/TwistControl/TwistControl.h"
#include "Utilities/ROS2Conversions.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void AckermannControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannControlComponent>("Ackermann Control", "Relays Ackermann commands to vehicle inputs")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
            }
        }
    }

    void AckermannControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2RobotControl");
        required.push_back(AZ_CRC_CE("VehicleModel"));
    }

    void AckermannControlComponent::AckermannReceived(const AckermannCommandStruct& angular)
    {
        // Notify input system for vehicle dynamics. Only speed and steering is currently supported.
        VehicleDynamics::InputControlBus::Broadcast(&VehicleDynamics::InputControlRequests::SetTargetLinearSpeed, acs.m_speed);
        VehicleDynamics::InputControlBus::Broadcast(&VehicleDynamics::InputControlRequests::SetTargetSteering, acs.m_steering);
    }
} // namespace ROS2
