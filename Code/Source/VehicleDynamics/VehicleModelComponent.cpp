/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleDynamics/VehicleModelComponent.h"
#include "VehicleDynamics/Utilities.h"
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace VehicleDynamics
{
    void VehicleModelComponent::Activate()
    {
    }

    void VehicleModelComponent::Deactivate()
    {
    }

    void VehicleModelComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VehicleModelComponent, AZ::Component>()->Version(1)->Field(
                "ChassisConfiguration", &VehicleModelComponent::m_chassisConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleModelComponent>("Vehicle Model", "Customizable vehicle model component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_chassisConfiguration,
                        "Chassis settings",
                        "Chassis settings including axles and common wheel parameters");
            }
        }
    }

    void VehicleModelComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // TODO - determine required services
    }

    void VehicleModelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("VehicleModel"));
    }

    void VehicleModelComponent::SetTargetLinearSpeed(float speedMps)
    {
        auto allDriveWheels = VehicleDynamics::Utilities::GetAllDriveWheelEntities(m_chassisConfiguration);

        // TODO - placeholder implementation - replace with a selection of algorithms
        for (auto driveWheelEntityId : allDriveWheels)
        {
            if (!driveWheelEntityId.IsValid())
            {
                continue;
            }

            Physics::RigidBodyRequestBus::Event(
                driveWheelEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, AZ::Vector3(speedMps, 0, 0));
        }
    }

    void VehicleModelComponent::SetTargetAcceleration([[maybe_unused]] float acceleration)
    {
        AZ_Error("SetTargetAcceleration", false, "Not implemented");
    }

    void VehicleModelComponent::SetTargetSteering([[maybe_unused]] float steering)
    {
        AZ_Error("SetTargetSteering", false, "Not implemented");
    }
} // namespace VehicleDynamics
