/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleDynamics/VehicleModelComponent.h"
#include "VehicleDynamics/ChassisConfiguration.h"
#include "VehicleDynamics/DriveModels/SimplifiedDriveModel.h"
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
        m_driveModel = AZStd::make_unique<SimplifiedDriveModel>();
        VehicleInputControlRequestBus::Handler::BusConnect();
        m_manualControlEventHandler.Activate();
        AZ::TickBus::Handler::BusConnect();
    }

    void VehicleModelComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_manualControlEventHandler.Deactivate();
        VehicleInputControlRequestBus::Handler::BusDisconnect();
        m_driveModel.reset();
    }

    void VehicleModelComponent::Reflect(AZ::ReflectContext* context)
    {
        ChassisConfiguration::Reflect(context);
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
        provided.push_back(AZ_CRC_CE("VehicleModelService"));
    }

    void VehicleModelComponent::SetTargetLinearSpeed(float speedMps)
    {
        // TODO - add a timeout to set to zero on no inputs. Determine best place.
        m_inputsState.m_speed = speedMps;
        m_accumulatedTimeoutSpeed = 0;
        AZ_TracePrintf("SetTargetLinearSpeed", "Setting speed to %f\n", speedMps);
    }

    void VehicleModelComponent::SetTargetAcceleration([[maybe_unused]] float acceleration)
    {
        AZ_Error("SetTargetAcceleration", false, "Not implemented");
    }

    void VehicleModelComponent::SetTargetSteering(float steering)
    {
        // TODO - add a timeout to set to zero on no inputs. Determine best place.
        m_inputsState.m_steering = steering;
        m_accumulatedTimeoutSteering = 0;
        AZ_TracePrintf("SetTargetSteering", "Setting steering to %f\n", steering);
    }

    void VehicleModelComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // TODO - expose the const for timeouts and handle them in generic way for each input (no repetitions)
        m_accumulatedTimeoutSpeed += deltaTime;
        m_accumulatedTimeoutSteering += deltaTime;
        const float zeroOutInputsThreshold = 0.5f; // 0.5 second without fresh input is considered loss of control, stop.
        if (m_accumulatedTimeoutSpeed > zeroOutInputsThreshold)
        {
            m_inputsState.m_speed = 0.0f;
            m_accumulatedTimeoutSpeed = zeroOutInputsThreshold;
        }
        if (m_accumulatedTimeoutSteering > zeroOutInputsThreshold)
        {
            m_inputsState.m_steering = 0.0f;
            m_accumulatedTimeoutSteering = zeroOutInputsThreshold;
        }

        uint64_t deltaTimeNs = deltaTime * 1000000000;
        m_driveModel->ApplyInputState(m_inputsState, m_chassisConfiguration, deltaTimeNs);
    }

} // namespace VehicleDynamics
