/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannDriveModel.h"
#include "VehicleDynamics/Utilities.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <HingeJointComponent.h>

namespace VehicleDynamics
{
    void AckermannDriveModel::Reflect(AZ::ReflectContext* context)
    {
        PidConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AckermannDriveModel, DriveModel>()
                ->Version(2)
                ->Field("SteeringPID", &AckermannDriveModel::m_steeringPid);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannDriveModel>("Simplified Drive Model", "Configuration of a simplified vehicle dynamics drive model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AckermannDriveModel::m_steeringPid,
                        "Steering PID",
                        "Configuration of steering PID controller");
            }
        }
    }

    void AckermannDriveModel::Activate(const VehicleConfiguration& vehicleConfig)
    {
        m_driveWheelsData.clear();
        m_steeringData.clear();
        m_vehicleConfiguration = vehicleConfig;
        m_steeringPid.InitializePid();
    }

    void AckermannDriveModel::ApplyInputState(const VehicleInputsState& inputs, uint64_t deltaTimeNs)
    {
        if (m_driveWheelsData.empty())
        {
            m_driveWheelsData = VehicleDynamics::Utilities::GetAllDriveWheelsData(m_vehicleConfiguration);
        }

        if (m_steeringData.empty())
        {
            m_steeringData = VehicleDynamics::Utilities::GetAllSteeringEntitiesData(m_vehicleConfiguration);
        }

        ApplySteering(inputs.m_steering.GetValue(), deltaTimeNs);
        ApplySpeed(inputs.m_speed.GetValue(), deltaTimeNs);
    }

    void AckermannDriveModel::ApplyWheelSteering(SteeringDynamicsData& wheelData, float steering, double deltaTimeNs)
    {
        const auto& steeringEntity = wheelData.m_steeringEntity;
        const auto& hingeComponent = wheelData.m_hingeJoint;

        auto id = AZ::EntityComponentIdPair(steeringEntity, hingeComponent);

        PhysX::JointRequestBus::Event(
            id,
            [&](PhysX::JointRequests* joint)
            {
                double  currentSteeringAngle = joint->GetPosition();
                const double pidCommand = m_steeringPid.ComputeCommand(steering - currentSteeringAngle, deltaTimeNs);
                PhysX::JointRequestBus::EventResult(currentSteeringAngle, id, &PhysX::JointRequests::GetPosition);
                joint->SetVelocity(pidCommand);
            });


    }

    // TODO - speed and steering handling is quite similar, possible to refactor?
    void AckermannDriveModel::ApplySteering(float steering, uint64_t deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
        }
        if (m_steeringData.empty())
        {
            AZ_Warning("ApplySteering", false, "Cannot apply steering since no steering elements are defined in the model");
            return;
        }

        auto innerSteering = atan(
            (m_vehicleConfiguration.m_wheelbase * tan(steering)) /
            (m_vehicleConfiguration.m_wheelbase - 0.5 * m_vehicleConfiguration.m_track * tan(steering)));
        auto outerSteering = atan(
            (m_vehicleConfiguration.m_wheelbase * tan(steering)) /
            (m_vehicleConfiguration.m_wheelbase + 0.5 * m_vehicleConfiguration.m_track * tan(steering)));

        ApplyWheelSteering(m_steeringData.front(), innerSteering, deltaTimeNs);
        ApplyWheelSteering(m_steeringData.back(), outerSteering, deltaTimeNs);
    }

    void AckermannDriveModel::ApplySpeed(float speed, uint64_t deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
        }
        if (m_driveWheelsData.empty())
        {
            AZ_Warning("ApplySpeed", false, "Cannot apply speed since no diving wheels are defined in the model");
            return;
        }

        for (const auto& wheelData : m_driveWheelsData)
        {

            const auto wheelEntity = wheelData.m_wheelEntity;
            const float speedScaling = wheelData.m_velocityScale;
            float wheelRadius = wheelData.m_wheelRadius;
            const auto hingeComponent = wheelData.m_hingeJoint;
            const auto id = AZ::EntityComponentIdPair(wheelEntity, hingeComponent);
            auto desiredAngularSpeedX = speedScaling*(speed / wheelRadius);
            PhysX::JointRequestBus::Event(id, &PhysX::JointRequests::SetVelocity, desiredAngularSpeedX);

        }
    }

    void AckermannDriveModel::SetDisabled(bool is_disabled)
    {
        m_disabled = is_disabled;
    }

} // namespace VehicleDynamics
