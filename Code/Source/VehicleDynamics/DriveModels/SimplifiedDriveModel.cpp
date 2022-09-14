/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimplifiedDriveModel.h"
#include "VehicleDynamics/Utilities.h"
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace VehicleDynamics
{
    SimplifiedDriveModel::SimplifiedDriveModel()
    {
        // TODO - make this a part of exposed configuration
        m_speedPid.initPid(50.0, 0.0, 0.0, 100.0, -100.0);
        m_steeringPid.initPid(50.0, 0.0, 0.0, 100.0, -100.0);
    }

    void SimplifiedDriveModel::ApplyInputState(const VehicleInputsState& inputs, const ChassisConfiguration& vehicleChassis, uint64_t nsDt)
    {
        ApplySteering(inputs.m_steering, vehicleChassis, nsDt);
        ApplySpeed(inputs.m_speed, vehicleChassis, nsDt);
    }

    // TODO - speed and steering handling is quite similar, possible to refactor?
    void SimplifiedDriveModel::ApplySteering(float steering, const ChassisConfiguration& vehicleChassis, uint64_t nsDt)
    {
        auto steeringEntities = VehicleDynamics::Utilities::GetAllSteeringEntities(vehicleChassis);
        for (auto& steeringEntity : steeringEntities)
        {
            AZ::Vector3 currentSteeringElementRotation;
            AZ::TransformBus::EventResult(currentSteeringElementRotation, steeringEntity, &AZ::TransformBus::Events::GetLocalRotation);
            auto currentSteeringAngle = currentSteeringElementRotation.GetZ(); // TODO - axis could be different
            double pidCommand = m_steeringPid.computeCommand(steering - currentSteeringAngle, nsDt);
            AZ_TracePrintf(
                "SimplifiedDriveModel",
                "Applying steering to entity %s with values: desired %f, current %f, impulse %lf\n",
                steeringEntity.ToString().c_str(),
                steering,
                currentSteeringAngle,
                pidCommand);

            if (AZ::IsClose(pidCommand, 0.0)) // TODO - use the third argument with some reasonable value which means "close enough"
            {
                continue;
            }

            auto torque = pidCommand;

            AZ::Transform steeringElementTransform;
            AZ::TransformBus::EventResult(steeringElementTransform, steeringEntity, &AZ::TransformBus::Events::GetWorldTM);
            auto transformedTorqueVector = steeringElementTransform.TransformVector(AZ::Vector3(0, 0, torque));
            Physics::RigidBodyRequestBus::Event(steeringEntity, &Physics::RigidBodyRequests::ApplyAngularImpulse, transformedTorqueVector);
        }
    }

    void SimplifiedDriveModel::ApplySpeed(float speed, const ChassisConfiguration& vehicleChassis, uint64_t nsDt)
    {
        auto wheelEntities = VehicleDynamics::Utilities::GetAllDriveWheelEntities(vehicleChassis);
        for (auto& wheelEntity : wheelEntities)
        {
            AZ::Transform wheelTransform;
            AZ::TransformBus::EventResult(wheelTransform, wheelEntity, &AZ::TransformBus::Events::GetWorldTM);

            AZ::Transform inverseWheelTransform = wheelTransform.GetInverse();
            AZ::Vector3 currentAngularVelocity;
            Physics::RigidBodyRequestBus::EventResult(currentAngularVelocity, wheelEntity, &Physics::RigidBodyRequests::GetAngularVelocity);
            currentAngularVelocity = inverseWheelTransform.TransformVector(currentAngularVelocity);
            auto currentAngularSpeedX = currentAngularVelocity.GetX();

            const double wheelRadius = 0.325f; // TODO - get from the actual wheel
            auto desiredAngularSpeedX = speed / wheelRadius;

            double pidCommand = m_speedPid.computeCommand(desiredAngularSpeedX - currentAngularSpeedX, nsDt);
            AZ_TracePrintf(
                "SimplifiedDriveModel",
                "Applying angular speed to wheel entity %s with values: desired %f, current %f, impulse %lf\n",
                wheelEntity.ToString().c_str(),
                desiredAngularSpeedX,
                currentAngularSpeedX,
                pidCommand);

            if (AZ::IsClose(pidCommand, 0.0)) // TODO - use the third argument with some reasonable value which means "close enough"
            {
                continue;
            }

            auto impulse = pidCommand;

            auto transformedTorqueVector = wheelTransform.TransformVector(AZ::Vector3(impulse, 0, 0));
            Physics::RigidBodyRequestBus::Event(wheelEntity, &Physics::RigidBodyRequests::ApplyAngularImpulse, transformedTorqueVector);
        }
    }
} // namespace VehicleDynamics
