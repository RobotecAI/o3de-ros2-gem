/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleDynamics/Utilities.h"
#include "WheelControllerComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics::Utilities
{
    AxleConfiguration Create2WheelAxle(
        AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, float wheelRadius, bool steering, bool drive)
    {
        VehicleDynamics::AxleConfiguration axleConfiguration;
        axleConfiguration.m_axleWheels.push_back(leftWheel);
        axleConfiguration.m_axleWheels.push_back(rightWheel);
        axleConfiguration.m_axleTag = AZStd::move(tag);
        axleConfiguration.m_isSteering = steering;
        axleConfiguration.m_isDrive = drive;
        axleConfiguration.m_wheelRadius = wheelRadius;
        return axleConfiguration;
    }

    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Front", wheelRadius, true, true);
    }

    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Rear", wheelRadius, false, true);
    }

    AZStd::vector<VehicleDynamics::SteeringDynamicsData> GetAllSteeringEntitiesData(const ChassisConfiguration& chassisConfig)
    {
        AZStd::vector<VehicleDynamics::SteeringDynamicsData> steeringEntitiesAndAxis;
        for (const auto& axle : chassisConfig.m_axles)
        {
            if (axle.m_isSteering)
            {
                for (const auto& wheel : axle.m_axleWheels)
                {
                    if (!wheel.IsValid())
                    { // TODO - warn
                        continue;
                    }
                    AZ::Entity* wheelEntity = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(wheelEntity, &AZ::ComponentApplicationRequests::FindEntity, wheel);
                    auto* controllerComponent = wheelEntity->FindComponent<WheelControllerComponent>();
                    if (!controllerComponent)
                    { // TODO - warn
                        continue;
                    }

                    AZ::EntityId steeringEntity = controllerComponent->m_steeringEntity;
                    AZ::Vector3 steeringDir = controllerComponent->m_steeringDir;
                    steeringDir.Normalize();
                    if (!steeringEntity.IsValid())
                    { // TODO - warn
                        continue;
                    }

                    VehicleDynamics::SteeringDynamicsData steeringData;
                    steeringData.m_steeringEntity = steeringEntity;
                    steeringData.m_turnAxis = steeringDir;
                    steeringEntitiesAndAxis.push_back(steeringData);
                }
            }
        }
        return steeringEntitiesAndAxis;
    }

    AZStd::vector<VehicleDynamics::WheelDynamicsData> GetAllDriveWheelsData(const ChassisConfiguration& chassisConfig)
    {
        AZStd::vector<VehicleDynamics::WheelDynamicsData> driveWheelEntities;
        for (const auto& axle : chassisConfig.m_axles)
        {
            if (axle.m_isDrive)
            {
                for (const auto& wheel : axle.m_axleWheels)
                {
                    AZ::Entity* wheelEntity = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(wheelEntity, &AZ::ComponentApplicationRequests::FindEntity, wheel);
                    auto* controllerComponent = wheelEntity->FindComponent<WheelControllerComponent>();
                    if (!wheel.IsValid() || !controllerComponent)
                    { // TODO - warn
                        continue;
                    }
                    AZ::Vector3 driveDir = controllerComponent->m_driveDir;
                    driveDir.Normalize();

                    VehicleDynamics::WheelDynamicsData wheelData;
                    wheelData.m_wheelEntity = wheel;
                    wheelData.m_driveAxis = driveDir;
                    wheelData.m_wheelRadius = axle.m_wheelRadius;
                    driveWheelEntities.push_back(wheelData);
                }
            }
        }
        return driveWheelEntities;
    }
} // namespace VehicleDynamics::Utilities
