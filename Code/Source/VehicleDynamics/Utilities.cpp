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
    AxleConfiguration Create2WheelAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, bool steering, bool drive)
    {
        VehicleDynamics::AxleConfiguration axleConfiguration;
        axleConfiguration.m_axleWheels.push_back(leftWheel);
        axleConfiguration.m_axleWheels.push_back(rightWheel);
        axleConfiguration.m_axleTag = AZStd::move(tag);
        axleConfiguration.m_isSteering = steering;
        axleConfiguration.m_isDrive = drive;
        return axleConfiguration;
    }

    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Front", true, true);
    }

    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Rear", false, true);
    }

    AZStd::vector<AZ::EntityId> GetAllSteeringEntities(const ChassisConfiguration& chassisConfig)
    {
        AZStd::vector<AZ::EntityId> steeringEntities;
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
                    if (!steeringEntity.IsValid())
                    { // TODO - warn
                        continue;
                    }
                    steeringEntities.push_back(steeringEntity);
                }
            }
        }
        return steeringEntities;
    }

    AZStd::vector<AZ::EntityId> GetAllDriveWheelEntities(const ChassisConfiguration& chassisConfig)
    {
        AZStd::vector<AZ::EntityId> driveWheelEntities;
        for (const auto& axle : chassisConfig.m_axles)
        {
            if (axle.m_isDrive)
            {
                for (const auto& wheel : axle.m_axleWheels)
                {
                    if (!wheel.IsValid())
                    { // TODO - warn
                        continue;
                    }

                    driveWheelEntities.push_back(wheel);
                }
            }
        }
        return driveWheelEntities;
    }
} // namespace VehicleDynamics::Utilities
