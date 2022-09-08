/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include <AzCore/std/string/string.h>

namespace VehicleDynamics::Utilities
{
    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel)
    {
        ROS2::AxleConfiguration axleConfiguration;
    }

    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel)
    {
        ROS2::AxleConfiguration axleConfiguration;
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

                    AZ::Entity* wheelEntity = AzToolsFramework::GetEntityById(wheel);
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
