/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include <AzCore/std/collections/vector.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics::Utilities
{
    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel);
    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel);

    // TODO - run and cache on Activate
    AZStd::vector<AZ::EntityId> GetAllSteeringEntities(const ChassisConfiguration& chassisConfig);
    AZStd::vector<AZ::EntityId> GetAllDriveWheelEntities(const ChassisConfiguration& chassisConfig);
} // namespace VehicleDynamics::Utilities
