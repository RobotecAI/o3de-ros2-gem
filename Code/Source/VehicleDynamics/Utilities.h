/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include "ChassisConfiguration.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics::Utilities
{
    AxleConfiguration Create2WheelAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, bool steering, bool drive);
    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel);
    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel);

    // TODO - run and cache on Activate
    AZStd::vector<AZStd::pair<AZ::EntityId, AZ::Vector3>> GetAllSteeringEntitiesAndAxes(const ChassisConfiguration& chassisConfig);
    AZStd::vector<AZStd::pair<AZ::EntityId, AZ::Vector3>> GetAllDriveWheelEntitiesAndAxes(const ChassisConfiguration& chassisConfig);
} // namespace VehicleDynamics::Utilities
