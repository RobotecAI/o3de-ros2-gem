/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include "VehicleConfiguration.h"
#include "WheelDynamicsData.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics::Utilities
{
    AxleConfiguration Create2WheelAxle(
        AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, float wheelRadius, bool steering, bool drive);
    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius);
    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius);
    AZStd::vector<VehicleDynamics::SteeringDynamicsData> GetAllSteeringEntitiesData(const VehicleConfiguration& vehicleConfig);
    AZStd::vector<VehicleDynamics::WheelDynamicsData> GetAllDriveWheelsData(const VehicleConfiguration& vehicleConfig);
} // namespace VehicleDynamics::Utilities
