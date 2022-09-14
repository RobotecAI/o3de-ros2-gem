/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ChassisConfiguration.h"
#include "VehicleInputsState.h"

namespace VehicleDynamics
{
    //! Abstract class for turning vehicle inputs into behavior of chassis elements (wheels, steering elements)
    class DriveModel
    {
    public:
        enum DriveModelType
        {
            SimplifiedDriveModelType
        };

        virtual ~DriveModel() = default;
        virtual DriveModel::DriveModelType DriveType() = 0;
        virtual void ApplyInputState(
            const VehicleInputsState& inputs, const ChassisConfiguration& vehicleChassis, uint64_t deltaTimeNs) = 0;
    };
} // namespace VehicleDynamics
