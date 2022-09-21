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
#include <AzCore/Serialization/SerializeContext.h>

namespace VehicleDynamics
{
    //! Abstract class for turning vehicle inputs into behavior of chassis elements (wheels, steering elements)
    class DriveModel
    {
    public:
        AZ_RTTI(DriveModel, "{1B57E83D-19BF-4403-8712-1AE98A12F0CD}");
        enum DriveModelType
        {
            SimplifiedDriveModelType
        };

        static void Reflect(AZ::ReflectContext* context);
        virtual ~DriveModel() = default;
        virtual DriveModel::DriveModelType DriveType() = 0;
        virtual void Activate(const ChassisConfiguration& vehicleChassis) = 0;
        virtual void ApplyInputState(const VehicleInputsState& inputs, uint64_t deltaTimeNs) = 0;
    };
} // namespace VehicleDynamics
