/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleDynamics/ChassisConfiguration.h"
#include "VehicleDynamics/DriveModel.h"
#include "VehicleDynamics/VehicleInputsState.h"
#include <control_toolbox/pid.hpp>

namespace VehicleDynamics
{
    //! A simple implementation converting speed and steering inputs into wheel impulse and steering element torque
    class SimplifiedDriveModel : public DriveModel
    {
    public:
        SimplifiedDriveModel();
        DriveModel::DriveModelType DriveType() override
        {
            return DriveModel::SimplifiedDriveModelType;
        }
        void ApplyInputState(const VehicleInputsState& inputs, const ChassisConfiguration& vehicleChassis, uint64_t deltaTimeNs) override;

    private:
        void ApplySteering(float steering, const ChassisConfiguration& vehicleChassis, uint64_t deltaTimeNs);
        void ApplySpeed(float speed, const ChassisConfiguration& vehicleChassis, uint64_t deltaTimeNs);

        control_toolbox::Pid m_steeringPid;
        control_toolbox::Pid m_speedPid;
    };
} // namespace VehicleDynamics
