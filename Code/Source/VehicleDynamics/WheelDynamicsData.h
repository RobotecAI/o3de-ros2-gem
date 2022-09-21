/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace VehicleDynamics
{
    //! Data structure to pass wheel dynamics data for a single wheel
    struct WheelDynamicsData
    {
        AZ::EntityId m_wheelEntity;
        AZ::Vector3 m_driveAxis;
        float m_wheelRadius;
    };

    struct SteeringDynamicsData
    {
        AZ::EntityId m_steeringEntity;
        AZ::Vector3 m_turnAxis;
    };
} // namespace VehicleDynamics
