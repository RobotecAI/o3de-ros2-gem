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
    //! The most recent inputs
    struct VehicleInputsState
    {
        float m_speed = 0.0f; // Mps
        float m_steering = 0.0f; // Radians, right is +, left is -
    };
} // namespace VehicleDynamics
