/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/Material/PhysicsMaterial.h>

namespace VehicleDynamics
{
    //! Drive and steering configuration for a vehicle
    class ChassisConfiguration
    {
    public:
        AZ_TYPE_INFO(ChassisConfiguration, "{C616E333-E618-4E37-8CE6-1E8A28182D00}");
        ChassisConfiguration() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! Axles of the vehicle, front to rear
        AZStd::vector<AxleConfiguration> m_axles;
    };
} // namespace VehicleDynamics
