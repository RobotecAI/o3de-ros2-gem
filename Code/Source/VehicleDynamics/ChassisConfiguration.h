/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/collections/vector.h>
#include <AzFramework/Physics/Material/PhysicsMaterial.h>

namespace VehicleDynamics
{
    //! Drive and steering configuration for a vehicle
    class ChassisConfiguration
    {
    public:
        ChassisConfiguration() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! Axles of the vehicle, front to rear
        AZStd::vector<AxleConfiguration> m_axles;
        Physics::Material m_tireMaterial; //! Can be overwritten for each wheel
    };
} // namespace VehicleDynamics
