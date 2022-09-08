/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics
{
    void AxleConfiguration::Reflect(AZ::ReflectContext* context)
    {
    }

    AZ::EntityId AxleConfiguration::GetLeftWheelEntityId() const
    {
        if (m_axleWheels.empty())
        {
            return AZ::EntityId();
        }
        return m_axleWheels.front();
    }

    AZ::EntityId AxleConfiguration::GetRightWheelEntityId() const
    {
        if (m_axleWheels.empty())
        {
            return AZ::EntityId();
        }
        return m_axleWheels.back();
    }
} // namespace VehicleDynamics
