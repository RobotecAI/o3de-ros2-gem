/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AxleConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace VehicleDynamics
{
    void AxleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AxleConfiguration>()
                ->Version(2)
                ->Field("AxleTag", &AxleConfiguration::m_axleTag)
                ->Field("AxleWheels", &AxleConfiguration::m_axleWheels)
                ->Field("WheelRadius", &AxleConfiguration::m_wheelRadius)
                ->Field("IsSteering", &AxleConfiguration::m_isSteering)
                ->Field("IsDrive", &AxleConfiguration::m_isDrive);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AxleConfiguration>("Axle configuration", "Axles of the vehicle model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &AxleConfiguration::m_axleTag, "Axle tag", "Helpful description of the axle")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AxleConfiguration::m_isSteering,
                        "Is it a steering axle",
                        "Is this axle used for steering (all attached wheels)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AxleConfiguration::m_isDrive,
                        "Is it a drive axle",
                        "Is this axle used for drive (all attached wheels)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AxleConfiguration::m_wheelRadius,
                        "Wheel radius",
                        "Radius of each wheel attached to axle")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AxleConfiguration::m_axleWheels,
                        "Axle wheels",
                        "One or more wheels attached to this axle. First wheel is the leftmost, last is the rightmost");
            }
        }
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
