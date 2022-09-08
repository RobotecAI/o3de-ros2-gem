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
#include <AzCore/Serialization/SerializeContext.h>

namespace VehicleDynamics
{
    void AxleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AxleConfiguration, AZ::Component>()
                ->Version(1)
                ->Field("AxleTag", &AxleConfiguration::m_axleTag)
                ->Field("AxleWheels", &AxleConfiguration::m_axleWheels);
            ->Field("IsSteering", &AxleConfiguration::m_isSteering)->Field("IsDrive", &AxleConfiguration::m_isDrive);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AxleConfiguration>("Axle configuration", "Axles of the vehicle model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
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
                        &AxleConfiguration::m_axleWheels,
                        "Axle wheels",
                        "One or more wheels attached to this axle");
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
