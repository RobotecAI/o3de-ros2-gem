/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleDynamics/ChassisConfiguration.h"
#include "VehicleDynamics/Utilities.h"
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace VehicleDynamics
{
    void ChassisConfiguration::Reflect(AZ::ReflectContext* context)
    {
        AxleConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ChassisConfiguration>()->Version(1)->Field("AxlesConfigurations", &ChassisConfiguration::m_axles);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ChassisConfiguration>("Chassis configuration", "Configuration of vehicle chassis")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ChassisConfiguration::m_axles, "Axles", "Configurations of axles for this chassis");
            }
        }
    }
} // namespace VehicleDynamics
