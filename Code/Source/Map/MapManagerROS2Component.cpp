/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Map/MapBus.h"
#include "Map/MapManagerROS2Component.h"
#include "Utilities/ROS2Names.h"
#include "Utilities/ROS2Conversions.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Math/Transform.h>

namespace ROS2
{
    void MapManagerROS2Component::Activate()
    {
    }

    void MapManagerROS2Component::Deactivate()
    {
    }

    MapManagerROS2Component::MapManagerROS2Component()
    {
    }

    MapManagerROS2Component::~MapManagerROS2Component()
    {
    }

    void MapManagerROS2Component::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("MapManager"));
    }

    void MapManagerROS2Component::Reflect(AZ::ReflectContext *context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapManagerROS2Component, AZ::Component>()
                    ->Version(1)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapManagerROS2Component>("ROS2 Map manager interface",
                    "ROS2 interface wrapper for map component")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ;
            }
        }
    }
}