/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/Serialization/EditContext.h>

#include "Map/MapConfiguration.h"

namespace ROS2
{
    AZ::Outcome<void, AZStd::string> MapConfiguration::ValidateHandle(void* newValue, [[maybe_unused]] const AZ::Uuid& valueType)
    {
        // Check if the object type is valid
        if (azrtti_typeid<AZ::EntityId>() != valueType)
        {
            AZ_Assert(false, "Unexpected value type");
            return AZ::Failure(AZStd::string("Trying to set an entity ID to something that isn't an entity ID."));
        }

        // Check if entity id is valid
        AZ::EntityId actualEntityId = *reinterpret_cast<AZ::EntityId*>(newValue);
        if (!actualEntityId.IsValid())
        {
            return AZ::Failure(AZStd::string("Invalid entity ID."));
        }

        // Check if entity exists
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, actualEntityId);
        if (entity == nullptr)
        {
            return AZ::Failure(AZStd::string("Can't find entity."));
        }

        return AZ::Success();
    }

    bool MapConfiguration::IsMapHookUsed() const
    {
        return m_useMapHook;
    }

    void MapConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapConfiguration>()
                    ->Version(1)
                    ->Field("useMapHook", &MapConfiguration::m_useMapHook)
                    ->Field("mapHook", &MapConfiguration::m_mapHook)
                    ->Field("originLatitude", &MapConfiguration::m_originLatitudeDeg)
                    ->Field("originLongitude", &MapConfiguration::m_originLongitudeDeg)
                    ->Field("originAltitude", &MapConfiguration::m_originAltitude)

                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapConfiguration>("Map configuration", "Map origin configuration")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MapConfiguration::m_useMapHook,
                                      "Use map hook", "Should a map hook be used to position the scene in the world.")
                            ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MapConfiguration::m_mapHook, "Map hook", "Map origin handle.")
                            ->Attribute(AZ::Edit::Attributes::ChangeValidate, &MapConfiguration::ValidateHandle)
                            ->Attribute(AZ::Edit::Attributes::Visibility, &MapConfiguration::IsMapHookUsed)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MapConfiguration::m_originLatitudeDeg,
                                      "Origin latitude", "Latitude position offset in degrees")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MapConfiguration::m_originLongitudeDeg,
                                      "Origin longitude", "Longitude position offset in degrees")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MapConfiguration::m_originAltitude,
                                      "Origin altitude", "Altitude position offset in meters")
                        ;
            }
        }
    }
}