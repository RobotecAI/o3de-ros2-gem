/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/Serialization/EditContext.h>

#include "Map/GeodeticConfiguration.h"

namespace ROS2::Map
{
    AZ::Outcome<void, AZStd::string> GeodeticConfiguration::ValidateHandle(void* newValue, [[maybe_unused]] const AZ::Uuid& valueType)
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

        // Check if entity has transform
        AZ::TransformInterface* transformInterface = entity->GetTransform();
        if (transformInterface == nullptr)
        {
            return AZ::Failure(AZStd::string("Entity doesn't have transform component."));
        }

        return AZ::Success();
    }

    void GeodeticConfiguration::SetHook()
    {
        if(m_useMapHook && !m_isMapHookSet) {
            if (!m_mapHook.IsValid()) {
                AZ_Warning("GeodeticConfiguration", false, "Map hook not set. Using identity on (0,0,0).");
            } else {
                AZ::TransformBus::EventResult(
                        m_mapHookTransform,
                        m_mapHook,
                        &AZ::TransformBus::Events::GetWorldTM);
            }
            m_isMapHookSet = true;
        }
    }

    bool GeodeticConfiguration::IsMapHookUsed() const
    {
        return m_useMapHook;
    }

    void GeodeticConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeodeticConfiguration>()
                    ->Version(1)
                    ->Field("useMapHook", &GeodeticConfiguration::m_useMapHook)
                    ->Field("mapHook", &GeodeticConfiguration::m_mapHook)
                    ->Field("originLatitude", &GeodeticConfiguration::m_originLatitudeDeg)
                    ->Field("originLongitude", &GeodeticConfiguration::m_originLongitudeDeg)
                    ->Field("originAltitude", &GeodeticConfiguration::m_originAltitude)

                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<GeodeticConfiguration>("Map configuration", "Map origin configuration")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GeodeticConfiguration::m_useMapHook,
                                      "Use map hook", "Should a map hook be used to position the scene in the world.")
                            ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GeodeticConfiguration::m_mapHook, "Map hook", "Map origin handle.")
                            ->Attribute(AZ::Edit::Attributes::ChangeValidate, &GeodeticConfiguration::ValidateHandle)
                            ->Attribute(AZ::Edit::Attributes::Visibility, &GeodeticConfiguration::IsMapHookUsed)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GeodeticConfiguration::m_originLatitudeDeg,
                                      "Origin latitude", "Latitude position offset in degrees")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GeodeticConfiguration::m_originLongitudeDeg,
                                      "Origin longitude", "Longitude position offset in degrees")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GeodeticConfiguration::m_originAltitude,
                                      "Origin altitude", "Altitude position offset in meters")
                        ;
            }
        }
    }
}