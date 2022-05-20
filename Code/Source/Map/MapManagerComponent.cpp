/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Map/MapManagerComponent.h"
#include "Map/GeodeticTransforms.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>

#include <AzCore/Debug/Trace.h>
#include <AzCore/Math/MathStringConversions.h>

namespace ROS2
{

    void MapManagerComponent::Activate()
    {
        MapRequestBus::Handler::BusConnect();
    }

    void MapManagerComponent::Deactivate()
    {
        MapRequestBus::Handler::BusDisconnect();
    }

    MapManagerComponent::MapManagerComponent()
    {
    }

    MapManagerComponent::~MapManagerComponent()
    {
    }

    void MapManagerComponent::Reflect(AZ::ReflectContext *context)
    {
        MapConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapManagerComponent, AZ::Component>()
                    ->Version(1)
                    ->Field("Map configuration", &MapManagerComponent::m_mapConfiguration)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapManagerComponent>("ROS2 Map", "Map configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_mapConfiguration, "Map configuration", "Map configuration")
                        ;
            }
        }
    }

    AZ::Vector3 MapManagerComponent::LocalToLatLon(const AZ::Vector3 &local) {
        if(m_mapConfiguration.m_useMapHook && !m_mapConfiguration.m_isMapHookSet) {
            if (!m_mapConfiguration.m_mapHook.IsValid()) {
                AZ_Warning("MapManager", false, "Map hook not set. Using identity on (0,0,0).");
            } else {
                AZ::TransformBus::EventResult(
                        m_mapConfiguration.m_mapHookTransform,
                        m_mapConfiguration.m_mapHook,
                        &AZ::TransformBus::Events::GetWorldTM);
            }
            m_mapConfiguration.m_isMapHookSet = true;
        }

        AZ::Transform localInMapHookFrame =
                m_mapConfiguration.m_mapHookTransform.GetInverse() * AZ::Transform(local, AZ::Quaternion::CreateIdentity(), 1.0);

        const AZ::Vector3 currentPositionECEF =
            Utilities::GeodeticTransforms::ENUToECEF(
                    {m_mapConfiguration.m_originLatitudeDeg,
                     m_mapConfiguration.m_originLongitudeDeg,
                     m_mapConfiguration.m_originAltitude},
                    localInMapHookFrame.GetTranslation());
        return Utilities::GeodeticTransforms::ECEFToWGS84(currentPositionECEF);
    }
}