/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Map/MapManagerComponent.h"
#include "Map/GeodeticTransforms.h"
#include "Utilities/ROS2Names.h"

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

    void MapManagerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MapManager"));
    }

    void MapManagerComponent::Reflect(AZ::ReflectContext *context)
    {
        Map::GeodeticConfiguration::Reflect(context);
        Map::SpawnPointsConfiguration::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapManagerComponent, AZ::Component>()
                    ->Version(1)
                    ->Field("MapFrameId", &MapManagerComponent::m_mapFrameId)
                    ->Field("OdomFrameId", &MapManagerComponent::m_odomFrameId)
                    ->Field("GeodeticConfiguration", &MapManagerComponent::m_geodeticConfiguration)
                    ->Field("SpawnPointsConfiguration", &MapManagerComponent::m_spawnPointsConfiguration)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapManagerComponent>("Map manager", "Map manager component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_mapFrameId, "Map frame name", "Frame name used for a map")
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateNamespaceField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_odomFrameId, "Odom frame name", "Frame name used for a robots odometry")
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateNamespaceField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_geodeticConfiguration, "Geodetic", "Geodetic configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_spawnPointsConfiguration, "Spawn points", "Spawn points configuration")
                        ;
            }
        }
    }

    AZStd::vector<AZ::Transform> MapManagerComponent::GetAvailableSpawnPoints() {
        auto spawnPointsTransforms = m_spawnPointsConfiguration.GetAvailableSpawnPointsTransforms();
        for (auto & spawnPointTransform : spawnPointsTransforms)
        {
            spawnPointTransform = m_geodeticConfiguration.m_mapHookTransform.GetInverse() * spawnPointTransform;
        }
        return spawnPointsTransforms;
    }

    AZ::Vector3 MapManagerComponent::LocalToLatLon(const AZ::Vector3 &local) {
        if(m_geodeticConfiguration.m_useMapHook && !m_geodeticConfiguration.m_isMapHookSet) {
            if (!m_geodeticConfiguration.m_mapHook.IsValid()) {
                AZ_Warning("MapManager", false, "Map hook not set. Using identity on (0,0,0).");
            } else {
                AZ::TransformBus::EventResult(
                        m_geodeticConfiguration.m_mapHookTransform,
                        m_geodeticConfiguration.m_mapHook,
                        &AZ::TransformBus::Events::GetWorldTM);
            }
            m_geodeticConfiguration.m_isMapHookSet = true;
        }

        AZ::Transform localInMapHookFrame =
                m_geodeticConfiguration.m_mapHookTransform.GetInverse() * AZ::Transform(local, AZ::Quaternion::CreateIdentity(), 1.0);

        const AZ::Vector3 currentPositionECEF =
            Map::Utilities::ENUToECEF(
                    {m_geodeticConfiguration.m_originLatitudeDeg,
                     m_geodeticConfiguration.m_originLongitudeDeg,
                     m_geodeticConfiguration.m_originAltitude},
                    localInMapHookFrame.GetTranslation());
        return Map::Utilities::ECEFToWGS84(currentPositionECEF);
    }
}