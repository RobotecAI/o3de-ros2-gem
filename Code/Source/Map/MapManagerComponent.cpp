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
        if (MapRequestInterface::Get() == nullptr)
        {
            MapRequestInterface::Register(this);
        }
    }

    MapManagerComponent::~MapManagerComponent()
    {
        if (MapRequestInterface::Get() == this)
        {
            MapRequestInterface::Unregister(this);
        }
    }

    void MapManagerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MapManager"));
    }

    void MapManagerComponent::Reflect(AZ::ReflectContext *context)
    {
        Map::GeodeticConfiguration::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapManagerComponent, AZ::Component>()
                    ->Version(1)
                    ->Field("MapFrameId", &MapManagerComponent::m_mapFrameId)
                    ->Field("OdomFrameId", &MapManagerComponent::m_odomFrameId)
                    ->Field("GeodeticConfiguration", &MapManagerComponent::m_geodeticConfiguration)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapManagerComponent>("Map manager", "Map manager component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_mapFrameId, "Map frame name", "Frame name used for the map")
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateNamespaceField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_odomFrameId, "Odom frame name", "Frame name used for a robot odometry")
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateNamespaceField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerComponent::m_geodeticConfiguration, "Geodetic", "Geodetic configuration")
                        ;
            }
        }
    }

    AZ::Transform MapManagerComponent::ConvertToMapCoordinateSystem(AZ::Transform transform)
    {
        m_geodeticConfiguration.SetHook();
        return m_geodeticConfiguration.m_mapHookTransform.GetInverse() * transform;
    }

    AZ::Transform MapManagerComponent::ConvertFromMapCoordinateSystem(AZ::Transform transform)
    {
        m_geodeticConfiguration.SetHook();
        return m_geodeticConfiguration.m_mapHookTransform * transform;
    }

    AZ::Vector3 MapManagerComponent::WorldPositionToLatLon(const AZ::Vector3 &worldPosition)
    {
        AZ::Transform worldPositionInMapHookFrame =
                ConvertToMapCoordinateSystem(AZ::Transform(worldPosition, AZ::Quaternion::CreateIdentity(), 1.0));

        const AZ::Vector3 currentPositionECEF =
            Map::Utilities::ENUToECEF(
                    {m_geodeticConfiguration.m_originLatitudeDeg,
                     m_geodeticConfiguration.m_originLongitudeDeg,
                     m_geodeticConfiguration.m_originAltitude},
                    worldPositionInMapHookFrame.GetTranslation());
        return Map::Utilities::ECEFToWGS84(currentPositionECEF);
    }

    AZ::Vector3 MapManagerComponent::LatLonToWorldPosition(const AZ::Vector3 &latlon)
    {
        const auto currentECEF = Map::Utilities::WGS84ToECEF(latlon);

        auto localPose =  Map::Utilities::ECEFToENU({m_geodeticConfiguration.m_originLatitudeDeg,
                                          m_geodeticConfiguration.m_originLongitudeDeg,
                                          m_geodeticConfiguration.m_originAltitude},
                                         currentECEF);

        AZ_Printf("ROS2GNSSSensorComponent", "PRE CONV TO MAP %lf, %lf, %lf", localPose.GetX(), localPose.GetY(), localPose.GetZ());

        return ConvertFromMapCoordinateSystem(AZ::Transform(localPose, AZ::Quaternion::CreateIdentity(), 1.0)).GetTranslation();
    }

}