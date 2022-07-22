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
        using std::placeholders::_1;
        using std::placeholders::_2;
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_spawnPointsService = ros2Node->create_service<o3de_ros2_gem_interfaces::srv::GetSpawnPoints>(
        m_spawnPointsServiceName.data(),
            std::bind(&MapManagerROS2Component::OnSpawnPointsRequest, this, _1, _2));
    }

    void MapManagerROS2Component::Deactivate()
    {
        m_spawnPointsService.reset();
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

    void MapManagerROS2Component::OnSpawnPointsRequest(
            o3de_ros2_gem_interfaces::srv::GetSpawnPoints::Request::SharedPtr request,
            o3de_ros2_gem_interfaces::srv::GetSpawnPoints::Response::SharedPtr response
        )
    {

        AZStd::vector<AZ::Transform> availableSpawnPoints;
        MapRequestBus::BroadcastResult(availableSpawnPoints, &MapRequests::GetAvailableSpawnPoints);

        geometry_msgs::msg::PoseArray availableSpawnPointsMsg;
        for(auto &transform : availableSpawnPoints)
        {
            auto pose = ROS2Conversions::ToROS2Pose(transform);
            availableSpawnPointsMsg.poses.push_back(pose);
        }

        AZStd::string mapFrameId;
        MapRequestBus::BroadcastResult(mapFrameId, &MapRequests::GetMapFrameId);
        availableSpawnPointsMsg.header.frame_id = mapFrameId.data();

        response->spawn_points_array = availableSpawnPointsMsg;
    }

    void MapManagerROS2Component::Reflect(AZ::ReflectContext *context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MapManagerROS2Component, AZ::Component>()
                    ->Version(1)
                    ->Field("SpawnPointsTopic", &MapManagerROS2Component::m_spawnPointsServiceName)
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateTopicField)
                    ->Field("SpawnPointsTopicQoS", &MapManagerROS2Component::m_spawnPointsServiceName)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MapManagerROS2Component>("ROS2 Map manager interface",
                    "ROS2 interface wrapper for map component")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MapManagerROS2Component::m_spawnPointsServiceName,
                                  "Spawn points service name", "Spawn points service name")
                        ;
            }
        }
    }
}