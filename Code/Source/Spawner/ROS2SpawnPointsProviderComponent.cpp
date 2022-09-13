/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnPointsProviderComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Spawner/SpawnPointComponent.h>

#include <geometry_msgs/msg/point.hpp>

namespace ROS2 {
    ROS2SpawnPointsProviderComponent::ROS2SpawnPointsProviderComponent() {
        SpawnPointsInterface::Register( this );
    }

    ROS2SpawnPointsProviderComponent::~ROS2SpawnPointsProviderComponent() {
        SpawnPointsInterface::Unregister( this );
    }

    void ROS2SpawnPointsProviderComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_is_spawn_point_suitable_service = ros2Node->create_service<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable>(
                "is_spawn_point_suitable",
                std::bind( &ROS2SpawnPointsProviderComponent::IsSpawnPointSuitableSrv, this, AZStd::placeholders::_1, AZStd::placeholders::_2 )
        );

        m_get_spawn_points_service = ros2Node->create_service<o3de_spawning_interface_srvs::srv::GetSpawnPoints>(
                "get_spawn_points",
                std::bind( &ROS2SpawnPointsProviderComponent::GetSpawnPointsSrv, this, AZStd::placeholders::_1, AZStd::placeholders::_2 )
        );
    }

    void ROS2SpawnPointsProviderComponent::Deactivate()
    {
    }

    void ROS2SpawnPointsProviderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnPointsProviderComponent, AZ::Component>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SpawnPointsProviderComponent>("ROS2 Spawn Points Provider", "Provides spawn points")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "Provides spawn points")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
            }
        }
    }

    void ROS2SpawnPointsProviderComponent::GetSpawnPointsSrv(const std::shared_ptr<o3de_spawning_interface_srvs::srv::GetSpawnPoints::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::GetSpawnPoints::Response> response)
    {
        std::vector<AZ::Vector3> bounding_box( request->bounding_box.size(), {0, 0, 0});
        std::transform(request->bounding_box.begin(), request->bounding_box.end(), bounding_box.begin(), [](auto point){ return AZ::Vector3(point.x, point.y, point.z); });

        auto points = GetSpawnPoints(bounding_box);

        for( const auto& spawn_point_info : points )
        {
            auto spawn_point = o3de_spawning_interface_srvs::msg::SpawnPoint();

            spawn_point.name = std::string(spawn_point_info.name.c_str(), spawn_point_info.name.size());
            spawn_point.desctiption = std::string(spawn_point_info.description.c_str(), spawn_point_info.description.size());
            spawn_point.position.x = spawn_point_info.position.GetX();
            spawn_point.position.y = spawn_point_info.position.GetY();
            spawn_point.position.z = spawn_point_info.position.GetZ();

            response->available_points.emplace_back( spawn_point );
        }
    }

    void ROS2SpawnPointsProviderComponent::IsSpawnPointSuitableSrv(const std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Response> response)
    {
        std::vector<AZ::Vector3> bounding_box = {};

        for( const auto& bounding_box_point : request->bounding_box )
        {
            bounding_box.emplace_back( AZ::Vector3( bounding_box_point.x, bounding_box_point.y, bounding_box_point.z));
        }

        response->result = IsSpawnPointSuitable( AZ::Vector3( request->center.x, request->center.y, request->center.z ), bounding_box );
    }

    bool ROS2SpawnPointsProviderComponent::IsSpawnPointSuitable(const AZ::Vector3 &point, const std::vector<AZ::Vector3> &bounding_box) const
    {
        //todo: implementation
        return true;
    }

    std::vector<SpawnPointInfo> ROS2SpawnPointsProviderComponent::GetSpawnPoints(const std::vector<AZ::Vector3> &bounding_box) const
    {
        std::vector<SpawnPointInfo> result = {};
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, GetEntityId(), &AZ::TransformBus::Events::GetChildren);

        for( auto child : children )
        {
            auto child_entity = AzToolsFramework::GetEntityById(child );

            auto spawn_point_component = child_entity->FindComponent<SpawnPointComponent>();
            if( spawn_point_component == nullptr )
            {
                continue;
            }

            auto spawn_point_position = child_entity->GetTransform()->GetWorldTM().GetTranslation();

            if(IsSpawnPointSuitable(spawn_point_position, bounding_box) )
            {
                result.emplace_back( SpawnPointInfo{ spawn_point_component->GetName(), spawn_point_component->GetDescription(), spawn_point_position } );
            }
        }

        return result;
    }
}