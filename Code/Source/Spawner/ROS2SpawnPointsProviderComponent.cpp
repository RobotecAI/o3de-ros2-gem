/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnPointsProviderComponent.h"
#include <AzCore/Serialization/EditContext.h>

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

        auto points = GetSpawnPoints( bounding_box );

        response->available_points.insert( response->available_points.end(), points.size(), geometry_msgs::msg::Point() );

        std::transform( points.begin(), points.end(), response->available_points.begin(), []( const auto& point )
            {
                auto p = geometry_msgs::msg::Point();
                p.x = point.GetX();
                p.y = point.GetY();
                p.z = point.GetZ();
                return p;
            }
        );
    }

    void ROS2SpawnPointsProviderComponent::IsSpawnPointSuitableSrv(const std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Response> response)
    {
        std::vector<AZ::Vector3> bounding_box( request->bounding_box.size(), {0, 0, 0});

        std::transform(request->bounding_box.begin(), request->bounding_box.end(), bounding_box.begin(), [](auto point){ return AZ::Vector3(point.x, point.y, point.z); });

        response->result = IsSpawnPointSuitable( AZ::Vector3( request->center.x, request->center.y, request->center.z ), bounding_box );
    }

    bool ROS2SpawnPointsProviderComponent::IsSpawnPointSuitable(const AZ::Vector3 &point, const std::vector<AZ::Vector3> &bounding_box) const
    {
        //todo: implementation
        return true;
    }

    std::vector<AZ::Vector3> ROS2SpawnPointsProviderComponent::GetSpawnPoints(const std::vector<AZ::Vector3> &bounding_box) const
    {
        //todo: implementation
        return std::vector<AZ::Vector3>{ {2,2,0.5} };
    }
}