/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Component.h>
#include <Spawner/SpawnPointsBus.h>
#include <Spawner/SpawnPointComponent.h>
#include <ROS2SystemComponent.h>

#include <o3de_spawning_interface_srvs/srv/get_spawn_points.hpp>
#include <o3de_spawning_interface_srvs/srv/is_spawn_point_suitable.hpp>
#include <o3de_spawning_interface_srvs/msg/spawn_point.hpp>

namespace ROS2
{
    class ROS2SpawnPointsProviderComponent:
            public AZ::Component,
            public SpawnPointsRequestBus::Handler
        {
        public:
            AZ_COMPONENT(ROS2SpawnPointsProviderComponent, "{199854D3-AEAD-4837-9A02-C07533676CE9}", AZ::Component, SpawnPointsRequestBus::Handler);

            // AZ::Component interface implementation.
            ROS2SpawnPointsProviderComponent();
            ~ROS2SpawnPointsProviderComponent();

            void Activate() override;
            void Deactivate() override;

            // Required Reflect function.
            static void Reflect(AZ::ReflectContext* context);

            std::vector<SpawnPointInfo> GetSpawnPoints(const std::vector<AZ::Vector3> &bounding_box) const override;

            bool IsSpawnPointSuitable(const AZ::Vector3 &point, const std::vector<AZ::Vector3> &bounding_box) const override;

        private:
            rclcpp::Service<o3de_spawning_interface_srvs::srv::GetSpawnPoints>::SharedPtr m_get_spawn_points_service;
            rclcpp::Service<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable>::SharedPtr m_is_spawn_point_suitable_service;

            void GetSpawnPointsSrv(const std::shared_ptr<o3de_spawning_interface_srvs::srv::GetSpawnPoints::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::GetSpawnPoints::Response> response);
            void IsSpawnPointSuitableSrv(const std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::IsSpawnPointSuitable::Response> response);
        };
}
