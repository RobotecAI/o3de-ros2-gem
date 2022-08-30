/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <Sensor/ROS2SensorComponent.h>
#include <ROS2SystemComponent.h>

#include <o3de_spawning_interface_srvs/srv/get_available_spawnable_names.hpp>
#include <o3de_spawning_interface_srvs/srv/spawn_robot.hpp>

namespace ROS2
{
    //! TODO: doc
    class ROS2SpawnerComponent : public AZ::Component
        {
        public:

            AZ_COMPONENT(ROS2SpawnerComponent, "{5950AC6B-75F3-4E0F-BA5C-17C877013710}", AZ::Component);

            // AZ::Component interface implementation.
            ROS2SpawnerComponent() = default;
            ~ROS2SpawnerComponent() = default;

            void Activate() override;
            void Deactivate() override;

            // Required Reflect function.
            static void Reflect(AZ::ReflectContext* context);

        private:
            std::map<AZStd::string, AzFramework::EntitySpawnTicket> m_tickets = {};
            AZStd::vector<AZ::Data::Asset<AzFramework::Spawnable>> m_spawnables = {};

            rclcpp::Service<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames>::SharedPtr get_names_service;
            rclcpp::Service<o3de_spawning_interface_srvs::srv::SpawnRobot>::SharedPtr spawn_service;

            void GetAvailableSpawnableNames(const std::shared_ptr<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames::Response> response);
            void SpawnRobot(const std::shared_ptr<o3de_spawning_interface_srvs::srv::SpawnRobot::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::SpawnRobot::Response> response);
        };
} // namespace ROS2
