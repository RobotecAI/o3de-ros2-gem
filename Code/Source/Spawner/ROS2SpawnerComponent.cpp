/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

namespace ROS2
{
    void ROS2SpawnerComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        get_names_service = ros2Node->create_service<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames>(
                "get_available_spawnable_names",
                std::bind( &ROS2SpawnerComponent::GetAvailableSpawnableNames, this, AZStd::placeholders::_1, AZStd::placeholders::_2 )
                );

        spawn_service = ros2Node->create_service<o3de_spawning_interface_srvs::srv::SpawnRobot>(
                "spawn_robot",
                std::bind( &ROS2SpawnerComponent::SpawnRobot, this, AZStd::placeholders::_1, AZStd::placeholders::_2 )
                );
    }

    void ROS2SpawnerComponent::Deactivate()
    {
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, AZ::Component>()->Version(1)
                ->Field("Spawner", &ROS2SpawnerComponent::m_spawnables);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SpawnerComponent>("ROS2 Spawner", "Spawner component")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "Allows spawning")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &ROS2SpawnerComponent::m_spawnables,
                                "Spawnable",
                                "Spawnable");
            }
        }
    }

    void ROS2SpawnerComponent::GetAvailableSpawnableNames(const std::shared_ptr<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::GetAvailableSpawnableNames::Response> response)
    {
        for( const auto& spawnable : m_spawnables )
        {
            response->names.emplace_back( spawnable.GetHint().c_str() );
        }
    }

    void ROS2SpawnerComponent::SpawnRobot(const std::shared_ptr<o3de_spawning_interface_srvs::srv::SpawnRobot::Request> request, std::shared_ptr<o3de_spawning_interface_srvs::srv::SpawnRobot::Response> response)
    {
        auto key = AZStd::string(request->robot_name.c_str(), request->robot_name.size() );

        auto spawnable = std::find_if( m_spawnables.begin(), m_spawnables.end(), [key](auto spwn) { return spwn.GetHint() == key; } );

        if(  spawnable != m_spawnables.end() )
        {
            if( m_tickets.find( key ) == m_tickets.end() )
            {
                // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to spawn an entity
                m_tickets.insert( { spawnable->GetHint(), AzFramework::EntitySpawnTicket(*spawnable) });
            }

            auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
            spawner->SpawnAllEntities( m_tickets.at(key) );
            response->result = true;
        }

        response->result = false;
    }
} // namespace ROS2