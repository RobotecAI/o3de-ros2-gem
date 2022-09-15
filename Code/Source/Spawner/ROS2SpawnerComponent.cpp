/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

namespace ROS2
{
    void ROS2SpawnerComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_get_names_service = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
            "get_available_spawnable_names",
            std::bind(&ROS2SpawnerComponent::GetAvailableSpawnableNames, this, AZStd::placeholders::_1, AZStd::placeholders::_2));

        m_spawn_service = ros2Node->create_service<gazebo_msgs::srv::SpawnEntity>(
            "spawn_entity", std::bind(&ROS2SpawnerComponent::SpawnRobot, this, AZStd::placeholders::_1, AZStd::placeholders::_2));
    }

    void ROS2SpawnerComponent::Deactivate()
    {
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, AZ::Component>()->Version(1)->Field("Spawner", &ROS2SpawnerComponent::m_spawnables);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SpawnerComponent>("ROS2 Spawner", "Spawner component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Allows spawning")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ROS2SpawnerComponent::m_spawnables, "Spawnable", "Spawnable");
            }
        }
    }

    void ROS2SpawnerComponent::GetAvailableSpawnableNames(
        const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response)
    {
        for (const auto& spawnable : m_spawnables)
        {
            response->model_names.emplace_back(spawnable.first.c_str());
        }
    }

    void ROS2SpawnerComponent::SpawnRobot(const SpawnEntityRequest request, SpawnEntityResponse response)
    {
        const AZStd::string key(request->name.c_str(), request->name.size());

        auto spawnable = m_spawnables.find(key);

        if (spawnable != m_spawnables.end())
        {
            if (m_tickets.find(key) == m_tickets.end())
            {
                // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to
                // spawn an entity
                m_tickets.emplace(spawnable->first, AzFramework::EntitySpawnTicket(spawnable->second));
            }

            auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

            AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

            optionalArgs.m_preInsertionCallback = [this, position = request->initial_pose](auto id, auto view)
            {
                this->pre_spawn(
                    id,
                    view,
                    AZ::Transform(
                        AZ::Vector3(position.position.x, position.position.y, position.position.z),
                        AZ::Quaternion(position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w),
                        1.0f));
            };

            spawner->SpawnAllEntities(m_tickets.at(key), optionalArgs);

            response->success = true;
            return;
        }

        response->success = false;
    }

    void ROS2SpawnerComponent::pre_spawn(
        AzFramework::EntitySpawnTicket::Id id, AzFramework::SpawnableEntityContainerView view, const AZ::Transform& transform)
    {
        AZ::Entity* root = *view.begin();

        auto* transformInterface_ = root->FindComponent<AzFramework::TransformComponent>();

        transformInterface_->SetWorldTM(transform);
    }

} // namespace ROS2
