/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "URDF/RobotImporter/URDFPrefabMaker.h"
#include "Frame/ROS2FrameComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Utils/Utils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/Component/EditorComponentAPIBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponent.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <AzToolsFramework/Prefab/PrefabPublicRequestBus.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    namespace Internal
    {
        void AddRequiredComponentsToEntity(AZ::EntityId entityId)
        {
            AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
            AzToolsFramework::EditorEntityContextRequestBus::Broadcast(
                    &AzToolsFramework::EditorEntityContextRequests::AddRequiredComponents, *entity);
        }
    }

    URDFPrefabMaker::URDFPrefabMaker()
    {
        m_prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF(urdf::ModelInterfaceSharedPtr model)
    {   // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
        // TODO - add a check if the prefab with a given name already exists. Choice to cancel, overwrite or suffix name

        // recursively add all entities
        AZ_TracePrintf("CreatePrefabFromURDF", "Creating a prefab for URDF model with name %s", model->getName().c_str());
        auto createEntityResult = AddEntitiesForLink(model->root_link_, AZ::EntityId());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }

        auto prefabName = AZStd::string::format("%s.%s", model->getName().c_str(), "prefab");
        auto newPrefabPath = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName.c_str();
        auto contentEntityId = createEntityResult.GetValue();

        // Create prefab, save it to disk immediately

        auto outcome = m_prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ contentEntityId }, newPrefabPath);
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            Internal::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created %s prefab", prefabName.c_str());
        return outcome;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId)
    {
        if (!link)
        {
            AZ::Failure(AZStd::string("Failed to create prefab entity - link is null"));
        }

        auto createEntityResult = m_prefabInterface->CreateEntity(parentEntityId, AZ::Vector3());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }

        AZ::EntityId entityId = createEntityResult.GetValue();
        if (!entityId.IsValid())
        {   // Verify that a valid entity is created.
            return AZ::Failure(AZStd::string("Invalid id for created entity"));
        }

        AZ_TracePrintf("AddEntitiesForLink", "Processing entity id:%s with link name:%s",
                       entityId.ToString().c_str(), link->name.c_str());
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZStd::string entityName(link->name.c_str());
        entity->SetName(entityName);
        entity->Deactivate();
        Internal::AddRequiredComponentsToEntity(entityId);
        entity->CreateComponent<ROS2FrameComponent>(entityName);
        for (auto childLink : link->child_links)
        {
            auto outcome = AddEntitiesForLink(childLink, entityId); // recursive call
            if (!outcome.IsSuccess())
            {   // TODO - decide on behavior. Still proceed to load other children?
                AZ_Warning("AddEntitiesForLink", false, "Unable to add entity due to an error: %s", outcome.GetError().c_str());
                continue;
            }

            AZ::EntityId childEntityId = outcome.GetValue();
            AddJointInformationToEntity(link, childLink, childEntityId);
        }

        return AZ::Success(entityId);
    }

    void URDFPrefabMaker::AddJointInformationToEntity(urdf::LinkSharedPtr parentLink,
                                                      urdf::LinkSharedPtr childLink, AZ::EntityId entityId)
    {   // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        {   // TODO - replace with std algoritm
            if (joint->child_link_name == childLink->name)
            {   // Found a match!
                // TODO - handle joint types etc. (a dedicated component) - check existing JointComponent

                // Get URDF pose elements
                urdf::Vector3 urdfPosition = joint->parent_to_joint_origin_transform.position;
                urdf::Rotation urdfRotation = joint->parent_to_joint_origin_transform.rotation;

                // TODO - create translation functions
                AZ::Quaternion azRotation(urdfRotation.x, urdfRotation.y, urdfRotation.z, urdfRotation.w);
                AZ::Vector3 azPosition(urdfPosition.x, urdfPosition.y, urdfPosition.z);
                AZ::Transform transformForChild(azPosition, azRotation, 1.0f);

                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                if (!transformInterface)
                {
                    AZ_Error("AddJointInformationToEntity", false, "Missing Transform component!");
                    continue;
                }
                // Alternative - but requires an active component. We would activate/deactivate a lot. Or - do it in a second pass.
                // AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, transformForChild);

                transformInterface->SetLocalTM(transformForChild);
                break;
            }
        }
    }

    void URDFPrefabMaker::AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        urdf::VisualSharedPtr visual = link->visual;
        if (!visual)
        {   // it is ok not to have a visual in a link
            return;
        }

        // TODO handle origin

        auto geometry = visual->geometry;
        if (!geometry)
        {   // non-empty visual should have a geometry
            AZ_Warning("AddVisuals", false, "No Geometry for a visual");
            return;
        }

        // TODO handle primitives (Sphere, box, cylinder)
        if (geometry->type == urdf::Geometry::MESH)
        {
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            AZ_TracePrintf("AddVisuals", "Adding mesh filename: %s", meshGeometry->filename.c_str());
            // TODO load mesh, use asset processor

            // TODO apply scale

        }
        // TODO handle material
    }
} // namespace ROS2
