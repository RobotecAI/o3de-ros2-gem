/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include "Frame/ROS2FrameComponent.h"
#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/InertialsMaker.h"
#include "RobotImporter/URDF/JointsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include "RobotImporter/URDF/VisualsMaker.h"

#include <AzCore/Utils/Utils.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <regex> // TODO - we are currently replacing package:// with an absolute path

namespace ROS2
{
    URDFPrefabMaker::URDFPrefabMaker(const AZStd::string& modelFilePath)
        : m_visualsMaker(modelFilePath)
    {
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF(urdf::ModelInterfaceSharedPtr model)
    { // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
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
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        auto outcome = prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ contentEntityId }, newPrefabPath);
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            PrefabMakerUtils::AddRequiredComponentsToEntity(prefabContainerEntityId);
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

        auto createEntityResult = PrefabMakerUtils::CreateEntity(parentEntityId, link->name.c_str());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }
        AZ::EntityId entityId = createEntityResult.GetValue();
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        // Add ROS2FrameComponent - TODO: only for top level and joints
        entity->CreateComponent<ROS2FrameComponent>(link->name.c_str());

        m_visualsMaker.AddVisuals(link, entityId);
        m_collidersMaker.AddColliders(link, entityId);
        m_inertialsMaker.AddInertial(link->inertial, entityId);

        for (auto childLink : link->child_links)
        {
            auto outcome = AddEntitiesForLink(childLink, entityId); // recursive call
            if (!outcome.IsSuccess())
            { // TODO - decide on behavior. Still proceed to load other children?
                AZ_Warning("AddEntitiesForLink", false, "Unable to add entity due to an error: %s", outcome.GetError().c_str());
                continue;
            }

            AZ::EntityId childEntityId = outcome.GetValue();
            m_jointsMaker.AddJointInformationToEntity(link, childLink, childEntityId);
        }

        return AZ::Success(entityId);
    }
} // namespace ROS2
