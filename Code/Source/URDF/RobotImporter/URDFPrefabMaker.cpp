/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "URDF/RobotImporter/URDFPrefabMaker.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Utils/Utils.h>
#include <AzToolsFramework/Component/EditorComponentAPIBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponent.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <AzToolsFramework/Prefab/PrefabPublicRequestBus.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h> // TODO - temporary test item
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    namespace Internal
    {
        void AddRequiredComponentsToEntity(AZ::EntityId entityId)
        {
            AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
            entity->Deactivate();
            AzToolsFramework::EditorEntityContextRequestBus::Broadcast(
                    &AzToolsFramework::EditorEntityContextRequests::AddRequiredComponents, *entity);
            entity->Activate();
        }
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF(urdf::ModelInterfaceSharedPtr model)
    {   // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
        // TODO - add a check if the prefab with a given name already exists. Choice to cancel, overwrite or suffix name
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        auto createEntityResult = prefabInterface->CreateEntity(AZ::EntityId(), AZ::Vector3());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }

        // Verify that a valid entity is created.
        AZ::EntityId newEntityId = createEntityResult.GetValue();
        if (!newEntityId.IsValid())
        {
            return AZ::Failure(AZStd::string("Failed to create top level prefab entity"));
        }
        AZ::Entity* entityForPrefab = AzToolsFramework::GetEntityById(newEntityId);
        AZStd::string prefabContentEntityName(model->getName().c_str());
        entityForPrefab->SetName(prefabContentEntityName);

        Internal::AddRequiredComponentsToEntity(newEntityId);

        // TODO: add temporary test components
        entityForPrefab->Deactivate();
        entityForPrefab->CreateComponent<PhysX::EditorShapeColliderComponent>();
        entityForPrefab->CreateComponent(LmbrCentral::EditorBoxShapeComponentTypeId);
        entityForPrefab->Activate();

        auto prefabName = AZStd::string::format("%s.%s", model->getName().c_str(), "prefab");
        auto newPrefabPath = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName.c_str();

        // Create prefab, save it to disk immediately
        auto outcome = prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ newEntityId }, newPrefabPath);
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            Internal::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        return outcome;
    }
} // namespace ROS2
