/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/string/conversions.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    AZStd::string PrefabMakerUtils::GetAzModelAssetPathFromModelPath(std::filesystem::path modelPath)
    {
        bool assetFound = false;
        AZ::Data::AssetInfo assetInfo;
        AZStd::string watchDir;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            assetFound,
            &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
            modelPath.c_str(),
            assetInfo,
            watchDir);

        if (!assetFound)
        {
            AZ_Error("AddVisuals", false, "Could not find model asset for %s", modelPath.c_str());
            return "";
        }

        auto relativePath = std::filesystem::path(assetInfo.m_relativePath.c_str());
        relativePath.replace_extension("azmodel");
        auto assetPath = AZStd::string(relativePath.string().c_str());
        AZStd::to_lower(assetPath.begin(), assetPath.end());

        return assetPath;
    }

    void PrefabMakerUtils::SetEntityTransform(const urdf::Pose& origin, AZ::EntityId entityId)
    {
        urdf::Vector3 urdfPosition = origin.position;
        urdf::Rotation urdfRotation = origin.rotation;
        AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(urdfRotation);
        AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(urdfPosition);
        AZ::Transform tf(azPosition, azRotation, 1.0f);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();

        if (!transformInterface)
        {
            AZ_Error("SetEntityTransform", false, "Missing Transform component!");
            return;
        }
        // Alternative - but requires an active component. We would activate/deactivate a lot. Or - do it in a second pass.
        // AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, transformForChild);

        transformInterface->SetLocalTM(tf);
    }

    AzToolsFramework::Prefab::PrefabEntityResult PrefabMakerUtils::CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name)
    {
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        auto createEntityResult = prefabInterface->CreateEntity(parentEntityId, AZ::Vector3());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }

        AZ::EntityId entityId = createEntityResult.GetValue();
        if (!entityId.IsValid())
        { // Verify that a valid entity is created.
            return AZ::Failure(AZStd::string("Invalid id for created entity"));
        }

        AZ_TracePrintf("CreateEntity", "Processing entity id:%s with name:%s", entityId.ToString().c_str(), name.c_str());
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZStd::string entityName(name.c_str());
        entity->SetName(entityName);
        entity->Deactivate();
        AddRequiredComponentsToEntity(entityId);
        return createEntityResult;
    }

    AzToolsFramework::Prefab::PrefabOperationResult PrefabMakerUtils::RemoveEntityWithDescendants(AZ::EntityId parentEntityId)
    {
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        return prefabInterface->DeleteEntitiesAndAllDescendantsInInstance({ parentEntityId });
    }

    void PrefabMakerUtils::AddRequiredComponentsToEntity(AZ::EntityId entityId)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AzToolsFramework::EditorEntityContextRequestBus::Broadcast(
            &AzToolsFramework::EditorEntityContextRequests::AddRequiredComponents, *entity);
    }

    bool PrefabMakerUtils::HasCollider(AZ::EntityId entityId)
    { // TODO - actually, we want EditorColliderComponent specifically, but the change can be applied only after moving to ECC
        // TODO - which will happen when Cylinder shape is supported. Until then, we check for either ECC or ESCC.
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        return entity->FindComponent<PhysX::EditorColliderComponent>() != nullptr ||
            entity->FindComponent<PhysX::EditorShapeColliderComponent>() != nullptr;
    }

    AzToolsFramework::EntityIdList PrefabMakerUtils::GetColliderChildren(AZ::EntityId parentEntityId)
    {
        AzToolsFramework::EntityIdList colliderChildren;
        AzToolsFramework::EntityIdList allChildren = AzToolsFramework::GetEntityChildOrder(parentEntityId);
        for (auto childId : allChildren)
        {
            AZ_TracePrintf("GetColliderChildren", "Considering child %s", childId.ToString().c_str());
            if (HasCollider(childId))
            {
                AZ_TracePrintf("GetColliderChildren", "Child %s has a collider", childId.ToString().c_str());
                colliderChildren.push_back(childId);
            }
        }
        return colliderChildren;
    }

    AZStd::string PrefabMakerUtils::MakeEntityName(const AZStd::string& rootName, const AZStd::string& type, size_t index)
    {
        const AZStd::string suffix = index == 0 ? AZStd::string("") : AZStd::string::format("_%zu", index);
        return AZStd::string::format("%s_%s%s", rootName.c_str(), type.c_str(), suffix.c_str());
    }
} // namespace ROS2
