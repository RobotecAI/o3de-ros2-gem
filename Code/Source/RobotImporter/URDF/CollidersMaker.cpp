/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/SceneManifest.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorShapeColliderComponent.h>
#include <AzCore/std/string/regex.h>

namespace ROS2
{
    bool CollidersMaker::GuessIsWheel(const AZStd::string &entityName){

        AZStd::regex wheel_regex("(?i)wheel|(?i)wh_|(?i)(_wh)");
        AZStd::smatch match;
        if (AZStd::regex_search(entityName, match, wheel_regex))
        {
            AZ_Printf("WheelMaterial", "%s is a wheel\n", entityName.c_str());
            return true;
        }
        AZ_Printf("WheelMaterial", "%s is NOT wheel\n", entityName.c_str());
        return false;
    }

    CollidersMaker::CollidersMaker(AZStd::string modelPath)
        : m_modelPath(AZStd::move(modelPath))
        , m_stopBuildFlag(false)
    {

        // get Wheel material from assets
        bool assetFound = false;
        AZ::Data::AssetInfo assetInfo;
        AZStd::string watchDir;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            assetFound,
            &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
            "/home/michal/github/o3de-ros2-gem/Assets/Prefabs/Materials/wheel_material.physxmaterial",
            assetInfo,
            watchDir);

        if (assetFound){
            AZ_Printf("Wheel Material", "path: %s type:  %s id: %s found %d\n", assetInfo.m_relativePath.c_str(), assetInfo.m_assetType.ToString<AZStd::string>().c_str(), assetInfo.m_assetId.ToString<AZStd::string>().c_str(),assetFound);
            m_wheelMaterial = AZ::Data::Asset<Physics::MaterialAsset>(assetInfo.m_assetId, Physics::MaterialAsset::TYPEINFO_Uuid(), assetInfo.m_relativePath);
            AZ_Printf("Wheel Material", "WaitForLoad m_wheelMaterial : %s\n", m_wheelMaterial.ToString<AZStd::string>().c_str());
            m_wheelMaterial.BlockUntilLoadComplete();
            AZ_Printf("Wheel Material", " Loaded  m_wheelMaterial : %s\n", m_wheelMaterial.ToString<AZStd::string>().c_str());
        }
        else{
            AZ_Warning("Wheel Material", false, "Cannot load wheel material");
        }
    }

    CollidersMaker::~CollidersMaker()
    {
        m_stopBuildFlag = true;
        m_buildThread.join();
    };

    void CollidersMaker::BuildColliders(urdf::LinkSharedPtr link)
    {
        for (auto collider : link->collision_array)
        {
            BuildCollider(collider);
        }

        if (link->collision_array.empty())
        {
            BuildCollider(link->collision);
        }
    }

    void CollidersMaker::BuildCollider(urdf::CollisionSharedPtr collision)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }

        auto geometry = collision->geometry;
        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;
        if (!isPrimitiveShape)
        {
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            auto azMeshPath = GetFullURDFMeshPath(AZ::IO::Path(m_modelPath), AZ::IO::Path(meshGeometry->filename.c_str()));

            AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
            AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
                scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, azMeshPath.c_str(), AZ::Uuid::CreateNull());
            if (!scene)
            {
                AZ_Error(
                    "CollisionMaker",
                    false,
                    "Error loading collider. Invalid scene: %s, URDF path: %s",
                    azMeshPath.c_str(),
                    meshGeometry->filename.c_str());
                return;
            }

            AZ::SceneAPI::Containers::SceneManifest& manifest = scene->GetManifest();
            auto valueStorage = manifest.GetValueStorage();
            if (valueStorage.empty())
            {
                AZ_Error("CollisionMaker", false, "Error loading collider. Invalid value storage: %s", azMeshPath.c_str());
                return;
            }

            auto view = AZ::SceneAPI::Containers::MakeDerivedFilterView<AZ::SceneAPI::DataTypes::ISceneNodeGroup>(valueStorage);
            if (view.empty())
            {
                AZ_Error("CollisionMaker", false, "Error loading collider. Invalid node views: %s", azMeshPath.c_str());
                return;
            }

            // Select all nodes for both visual and collision nodes
            for (AZ::SceneAPI::DataTypes::ISceneNodeGroup& mg : view)
            {
                AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), mg.GetSceneNodeSelectionList());
            }

            // Update scene with all nodes selected
            AZ::SceneAPI::Events::ProcessingResultCombiner result;
            AZ::SceneAPI::Events::AssetImportRequestBus::BroadcastResult(
                result,
                &AZ::SceneAPI::Events::AssetImportRequest::UpdateManifest,
                *scene,
                AZ::SceneAPI::Events::AssetImportRequest::ManifestAction::Update,
                AZ::SceneAPI::Events::AssetImportRequest::RequestingApplication::Editor);

            if (result.GetResult() != AZ::SceneAPI::Events::ProcessingResult::Success)
            {
                AZ_TracePrintf("CollisionMaker", "Scene updated\n");
                return;
            }

            auto assetInfoFilePath = azMeshPath;
            assetInfoFilePath.Native() += ".assetinfo";
            AZ_Printf("CollisionMaker", "Saving collider manifest to %s", assetInfoFilePath.c_str());
            scene->GetManifest().SaveToFile(assetInfoFilePath.c_str());

            bool assetFound = false;
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchDir;
            AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
                assetFound,
                &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
                azMeshPath.c_str(),
                assetInfo,
                watchDir);

            // Set export method to convex mesh
            auto readOutcome = AZ::JsonSerializationUtils::ReadJsonFile(assetInfoFilePath.c_str());
            if (!readOutcome.IsSuccess())
            {
                AZ_Error("CollisionMaker", false, "Could not read %s with %s", assetInfoFilePath.c_str(), readOutcome.GetError().c_str());
                return;
            }
            rapidjson::Document assetInfoJson = readOutcome.TakeValue();
            auto manifestObject = assetInfoJson.GetObject();
            auto valuesIterator = manifestObject.FindMember("values");
            if (valuesIterator == manifestObject.MemberEnd())
            {
                AZ_Error("CollisionMaker", false, "Invalid json file: %s (Missing 'values' node)", assetInfoFilePath.c_str());
                return;
            }

            constexpr AZStd::string_view physXMeshGroupType = "{5B03C8E6-8CEE-4DA0-A7FA-CD88689DD45B} MeshGroup";
            auto valuesArray = valuesIterator->value.GetArray();
            for (auto& value : valuesArray)
            {
                auto object = value.GetObject();

                auto physXMeshGroupIterator = object.FindMember("$type");
                if (AZ::StringFunc::Equal(physXMeshGroupIterator->value.GetString(), physXMeshGroupType))
                {
                    value.AddMember(rapidjson::StringRef("export method"), rapidjson::StringRef("1"), assetInfoJson.GetAllocator());
                }
            }

            auto saveOutcome = AZ::JsonSerializationUtils::WriteJsonFile(assetInfoJson, assetInfoFilePath.c_str());
            if (!saveOutcome.IsSuccess())
            {
                AZ_Error("CollisionMaker", false, "Could not save %s with %s", assetInfoFilePath.c_str(), saveOutcome.GetError().c_str());
                return;
            }

            // Add asset to expected assets list
            if (assetFound)
            {
                auto pxmeshAssetPath = AZ::IO::Path(assetInfo.m_relativePath).ReplaceExtension("pxmesh");

                AZStd::lock_guard lock{ m_buildMutex };
                m_meshesToBuild.push_back(pxmeshAssetPath);
            }
        }
    }

    void CollidersMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        AZStd::string typeString = "collider";
        size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed colliders are present. The order does not matter here
        for (auto collider : link->collision_array)
        { // one or more colliders - the array is used
            AddCollider(collider, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString, nameSuffixIndex));
            nameSuffixIndex++;
        }

        if (nameSuffixIndex == 0)
        { // no colliders in the array - zero or one in total, the element member is used instead
            AddCollider(link->collision, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString));
        }
    }

    void CollidersMaker::AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZStd::string& generatedName)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }
        AZ_TracePrintf("AddCollider", "Processing collisions for entity id:%s\n", entityId.ToString().c_str());

        auto geometry = collision->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning("AddCollider", false, "No Geometry for a collider");
            return;
        }

        AddColliderToEntity(collision, entityId);
    }

    void CollidersMaker::AddColliderToEntity(urdf::CollisionSharedPtr collision, AZ::EntityId entityId)
    {

        // TODO - we are unable to set collider origin. Sub-entities don't work since they would need to parent visuals etc.
        // TODO - solution: once Collider Component supports Cylinder Shape, switch to it from Shape Collider Component.

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "entity not exists");
        auto geometry = collision->geometry;
        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;

        Physics::ColliderConfiguration colliderConfiguration;
        const bool is_wheel = GuessIsWheel(entity->GetName());
        if (is_wheel){
            colliderConfiguration.m_materialSlots.SetMaterialAsset(0, m_wheelMaterial);
        }

        if (!isPrimitiveShape)
        {

            // TODO move setting mesh with ebus here - othervise material is not assigned
            Physics::ConvexHullShapeConfiguration shapeConfiguration;
            entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfiguration, shapeConfiguration);
            entity->Activate();

            AZ_Printf("CollisionMaker", "Adding mesh collider to %s\n", entityId.ToString().c_str());
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            AZ_Assert(meshGeometry, "geometry is not meshGeometry");
            auto azMeshPath = GetFullURDFMeshPath(AZ::IO::Path(m_modelPath), AZ::IO::Path(meshGeometry->filename.c_str()));

            // Get asset path relative to watch folder
            bool assetFound = false;
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchDir;
            AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
                assetFound,
                &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
                azMeshPath.c_str(),
                assetInfo,
                watchDir);

            // We are expecting .pxmesh
            auto pxmodelPath = AZStd::string(assetInfo.m_relativePath.c_str());
            AzFramework::StringFunc::Path::ReplaceExtension(pxmodelPath, ".pxmesh");

            // Get asset product id (pxmesh)
            AZ::Data::AssetId assetId;
            AZ::Data::AssetType assetType = AZ::AzTypeInfo<PhysX::Pipeline::MeshAsset>::Uuid();
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                assetId, &AZ::Data::AssetCatalogRequests::GetAssetIdByPath, pxmodelPath.c_str(), assetType, false);
            // Insert pxmesh into the collider component
            PhysX::MeshColliderComponentRequestsBus::Event(entityId, &PhysX::MeshColliderComponentRequests::SetMeshAsset, assetId);
            entity->Deactivate();
            if (is_wheel){
                AZ_Warning("CollisionMaker", false, "MeshColliderComponentRequests to %s overridden (probably) the material", entityId.ToString().c_str() )
            }
            return;
        }

        AZ_Printf("Collision", "URDF geometry type : %d\n", geometry->type);
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                AZ_Assert(sphereGeometry, "geometry is not sphereGeometry");
                const Physics::SphereShapeConfiguration cfg{static_cast<float>(sphereGeometry->radius)};
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfiguration, cfg);
            }
            break;
        case urdf::Geometry::BOX:
            {
                const auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                AZ_Assert(boxGeometry, "geometry is not boxGeometry");
                const Physics::BoxShapeConfiguration cfg{URDF::TypeConversions::ConvertVector3(boxGeometry->dim)};
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfiguration, cfg);
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                AZ_Assert(cylinderGeometry, "geometry is not cylinderGeometry");
                // TODO HACK Underlying API of O3DE  does not have Physic::CylinderShapeConfiguration implementation
                Physics::BoxShapeConfiguration cfg;
                auto* component = entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfiguration, cfg);
                entity->Activate();
                PhysX::EditorColliderComponentRequestBus::Event(AZ::EntityComponentIdPair(entityId, component->GetId()),
                                                               &PhysX::EditorColliderComponentRequests::SetShapeType, Physics::ShapeType::Cylinder);
                PhysX::EditorColliderComponentRequestBus::Event(AZ::EntityComponentIdPair(entityId, component->GetId()),
                                                                &PhysX::EditorColliderComponentRequests::SetCylinderHeight, cylinderGeometry->length);
                PhysX::EditorColliderComponentRequestBus::Event(AZ::EntityComponentIdPair(entityId, component->GetId()),
                                                                &PhysX::EditorColliderComponentRequests::SetCylinderRadius, cylinderGeometry->radius);
                entity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddCollider", false, "Unsupported collider geometry type, %d", geometry->type);
            break;
        }
    }
    AZ::IO::Path CollidersMaker::GetFullURDFMeshPath(AZ::IO::Path modelPath, AZ::IO::Path meshPath)
    {
        modelPath.RemoveFilename();
        AZ::StringFunc::Replace(meshPath.Native(), "package://", "", true, true);
        modelPath /= meshPath;

        return modelPath;
    }

    void CollidersMaker::ProcessMeshes(BuildReadyCallback notifyBuildReadyCb)
    {
        m_buildThread = AZStd::thread(
            [this, notifyBuildReadyCb]()
            {
                AZ_Printf("CollisionMaker", "Waiting for URDF assets\n");

                while (!m_meshesToBuild.empty() && !m_stopBuildFlag)
                {
                    {
                        AZStd::lock_guard lock{ m_buildMutex };
                        auto eraseFoundMesh = [](const AZ::IO::Path& meshPath)
                        {
                            AZ::Data::AssetId assetId;
                            AZ::Data::AssetType assetType = AZ::AzTypeInfo<PhysX::Pipeline::MeshAsset>::Uuid();
                            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                                assetId, &AZ::Data::AssetCatalogRequests::GetAssetIdByPath, meshPath.c_str(), assetType, false);
                            if (assetId.IsValid())
                            {
                                AZ_Printf("CollisionMaker", "Asset %s found and valid...\n", meshPath.c_str());
                                return true; // return true to erase the mesh path
                            }

                            return false;
                        };
                        AZStd::erase_if(m_meshesToBuild, AZStd::move(eraseFoundMesh));
                    }
                    if (!m_meshesToBuild.empty())
                    {
                        AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(50));
                    }
                }

                AZ_Printf("CollisionMaker", "All URDF assets ready!\n");
                // Notify the caller that we can continue with constructing the prefab.
                notifyBuildReadyCb();
            });
    }
} // namespace ROS2