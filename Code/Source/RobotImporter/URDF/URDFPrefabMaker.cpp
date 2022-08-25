/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include "Frame/ROS2FrameComponent.h"
#include "RobotControl/ROS2RobotControlComponent.h"
#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"

#include "RobotImporter/RobotImporterWidget.h"

#include <AzCore/IO/FileIO.h>
#include <AzCore/Utils/Utils.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <PhysX/MeshAsset.h>

#include <regex> // TODO - we are currently replacing package:// with an absolute path

namespace ROS2
{
    URDFPrefabMaker::URDFPrefabMaker(const AZStd::string& modelFilePath, urdf::ModelInterfaceSharedPtr model, AZStd::string prefabPath)
        : m_model(model)
        , m_visualsMaker(modelFilePath, model->materials_)
        , m_collidersMaker(new CollidersMaker(modelFilePath))
        , m_stopBuildFlag(false)
        , m_prefabPath(std::move(prefabPath))
    {
        AZ_Assert(!m_prefabPath.empty(), "Prefab path is empty");
        AZ_Assert(m_model, "Model is nullptr");
    }

    URDFPrefabMaker::~URDFPrefabMaker()
    {
        m_stopBuildFlag = true;
        m_buildThread.join();
    }

    void URDFPrefabMaker::LoadURDF(BuildReadyCallback buildReadyCb)
    {
        m_notifyBuildReadyCb = buildReadyCb;

        // Request the build of collider meshes by constructing .assetinfo files.
        BuildAssetsForLink(m_model->root_link_);

        // Wait for all collider meshes to be ready
        m_buildThread = AZStd::thread(
            [&]()
            {
                {
                    AZ_Printf("CollisionMaker", "Waiting for URDF assets...");
                    while (!m_collidersMaker->m_meshesToBuild.empty() && !m_stopBuildFlag)
                    {
                        {
                            AZStd::lock_guard lock{ m_collidersMaker->m_buildMutex };
                            for (auto iter = m_collidersMaker->m_meshesToBuild.begin(); iter != m_collidersMaker->m_meshesToBuild.end();
                                 iter++)
                            {
                                AZ::Data::AssetId assetId;
                                AZ::Data::AssetType assetType = AZ::AzTypeInfo<PhysX::Pipeline::MeshAsset>::Uuid();
                                AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                                    assetId, &AZ::Data::AssetCatalogRequests::GetAssetIdByPath, iter->c_str(), assetType, false);
                                if (assetId.IsValid())
                                {
                                    AZ_Printf("CollisionMaker", "Asset %s found and valid...", iter->c_str());
                                    m_collidersMaker->m_meshesToBuild.erase(iter--);
                                }
                            }
                        }
                        if (!m_collidersMaker->m_meshesToBuild.empty())
                        {
                            AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(50));
                        }
                    }

                    AZ_Printf("CollisionMaker", "All URDF assets ready!");
                    // Notify the caller that we can continue with constructing the prefab.
                    m_notifyBuildReadyCb();
                }
            });
    }

    void URDFPrefabMaker::BuildAssetsForLink(urdf::LinkSharedPtr link)
    {
        m_collidersMaker->BuildColliders(link);
        for (auto childLink : link->child_links)
        {
            BuildAssetsForLink(childLink);
        }
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF()
    { // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping

        // recursively add all entities
        auto createEntityResult = AddEntitiesForLink(m_model->root_link_, AZ::EntityId());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }

        auto contentEntityId = createEntityResult.GetValue();
        AddRobotControl(contentEntityId);

        // Create prefab, save it to disk immediately
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        auto outcome = prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ contentEntityId }, m_prefabPath.c_str());
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            PrefabMakerUtils::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created %s prefab", m_prefabPath.c_str());
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
        // TODO - add unique namespace to the robot's top level frame
        entity->CreateComponent<ROS2FrameComponent>(link->name.c_str());

        m_visualsMaker.AddVisuals(link, entityId);
        m_collidersMaker->AddColliders(link, entityId);
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
            m_jointsMaker.AddJoint(link, childLink, childEntityId, entityId);
        }

        return AZ::Success(entityId);
    }

    void URDFPrefabMaker::AddRobotControl(AZ::EntityId rootEntityId)
    {
        // TODO - check for RigidBody
        ControlConfiguration controlConfiguration;
        controlConfiguration.m_robotConfiguration.m_body = rootEntityId;
        controlConfiguration.m_broadcastBusMode = false;
        controlConfiguration.m_topic = "cmd_vel";
        AZ::Entity* rootEntity = AzToolsFramework::GetEntityById(rootEntityId);
        rootEntity->CreateComponent<ROS2RobotControlComponent>(controlConfiguration);
    }
    const AZStd::string& URDFPrefabMaker::GetPrefabPath() const
    {
        return m_prefabPath;
    }
} // namespace ROS2
