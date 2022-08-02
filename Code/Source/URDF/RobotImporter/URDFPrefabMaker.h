/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "URDF/UrdfParser.h"
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

#include <filesystem> // TODO - instead, use AZ API for filesystem

namespace ROS2
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class URDFPrefabMaker
    {
    public:
        URDFPrefabMaker();
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefabFromURDF(
            urdf::ModelInterfaceSharedPtr model, const AZStd::string& modelFilePath);

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId);
        void AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId);
        void AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId);
        void AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId);
        void AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId);
        void AddCollider(urdf::CollisionSharedPtr collider, AZ::EntityId entityId);
        void AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId);
        void AddJointInformationToEntity(urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId entityId);
        AZStd::string GetAssetPathFromModelPath(std::filesystem::path modelPath);
        void SetEntityTransform(const urdf::Pose& origin, AZ::EntityId entityId);
        AzToolsFramework::Prefab::PrefabEntityResult CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name);

        AzToolsFramework::Prefab::PrefabPublicInterface* m_prefabInterface;
        AZStd::string m_modelFilePath;
    };
} // namespace ROS2
