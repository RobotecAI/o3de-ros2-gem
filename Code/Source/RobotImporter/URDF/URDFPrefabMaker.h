/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/URDF/UrdfParser.h"
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

#include <filesystem> // TODO - instead, use AZ API for filesystem
#include "RobotImporter/URDF/RobotImporterInputInterface.h"

namespace ROS2
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class URDFPrefabMaker
    {
    public:
        URDFPrefabMaker(RobotImporterInputInterface& inputInterface);
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefabFromURDF(
            urdf::ModelInterfaceSharedPtr model, const AZStd::string& modelFilePath);

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLinkRecursively(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId);
        void AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId);
        void AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId);
        void AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId);
        void AddCollider(urdf::CollisionSharedPtr collider, AZ::EntityId entityId);
        void AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId);
        void AddJointInformationToEntity(urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId entityId);
        AZStd::string GetAssetPathFromModelPath(std::filesystem::path modelPath);

        AzToolsFramework::Prefab::PrefabPublicInterface* m_prefabInterface;
        RobotImporterInputInterface& m_robotImporterInputInterface;

        AZStd::string m_modelFilePath;
    };
} // namespace ROS2
