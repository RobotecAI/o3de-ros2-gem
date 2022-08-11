/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/InertialsMaker.h"
#include "RobotImporter/URDF/JointsMaker.h"
#include "RobotImporter/URDF/VisualsMaker.h"
#include "UrdfParser.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

namespace ROS2
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class URDFPrefabMaker
    {
        typedef std::function<void()> BuildReadyCallback;

    public:
        URDFPrefabMaker(const AZStd::string& modelFilePath, urdf::ModelInterfaceSharedPtr model);
        ~URDFPrefabMaker();

        //! Loads URDF file and builds all required meshes and colliders.
        //! @param buildReadyCb Function to call when the build finishes.
        void LoadURDF(BuildReadyCallback buildReadyCb);

        //! Constructs prefab from URDF, all meshes and colliders must be ready before calling this function.
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefabFromURDF();

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId);
        void BuildAssetsForLink(urdf::LinkSharedPtr link);
        void AddRobotControl(AZ::EntityId rootEntityId);

        urdf::ModelInterfaceSharedPtr m_model;
        VisualsMaker m_visualsMaker;
        AZStd::shared_ptr<CollidersMaker> m_collidersMaker;
        InertialsMaker m_inertialsMaker;
        JointsMaker m_jointsMaker;

        BuildReadyCallback m_notifyBuildReadyCb;

        AZStd::atomic_bool m_stopBuildFlag;
        AZStd::thread m_buildThread;
    };
} // namespace ROS2
