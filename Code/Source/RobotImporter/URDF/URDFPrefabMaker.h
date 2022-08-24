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
    public:
        URDFPrefabMaker(const AZStd::string& modelFilePath, urdf::ModelInterfaceSharedPtr model, AZStd::string prefabPath);
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefabFromURDF();

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId);
        void AddRobotControl(AZ::EntityId rootEntityId);
        urdf::ModelInterfaceSharedPtr m_model;
        AZStd::string m_prefabPath;
        VisualsMaker m_visualsMaker;
        CollidersMaker m_collidersMaker;
        InertialsMaker m_inertialsMaker;
        JointsMaker m_jointsMaker;
    };
} // namespace ROS2
