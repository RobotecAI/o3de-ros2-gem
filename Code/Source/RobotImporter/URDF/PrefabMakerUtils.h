/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

#include <filesystem> // TODO - instead, use AZ API for filesystem

namespace ROS2
{
    //! Common utils for Prefab Maker classes
    class PrefabMakerUtils
    {
    public:
        static void AddRequiredComponentsToEntity(AZ::EntityId entityId);
        static AZStd::string GetAssetPathFromModelPath(std::filesystem::path modelPath);
        static void SetEntityTransform(const urdf::Pose& origin, AZ::EntityId entityId);
        static AzToolsFramework::Prefab::PrefabEntityResult CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name);
        static AzToolsFramework::EntityIdList GetColliderChildren(AZ::EntityId parentEntityId);
        static bool HasCollider(AZ::EntityId entityId);
    };
} // namespace ROS2
