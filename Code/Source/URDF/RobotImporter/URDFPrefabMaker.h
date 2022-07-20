/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include "URDF/UrdfParser.h"

namespace ROS2
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class URDFPrefabMaker
    {
    public:
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefabFromURDF(urdf::ModelInterfaceSharedPtr model);
    };
} // namespace ROS2
