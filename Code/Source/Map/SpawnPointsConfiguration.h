/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Entity.h>

namespace ROS2::Map {

class SpawnPointsConfiguration {
public:
    AZ_TYPE_INFO(SpawnPointsConfiguration, "{f860a0f9-e24a-4a05-8dbf-a69bf061b675}");

    static void Reflect(AZ::ReflectContext* context);
    AZStd::vector<AZ::Transform> GetAvailableSpawnPointsTransforms();
    AZStd::vector<AZ::EntityId> m_spawnPoints = {};

private:
    AZStd::string m_spawnPointsFrameId;
};

}


