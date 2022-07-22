/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>

#include "SpawnPointsConfiguration.h"

namespace ROS2::Map {

    void SpawnPointsConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SpawnPointsConfiguration>()
                    ->Version(1)
                    ->Field("spawnPoints", &SpawnPointsConfiguration::m_spawnPoints)
                    ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SpawnPointsConfiguration>("Spawn points", "Available spawn points")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SpawnPointsConfiguration::m_spawnPoints,
                                      "Spawn points", "Available spawn points")
                        ;
            }
        }
    }

    AZStd::vector<AZ::Transform> SpawnPointsConfiguration::GetAvailableSpawnPointsTransforms()
    {
        AZStd::vector<AZ::Transform> result;

        for(auto & m_spawnPoint : m_spawnPoints)
        {
            if (m_spawnPoint.IsValid()) {
                result.push_back();
            }
            AZ::TransformBus::EventResult(
                    result.back(),
                    m_spawnPoint,
                    &AZ::TransformBus::Events::GetWorldTM);
        }

        return result;
    }
}
