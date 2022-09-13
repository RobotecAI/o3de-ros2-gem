/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnPointComponent.h"
#include <AzCore/Serialization/EditContext.h>

namespace ROS2 {
    void SpawnPointComponent::Activate() {

    }

    void SpawnPointComponent::Deactivate() {
    }

    void SpawnPointComponent::Reflect(AZ::ReflectContext *context) {
        if (AZ::SerializeContext *serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
            serialize->Class<SpawnPointComponent, AZ::Component>()->Version(1)
                    ->Field("Name", &SpawnPointComponent::m_name)
                    ->Field("Description", &SpawnPointComponent::m_description);

            if (AZ::EditContext *ec = serialize->GetEditContext()) {
                ec->Class<SpawnPointComponent>("Spawn Point", "Spawn point location")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "Marks a place which is good for spawning")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &SpawnPointComponent::m_name,
                                "Name",
                                "Name")
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &SpawnPointComponent::m_description,
                                "Description",
                                "Description");
            }
        }
    }

    AZStd::string SpawnPointComponent::GetName() {
        return m_name;
    }

    AZStd::string SpawnPointComponent::GetDescription() {
        return m_description;
    }
}

#include "SpawnPointComponent.h"
