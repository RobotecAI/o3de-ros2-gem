/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2
{
    struct SpawnPointInfo
    {
        AZStd::string name;
        AZStd::string description;
        AZ::Vector3 position;
    };

    class SpawnPointComponent: public AZ::Component {
    public:
        AZ_COMPONENT(SpawnPointComponent, "{8CDC810F-9D25-4BD7-906D-6EE170CEBC55}", AZ::Component);

        // AZ::Component interface implementation.
        SpawnPointComponent() = default;
        ~SpawnPointComponent() = default;

        void Activate() override;
        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string GetName();
        AZStd::string GetDescription();

    private:
        AZStd::string m_name = {};
        AZStd::string m_description = {};
    };
}


