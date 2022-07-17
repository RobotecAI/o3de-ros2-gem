/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Vector3.h>

#include <Map/MapBus.h>
#include "Map/MapConfiguration.h"

#pragma once
namespace ROS2
{
class MapManagerComponent
        : public AZ::Component
        , protected MapRequestBus::Handler
{
public:
    AZ_COMPONENT(MapManagerComponent, "{043ea1d0-5751-4ee9-af73-cca4ae11c475}");

    void Activate() override;
    void Deactivate() override;

    static void Reflect(AZ::ReflectContext* context);

    [[nodiscard]] AZ::Vector3 LocalToLatLon(const AZ::Vector3 &local) override;

    MapManagerComponent();
    ~MapManagerComponent();

private:
    MapConfiguration m_mapConfiguration;
};

}
