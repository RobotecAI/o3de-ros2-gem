/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Map/MapBus.h"
#include "ROS2SystemComponent.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2
{
//! The MapManagerROS2Component wraps the MapManagerComponent with ROS2 services and topics.
//! MapManagerComponent is accessed indirectly via MapRequestsBus.
class MapManagerROS2Component
: public AZ::Component
{
public:
    AZ_COMPONENT(MapManagerROS2Component, "{0ccfea4e-5275-4450-9a8f-a9862ef1dd20}");

    void Activate() override;
    void Deactivate() override;

    static void Reflect(AZ::ReflectContext* context);
    static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    //! TODO: ROS2 services and topics to communicate with MapManager.

    MapManagerROS2Component();
    ~MapManagerROS2Component();
};

}
