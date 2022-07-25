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

#include "Map/MapBus.h"
#include "Map/GeodeticConfiguration.h"

namespace ROS2
{

//! The map manager component provides the context about the o3de scene in ROS2 environment.
//! It provides position to GPS location tool, conversion to map frame for any o3de Transform as well
//! as it handles the global information about map and odom frame names.
//! A hook entity can be used to describe the map origin in geographic coordinate system.
class MapManagerComponent
        : public AZ::Component
        , protected MapRequestBus::Handler
{
public:
    AZ_COMPONENT(MapManagerComponent, "{043ea1d0-5751-4ee9-af73-cca4ae11c475}");

    void Activate() override;
    void Deactivate() override;

    static void Reflect(AZ::ReflectContext* context);
    static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

    [[nodiscard]] AZ::Transform ConvertToMapCoordinateSystem(AZ::Transform transform) override;
    [[nodiscard]] AZ::Transform ConvertFromMapCoordinateSystem(AZ::Transform transform) override;
    [[nodiscard]] AZ::Vector3 WorldPositionToLatLon(const AZ::Vector3 &worldPosition) override;
    [[nodiscard]] AZ::Vector3 LatLonToWorldPosition(const AZ::Vector3 &latLonAlt) override;
    [[nodiscard]] AZStd::string GetMapFrameId() override { return m_mapFrameId;};
    [[nodiscard]] AZStd::string GetOdomFrameId() override { return m_odomFrameId;};

    MapManagerComponent();
    ~MapManagerComponent();

private:
    Map::GeodeticConfiguration m_geodeticConfiguration;

    AZStd::string m_mapFrameId = "map";
    AZStd::string m_odomFrameId = "odom";
};

}
