/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Map
{
//! Configuration for handling geodetic map info.
//! This configuration allows setting a geographic coordinate system hook to a simulation scene.
//! System hook points to the real-world location in latitude, longitude, and altitude offsets.
class GeodeticConfiguration
{
public:
    AZ_TYPE_INFO(GeodeticConfiguration, "{094e5059-698a-42ad-ae2d-36be20868004}");

    static void Reflect(AZ::ReflectContext* context);

    //! Updates the hook Transform
    void SetHook();

    bool m_useMapHook = false;
    bool m_isMapHookSet = false;
    AZ::EntityId m_mapHook = AZ::EntityId{};
    AZ::Transform m_mapHookTransform = AZ::Transform::CreateIdentity();

    float m_originLatitudeDeg = 0.0f;
    float m_originLongitudeDeg = 0.0f;
    float m_originAltitude = 0.0f;

private:
    AZ::Outcome<void, AZStd::string> ValidateHandle(void* newValue, [[maybe_unused]] const AZ::Uuid& valueType);
    bool IsMapHookUsed() const;
};
}