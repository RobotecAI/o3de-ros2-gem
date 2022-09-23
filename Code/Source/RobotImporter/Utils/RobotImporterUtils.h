/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/std/string/string.h"
#include "RobotImporter/URDF/UrdfParser.h"
namespace ROS2
{
    namespace Utils
    {
        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        /// Create component for given entity in safe way.
        /// \param entityId entity that will own component
        /// \param componentType Uuid of component to create
        /// \return created componentId, if fails returns invalid id
        AZ::ComponentId CreateComponent(const AZ::EntityId entityId, const AZ::Uuid componentType);

    } // namespace Utils
} // namespace ROS2