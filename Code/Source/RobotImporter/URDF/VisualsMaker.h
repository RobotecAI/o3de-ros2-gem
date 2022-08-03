/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/Component/EntityId.h>

namespace ROS2
{
    //! Populates a given entity with all the contents of the <visual> tag in robot description
    class VisualsMaker
    {
    public:
        VisualsMaker(const AZStd::string& modelPath);

        //! Add zero, one or many visual elements to a given entity (depending on link content).
        //! Note that a sub-entity will be added to hold each visual (since they can have different transforms).
        //! @param link A parsed URDF tree link node which could hold information about visuals.
        //! @param entityId A non-active entity which will be affected.
        void AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId);

    private:
        void AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId);
        void AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId);
        void AddMaterialForVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId);

        AZStd::string m_modelPath; // TODO
    };
} // namespace ROS2
