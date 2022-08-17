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
    //! Populates a given entity with all the contents of the <collider> tag in robot description.
    class CollidersMaker
    {
    public:
        //! Add zero, one or many collider elements to a given entity (depending on link content).
        //! @param link A parsed URDF tree link node which could hold information about colliders.
        //! @param entityId A non-active entity which will be affected.
        void AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId);

    private:
        void AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId);
    };
} // namespace ROS2
