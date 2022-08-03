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
    //! Populates a given entity with all the contents of the <joint> tag in robot description.
    class JointsMaker
    {
    public:
        //! Add zero or one joint elements to a given entity (depending on link content).
        //! @param parentLink A parent link for the joint.
        //! @param childLink A child link for the joint.
        //! @param childEntityId A non-active entity which will be populated with Joint components.
        void AddJointInformationToEntity(urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId childEntityId);
    };
} // namespace ROS2
