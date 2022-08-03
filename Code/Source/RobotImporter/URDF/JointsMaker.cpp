/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/JointsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"

namespace ROS2
{
    void JointsMaker::AddJointInformationToEntity(urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId childEntityId)
    { // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        { // TODO - replace with std algorithm
            if (joint->child_link_name == childLink->name)
            { // Found a match!
                // TODO - handle joint types etc. (a dedicated component) - check existing JointComponent
                PrefabMakerUtils::SetEntityTransform(joint->parent_to_joint_origin_transform, childEntityId);
                break;
            }
        }
    }
} // namespace ROS2
