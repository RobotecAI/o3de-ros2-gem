/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/InertialsMaker.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Source/EditorRigidBodyComponent.h>

namespace ROS2
{
    void InertialsMaker::AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId)
    {
        if (!inertial)
        { // it is ok not to have inertia in a link
            return;
        }

        AZ_TracePrintf("AddInertial", "Processing inertial for entity id:%s", entityId.ToString().c_str());

        // TODO - this is likely invalid due to domain mismatch
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        // TODO - consider explicit 2 arg constructor instead
        PhysX::EditorRigidBodyConfiguration rigidBodyConfiguration;
        rigidBodyConfiguration.m_mass = inertial->mass;
        // TODO - is the origin.rotation part applicable? Does non-zero make value sense? Investigate.
        // TODO - this should be in relationship to link, not the collider origin already applied in this entity
        rigidBodyConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);

        // Inertia tensor is symmetrical
        auto inertiaMatrix = AZ::Matrix3x3::CreateFromRows(
            AZ::Vector3(inertial->ixx, inertial->ixy, inertial->ixz),
            AZ::Vector3(inertial->ixy, inertial->iyy, inertial->iyz),
            AZ::Vector3(inertial->ixz, inertial->iyz, inertial->izz));
        rigidBodyConfiguration.m_inertiaTensor = inertiaMatrix;
        entity->CreateComponent<PhysX::EditorRigidBodyComponent>(rigidBodyConfiguration);
    }
} // namespace ROS2
