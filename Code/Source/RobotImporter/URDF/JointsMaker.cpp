/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/JointsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    void JointsMaker::AddJointInformationToEntity(
        urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId childEntityId, AZ::EntityId parentEntityId)
    { // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        { // TODO - replace with std algorithm
            if (joint->child_link_name == childLink->name)
            { // Found a match!
                PrefabMakerUtils::SetEntityTransform(joint->parent_to_joint_origin_transform, childEntityId);
                return AddJoint(joint, childEntityId, parentEntityId);
            }
        }
    }

    bool JointsMaker::HasRequiredComponentsForJoint(AZ::EntityId entityId)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        return entity->FindComponent<PhysX::EditorColliderComponent>()
            ? true
            : entity->FindComponent<PhysX::EditorShapeColliderComponent>() != nullptr;
    }

    void JointsMaker::AddJoint(urdf::JointSharedPtr joint, AZ::EntityId childEntityId, AZ::EntityId parentEntityId)
    {
        if (!HasRequiredComponentsForJoint(childEntityId) || !HasRequiredComponentsForJoint(parentEntityId))
        {
            AZ_Error(
                "AddJoint",
                false,
                "Unable to add a joint %s without Collider component in both its own and parent entity",
                joint->name.c_str());
            return;
        }

        AZ::Entity* childEntity = AzToolsFramework::GetEntityById(childEntityId);
        PhysX::EditorJointComponent* jointComponent = nullptr;
        // TODO - ATM, there is no support withing Joint Components for the following:
        // TODO <calibration> <dynamics> <mimic>, friction, effort, velocity, joint safety and several joint types
        // TODO - apply <axis>
        switch (joint->type)
        { // TODO - replace with a generic member function
        case urdf::Joint::FIXED:
            {
                jointComponent = childEntity->CreateComponent<PhysX::EditorFixedJointComponent>();
            }
            break;
        case urdf::Joint::CONTINUOUS:
            // TODO - disable limits for Continuous type. API for this seems to be missing.
            [[fallthrough]];
        case urdf::Joint::REVOLUTE:
            { // Hinge
                jointComponent = childEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                childEntity->Activate();
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(childEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                    PhysX::AngleLimitsFloatPair(AZ::RadToDeg(joint->limits->upper), AZ::RadToDeg(joint->limits->upper)));
                childEntity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddJoint", false, "Unknown or unsupported joint type %d for joint %s", joint->type, joint->name.c_str());
            break;
        }

        if (!jointComponent)
        {
            return;
        }

        childEntity->Activate();
        PhysX::EditorJointRequestBus::Event(
            AZ::EntityComponentIdPair(childEntityId, jointComponent->GetId()),
            &PhysX::EditorJointRequests::SetEntityIdValue,
            PhysX::JointsComponentModeCommon::ParamaterNames::LeadEntity,
            parentEntityId);
        childEntity->Deactivate();
    }
} // namespace ROS2
