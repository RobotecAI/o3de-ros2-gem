/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorRigidBodyComponent.h>
#include <Source/EditorShapeColliderComponent.h>
#include <Source/Utils.h>

namespace ROS2
{
    void CollidersMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    { // TODO - refactor out naming (colliders, visuals)
        auto colliderName = AZStd::string::format("%s_collider", link->name.c_str());
        int nameSuffixIndex = 1; // For disambiguation when multiple unnamed colliders are present. The order does not matter here
        for (auto collider : link->collision_array)
        { // one or more colliders - the array is used
            auto generatedName =
                link->visual_array.size() > 1 ? AZStd::string::format("%s_%d", colliderName.c_str(), nameSuffixIndex) : colliderName;
            nameSuffixIndex++;
            AddCollider(collider, entityId, generatedName);
        }

        if (link->collision_array.size() == 0)
        { // no colliders in the array - zero or one in total, the element member is used instead
            AddCollider(link->collision, entityId, colliderName);
        }
    }

    void CollidersMaker::AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZStd::string& generatedName)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }

        auto geometry = collision->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning("AddCollider", false, "No Geometry for a collider");
            return;
        }

        AddColliderToEntity(collision, entityId);
    }

    void CollidersMaker::AddColliderToEntity(urdf::CollisionSharedPtr collision, AZ::EntityId entityId)
    {
        // TODO - we are unable to set collider origin. Sub-entities don't work since they would need to parent visuals etc.
        // PrefabMakerUtils::SetEntityTransform(collision->origin, entityId);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        Physics::ColliderConfiguration colliderConfig;
        colliderConfig.m_position = URDF::TypeConversions::ConvertVector3(collision->origin.position);
        colliderConfig.m_rotation = URDF::TypeConversions::ConvertQuaternion(collision->origin.rotation);
        if (!collision->name.empty())
        {
            colliderConfig.m_tag = AZStd::string(collision->name.c_str());
        }

        auto geometry = collision->geometry;
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                Physics::SphereShapeConfiguration sphere(sphereGeometry->radius);
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, sphere);
            }
            break;
        case urdf::Geometry::BOX:
            {
                auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                Physics::BoxShapeConfiguration box(URDF::TypeConversions::ConvertVector3(boxGeometry->dim));
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, box);
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                entity->CreateComponent<PhysX::EditorShapeColliderComponent>();
                entity->CreateComponent(LmbrCentral::EditorCylinderShapeComponentTypeId);
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                entity->Activate();
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetHeight, cylinderGeometry->length);
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetRadius, cylinderGeometry->radius);
                entity->Deactivate();

                /* TODO - use shape config for EditorColliderComponent instead, provide convex mesh from cylinder
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                const AZ::u8 subdivisionCount = 32;
                const AZ::Vector3 scale(1.0f, 1.0f, 1.0f);
                AZStd::optional<AZStd::vector<AZ::Vector3>> points = PhysX::Utils::CreatePointsAtFrustumExtents(
                    cylinderGeometry->length, cylinderGeometry->radius, cylinderGeometry->radius, subdivisionCount);
                if (!points.has_value())
                {
                    AZ_Warning("AddColliderToEntity", false, "Could not generate cylinder points for collider");
                    return;
                }

                const AZStd::optional<Physics::CookedMeshShapeConfiguration> maybeShapeConfig =
                    PhysX::Utils::CreatePxCookedMeshConfiguration(points.value(), scale);
                if (!maybeShapeConfig.has_value())
                {
                    AZ_Warning("AddColliderToEntity", false, "Could not generate shapeConfig for collider");
                    return;
                }
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, maybeShapeConfig.value());
                */
            }
            break;
        case urdf::Geometry::MESH:
            {
                AZ_Warning("AddColliderToEntity", false, "mesh collider is not supported at this moment");
                entity->CreateComponent<PhysX::EditorColliderComponent>();
            }
            break;
        default:
            AZ_Warning("AddColliderToEntity", false, "Unsupported collider geometry type, %d", geometry->type);
            break;
        }
        if (!entity->FindComponent<PhysX::EditorRigidBodyComponent>())
        {   // This component could already be there if inertia was defined in URDF for this entity's link
            AZ_TracePrintf("AddColliderToEntity", "Adding RigidBody for entity id:%s", entityId.ToString().c_str());
            entity->CreateComponent<PhysX::EditorRigidBodyComponent>();
        }
    }
} // namespace ROS2
