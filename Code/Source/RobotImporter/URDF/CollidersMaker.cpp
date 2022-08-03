/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/CollidersMaker.h"
#include <Source/EditorColliderComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    void CollidersMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        if (link->collision_array.size() == 0)
        { // one or zero colliders - element is used
            AddCollider(link->collision, entityId);
            return;
        }

        for (auto collider : link->collision_array)
        { // one or more colliders - array is used
            AddCollider(collider, entityId);
        }
    }

    void CollidersMaker::AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto geometry = collision->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning("AddCollider", false, "No Geometry for a collider");
            return;
        }

        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH; // class TriangleMeshShapeConfiguration : public ShapeConfiguration
        if (isPrimitiveShape)
        {
            Physics::ColliderConfiguration colliderConfig;
            colliderConfig.m_position = URDF::TypeConversions::ConvertVector3(collision->origin.position);
            colliderConfig.m_rotation = URDF::TypeConversions::ConvertQuaternion(collision->origin.rotation);
            colliderConfig.m_tag = AZStd::string(collision->name.c_str());
            Physics::ShapeConfiguration* shapeConfig = nullptr; // This will be initialized and passed by const reference to be copied.
            switch (geometry->type)
            {
            case urdf::Geometry::SPHERE:
                {
                    auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                    Physics::SphereShapeConfiguration sphere(sphereGeometry->radius);
                    shapeConfig = &sphere;
                }
                break;
            case urdf::Geometry::BOX:
                {
                    auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                    Physics::BoxShapeConfiguration box(URDF::TypeConversions::ConvertVector3(boxGeometry->dim));
                    shapeConfig = &box;
                }
                break;
            case urdf::Geometry::CYLINDER:
                {
                    auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                    Physics::CapsuleShapeConfiguration capsule(cylinderGeometry->length, cylinderGeometry->radius);
                    shapeConfig = &capsule;
                }
                break;
            case urdf::Geometry::MESH:
                {
                    auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
                    Physics::TriangleMeshShapeConfiguration triangleMesh;
                    // TODO - fill in this mesh shape configuration fields. Look at the memory management.
                    *shapeConfig = triangleMesh;
                }
                break;
            default:
                AZ_Warning("AddCollider", false, "Unsupported collider geometry type, %d", geometry->type);
                break;
            }

            entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, *shapeConfig);
            // TODO - set name as in collision->name
        }
    }
} // namespace ROS2
