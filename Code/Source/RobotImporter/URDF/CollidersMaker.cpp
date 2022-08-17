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

        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;
        if (!isPrimitiveShape)
        { // TODO - implement mesh colliders
            return;
        }

        entity->CreateComponent<PhysX::EditorShapeColliderComponent>();
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                entity->CreateComponent(LmbrCentral::EditorSphereShapeComponentTypeId);
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                entity->Activate();
                LmbrCentral::SphereShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::SphereShapeComponentRequests::SetRadius, sphereGeometry->radius);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::BOX:
            {
                entity->CreateComponent(LmbrCentral::EditorBoxShapeComponentTypeId);
                auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                entity->Activate();
                LmbrCentral::BoxShapeComponentRequestsBus::Event(
                    entityId,
                    &LmbrCentral::BoxShapeComponentRequests::SetBoxDimensions,
                    URDF::TypeConversions::ConvertVector3(boxGeometry->dim));
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                entity->CreateComponent(LmbrCentral::EditorCylinderShapeComponentTypeId);
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                entity->Activate();
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetHeight, cylinderGeometry->length);
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetRadius, cylinderGeometry->radius);
                entity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddCollider", false, "Unsupported collider geometry type, %d", geometry->type);
            break;
        }
    }
} // namespace ROS2
