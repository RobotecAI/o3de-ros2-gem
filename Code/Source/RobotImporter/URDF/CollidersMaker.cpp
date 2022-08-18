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
        auto geometry = collision->geometry;
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
