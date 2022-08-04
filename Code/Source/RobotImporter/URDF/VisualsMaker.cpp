/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/VisualsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/URDF/TypeConversions.h"

#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentConstants.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/EditorNonUniformScaleComponent.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/EditorShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>

namespace ROS2
{
    VisualsMaker::VisualsMaker(const AZStd::string& modelPath)
        : m_modelPath(modelPath)
    {
    }

    void VisualsMaker::AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        if (link->visual_array.size() == 0)
        { // one or zero visuals - element is used
            AddVisual(link->visual, entityId);
            return;
        }

        for (auto visual : link->visual_array)
        { // one or more visuals - array is used
            AddVisual(visual, entityId);
        }
    }

    void VisualsMaker::AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
    {
        if (!visual)
        { // it is ok not to have a visual in a link
            return;
        }

        if (!visual->geometry)
        { // non-empty visual should have a geometry
            AZ_Warning("AddVisual", false, "No Geometry for a visual");
            return;
        }

        // Since o3de does not allow origin for visuals, we need to create a sub-entity and store visual there
        AZStd::string subEntityName = "visual"; // TODO add index and maybe a name prefix for multiple visual case
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName.c_str());
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("AddVisual", false, "Unable to create a sub-entity for visual element %s", subEntityName.c_str());
            return;
        }
        auto visualEntityId = createEntityResult.GetValue();
        AddVisualToEntity(visual, visualEntityId);
        AddMaterialForVisual(visual, visualEntityId);
    }

    void VisualsMaker::AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
    {
        // Apply transform as per origin
        PrefabMakerUtils::SetEntityTransform(visual->origin, entityId);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto geometry = visual->geometry;
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                entity->CreateComponent(LmbrCentral::EditorSphereShapeComponentTypeId);
                entity->Activate();
                LmbrCentral::SphereShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::SphereShapeComponentRequests::SetRadius, sphereGeometry->radius);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                entity->CreateComponent(LmbrCentral::EditorCylinderShapeComponentTypeId);
                entity->Activate();
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetRadius, cylinderGeometry->radius);
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetHeight, cylinderGeometry->length);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::BOX:
            {
                auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                entity->CreateComponent(LmbrCentral::EditorBoxShapeComponentTypeId);
                AZ::Vector3 boxDimensions = URDF::TypeConversions::ConvertVector3(boxGeometry->dim);
                entity->Activate();
                LmbrCentral::BoxShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::BoxShapeComponentRequests::SetBoxDimensions, boxDimensions);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::MESH:
            {
                auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);

                // TODO - a PoC solution, replace with something generic, robust, proper
                std::filesystem::path modelPath(m_modelPath.c_str());
                modelPath = modelPath.remove_filename();
                auto relativePathToMesh = std::regex_replace(meshGeometry->filename, std::regex("package://"), "");
                modelPath += relativePathToMesh;

                // Get asset path for a given model path
                auto assetPath = PrefabMakerUtils::GetAssetPathFromModelPath(modelPath);

                entity->CreateComponent(AZ::Render::EditorMeshComponentTypeId);

                // Prepare scale
                AZ::Vector3 scaleVector = URDF::TypeConversions::ConvertVector3(meshGeometry->scale);
                bool isUniformScale = AZ::IsClose(scaleVector.GetMaxElement(), scaleVector.GetMinElement(), AZ::Constants::FloatEpsilon);
                if (!isUniformScale)
                {
                    entity->CreateComponent<AzToolsFramework::Components::EditorNonUniformScaleComponent>();
                }

                entity->Activate();
                // Set asset path
                AZ::Render::MeshComponentRequestBus::Event(
                    entityId, &AZ::Render::MeshComponentRequestBus::Events::SetModelAssetPath, assetPath.c_str());
                // Set scale, uniform or non-uniform
                if (isUniformScale)
                {
                    AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalUniformScale, scaleVector.GetX());
                }
                else
                {
                    AZ::NonUniformScaleRequestBus::Event(entityId, &AZ::NonUniformScaleRequests::SetScale, scaleVector);
                }
                entity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddVisual", false, "Unsupported visual geometry type, %d", geometry->type);
            return;
        }
    }

    void VisualsMaker::AddMaterialForVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
    {
        // TODO URDF does not include information from <gazebo> tags with specific materials, diffuse, specular and emissive tags
        if (!visual->material || !visual->geometry)
        {
            // Material is optional, and it requires geometry
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ::Color materialColor = URDF::TypeConversions::ConvertColor(visual->material->color);
        bool isPrimitive = visual->geometry->type != urdf::Geometry::MESH;
        if (isPrimitive)
        { // For primitives, set the color in the shape component
            entity->Activate();
            LmbrCentral::EditorShapeComponentRequestsBus::Event(
                entityId, &LmbrCentral::EditorShapeComponentRequests::SetShapeColor, materialColor);
            entity->Deactivate();
            return;
        }

        // Mesh visual - we can have either filename or default material with a given color
        // TODO - handle texture_filename - file materials
        entity->CreateComponent(AZ::Render::MaterialComponentTypeId);
        AZ_Warning("AddVisual", false, "Setting color for material %s", visual->material->name.c_str());
        entity->Activate();
        AZ::Render::MaterialComponentRequestBus::Event(
            entityId,
            &AZ::Render::MaterialComponentRequestBus::Events::SetPropertyValue,
            AZ::Render::DefaultMaterialAssignmentId,
            "settings.color",
            AZStd::any(materialColor));
        entity->Deactivate();
    }
} // namespace ROS2
