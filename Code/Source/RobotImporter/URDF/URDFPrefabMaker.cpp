/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include "Frame/ROS2FrameComponent.h"
#include "RobotImporter/URDF/TypeConversions.h"

#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/string/conversions.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Component/EditorComponentAPIBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <AzToolsFramework/Prefab/PrefabPublicRequestBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorRigidBodyComponent.h>
#include <Source/EditorShapeColliderComponent.h>

#include <regex> // TODO - we are currently replacing package:// with an absolute path

namespace ROS2
{
    namespace Internal
    {
        void AddRequiredComponentsToEntity(AZ::EntityId entityId)
        {
            AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
            AzToolsFramework::EditorEntityContextRequestBus::Broadcast(
                &AzToolsFramework::EditorEntityContextRequests::AddRequiredComponents, *entity);
        }
    } // namespace Internal

    URDFPrefabMaker::URDFPrefabMaker(RobotImporterInputInterface& inputInterface)
        : m_robotImporterInputInterface(inputInterface)
    {
        m_prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF(
        urdf::ModelInterfaceSharedPtr model, const AZStd::string& modelFilePath)
    { // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
        // TODO - add a check if the prefab with a given name already exists. Choice to cancel, overwrite or suffix name

        m_modelFilePath = modelFilePath;
        auto modelName = model->getName().c_str();

        AZ_TracePrintf("CreatePrefabFromURDF", "Creating a prefab for URDF model with name %s", modelName);
        auto createEntityResult = AddEntitiesForLinkRecursively(model->root_link_, AZ::EntityId());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }
        auto rootEntityId = createEntityResult.GetValue();

        auto prefabName = AZStd::string::format("%s.%s", modelName, "prefab");
        auto prefabFileName = AZStd::string(AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName.c_str());

        if (AZ::IO::FileIOBase::GetInstance()->Exists(prefabFileName.c_str()))
        {
            switch (m_robotImporterInputInterface.GetExistingPrefabAction())
            {
            case RobotImporterInputInterface::ExistingPrefabAction::Cancel:
                return AZ::Failure(AZStd::string("User cancelled"));
            case RobotImporterInputInterface::ExistingPrefabAction::Overwrite:
                break;
            case RobotImporterInputInterface::ExistingPrefabAction::NewName:
                prefabFileName = m_robotImporterInputInterface.GetNewPrefabPath();
                break;
            }
        }

        auto outcome = m_prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList({ rootEntityId }), prefabFileName.c_str());
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            Internal::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        return outcome;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLinkRecursively(
        urdf::LinkSharedPtr link, AZ::EntityId parentEntityId)
    {
        if (!link)
        {
            AZ::Failure(AZStd::string("Failed to create prefab entity - link is null"));
        }

        auto createEntityResult = m_prefabInterface->CreateEntity(parentEntityId, AZ::Vector3());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }

        AZ::EntityId entityId = createEntityResult.GetValue();
        if (!entityId.IsValid())
        { // Verify that a valid entity is created.
            return AZ::Failure(AZStd::string("Invalid id for created entity"));
        }

        AZ_TracePrintf(
            "AddEntitiesForLinkRecursively", "Processing entity id:%s with link name:%s", entityId.ToString().c_str(), link->name.c_str());
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZStd::string entityName(link->name.c_str());
        entity->SetName(entityName);
        entity->Deactivate();
        Internal::AddRequiredComponentsToEntity(entityId);
        entity->CreateComponent<ROS2FrameComponent>(entityName);

        AddVisuals(link, entityId);
        AddColliders(link, entityId);
        AddInertial(link->inertial, entityId);

        for (auto childLink : link->child_links)
        {
            auto outcome = AddEntitiesForLinkRecursively(childLink, entityId); // recursive call
            if (!outcome.IsSuccess())
            { // TODO - decide on behavior. Still proceed to load other children?
                AZ_Warning("AddEntitiesForLinkRecursively", false, "Unable to add entity due to an error: %s", outcome.GetError().c_str());
                continue;
            }

            AZ::EntityId childEntityId = outcome.GetValue();
            AddJointInformationToEntity(link, childLink, childEntityId);
        }

        return AZ::Success(entityId);
    }

    void URDFPrefabMaker::AddJointInformationToEntity(urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId entityId)
    { // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        { // TODO - replace with std algoritm
            if (joint->child_link_name == childLink->name)
            { // Found a match!
                // TODO - handle joint types etc. (a dedicated component) - check existing JointComponent

                // Get URDF pose elements
                urdf::Vector3 urdfPosition = joint->parent_to_joint_origin_transform.position;
                urdf::Rotation urdfRotation = joint->parent_to_joint_origin_transform.rotation;
                AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(urdfRotation);
                AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(urdfPosition);
                AZ::Transform transformForChild(azPosition, azRotation, 1.0f);

                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                if (!transformInterface)
                {
                    AZ_Error("AddJointInformationToEntity", false, "Missing Transform component!");
                    continue;
                }
                // Alternative - but requires an active component. We would activate/deactivate a lot. Or - do it in a second pass.
                // AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, transformForChild);

                transformInterface->SetLocalTM(transformForChild);
                break;
            }
        }
    }

    void URDFPrefabMaker::AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId)
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

    void URDFPrefabMaker::AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
    {
        if (!visual)
        { // it is ok not to have a visual in a link
            return;
        }

        // TODO handle origin

        auto geometry = visual->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning("AddVisual", false, "No Geometry for a visual");
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

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
                std::filesystem::path modelPath(m_modelFilePath.c_str());
                modelPath = modelPath.remove_filename();
                auto relativePathToMesh = std::regex_replace(meshGeometry->filename, std::regex("package://"), "");
                modelPath += relativePathToMesh;

                // Get asset path for a given model path
                auto assetPath = GetAssetPathFromModelPath(modelPath);

                entity->CreateComponent(AZ::Render::EditorMeshComponentTypeId);
                entity->Activate();
                AZ::Render::MeshComponentRequestBus::Event(
                    entityId, &AZ::Render::MeshComponentRequestBus::Events::SetModelAssetPath, assetPath.c_str());
                entity->Deactivate();

                // TODO apply scale
            }
            break;
        default:
            AZ_Warning("AddVisual", false, "Unsupported visual geometry type, %d", geometry->type);
            return;
        }

        // TODO handle material
    }

    void URDFPrefabMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
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

    void URDFPrefabMaker::AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId)
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

    void URDFPrefabMaker::AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId)
    {
        if (!inertial)
        { // it is ok not to have inertia in a link
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        // TODO - consider explicit 2 arg constructor instead
        PhysX::EditorRigidBodyConfiguration rigidBodyConfiguration;
        rigidBodyConfiguration.m_mass = inertial->mass;
        // TODO - is the origin.rotation part applicable? Does non-zero make value sense? Investigate.
        rigidBodyConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);

        // Inertia tensor is symmetrical
        auto inertiaMatrix = AZ::Matrix3x3::CreateFromRows(
            AZ::Vector3(inertial->ixx, inertial->ixy, inertial->ixz),
            AZ::Vector3(inertial->ixy, inertial->iyy, inertial->iyz),
            AZ::Vector3(inertial->ixz, inertial->iyz, inertial->izz));
        rigidBodyConfiguration.m_inertiaTensor = inertiaMatrix;
        entity->CreateComponent<PhysX::EditorRigidBodyComponent>(rigidBodyConfiguration);
    }

    AZStd::string URDFPrefabMaker::GetAssetPathFromModelPath(std::filesystem::path modelPath)
    {
        bool assetFound = false;
        AZ::Data::AssetInfo assetInfo;
        AZStd::string watchDir;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            assetFound,
            &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
            modelPath.c_str(),
            assetInfo,
            watchDir);

        if (!assetFound)
        {
            AZ_Error("AddVisuals", false, "Could not find model asset for %s", modelPath.c_str());
            return "";
        }

        auto relativePath = std::filesystem::path(assetInfo.m_relativePath.c_str());
        relativePath.replace_extension("azmodel");
        auto assetPath = AZStd::string(relativePath.string().c_str());
        AZStd::to_lower(assetPath.begin(), assetPath.end());

        return assetPath;
    }
} // namespace ROS2
