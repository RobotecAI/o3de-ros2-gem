/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "URDF/RobotImporter/URDFPrefabMaker.h"
#include "Frame/ROS2FrameComponent.h"
#include "URDF/RobotImporter/TypeConversions.h"

#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentConstants.h>
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
#include <LmbrCentral/Shape/EditorShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>
#include <Source/EditorBallJointComponent.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
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

    URDFPrefabMaker::URDFPrefabMaker()
    {
        m_prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF(
        urdf::ModelInterfaceSharedPtr model, const AZStd::string& modelFilePath)
    { // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
        // TODO - add a check if the prefab with a given name already exists. Choice to cancel, overwrite or suffix name

        m_modelFilePath = modelFilePath;

        // recursively add all entities
        AZ_TracePrintf("CreatePrefabFromURDF", "Creating a prefab for URDF model with name %s", model->getName().c_str());
        auto createEntityResult = AddEntitiesForLink(model->root_link_, AZ::EntityId());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }

        auto prefabName = AZStd::string::format("%s.%s", model->getName().c_str(), "prefab");
        auto newPrefabPath = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName.c_str();
        auto contentEntityId = createEntityResult.GetValue();

        // Create prefab, save it to disk immediately

        auto outcome = m_prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ contentEntityId }, newPrefabPath);
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            Internal::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created %s prefab", prefabName.c_str());
        return outcome;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name)
    {
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

        AZ_TracePrintf("CreateEntity", "Processing entity id:%s with name:%s", entityId.ToString().c_str(), name.c_str());
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZStd::string entityName(name.c_str());
        entity->SetName(entityName);
        entity->Deactivate();
        Internal::AddRequiredComponentsToEntity(entityId);
        return createEntityResult;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId)
    {
        if (!link)
        {
            AZ::Failure(AZStd::string("Failed to create prefab entity - link is null"));
        }

        auto createEntityResult = CreateEntity(parentEntityId, link->name.c_str());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }
        AZ::EntityId entityId = createEntityResult.GetValue();
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        // Add ROS2FrameComponent - TODO: only for joints
        entity->CreateComponent<ROS2FrameComponent>(link->name.c_str());

        AddVisuals(link, entityId);
        AddColliders(link, entityId);
        AddInertial(link->inertial, entityId);

        for (auto childLink : link->child_links)
        {
            auto outcome = AddEntitiesForLink(childLink, entityId); // recursive call
            if (!outcome.IsSuccess())
            { // TODO - decide on behavior. Still proceed to load other children?
                AZ_Warning("AddEntitiesForLink", false, "Unable to add entity due to an error: %s", outcome.GetError().c_str());
                continue;
            }

            AZ::EntityId childEntityId = outcome.GetValue();
            AddJointInformationToEntity(link, childLink, childEntityId, entityId);
        }

        return AZ::Success(entityId);
    }

    void URDFPrefabMaker::AddJointInformationToEntity(
        urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId childEntityId, AZ::EntityId parentEntityId)
    { // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        { // TODO - replace with std algorithm
            if (joint->child_link_name == childLink->name)
            { // Found a match!
                SetEntityTransform(joint->parent_to_joint_origin_transform, childEntityId);
                // TODO - apply <axis>
                /*
                PhysX::JointComponentConfiguration jointComponentConfiguration(AZ::Transform::Identity(), parentEntityId, childEntityId);
                PhysX::JointGenericProperties jointGenericProperties;
                PhysX::JointLimitProperties jointLimitProperties;
                */

                AZ::Entity* childEntity = AzToolsFramework::GetEntityById(childEntityId);
                AZ::Entity* parentEntity = AzToolsFramework::GetEntityById(parentEntityId);

                if (!childEntity->FindComponent<PhysX::EditorColliderComponent>() ||
                    !parentEntity->FindComponent<PhysX::EditorColliderComponent>())
                {
                    AZ_Error(
                        "AddJointInformationToEntity",
                        false,
                        "Unable to add a joint %s without Collider component in both its own and parent entity",
                        joint->name.c_str());
                    return;
                }

                PhysX::EditorJointComponent* jointComponent = nullptr;
                // TODO - ATM, there is no support in Joint Components for the following:
                // TODO <calibration> <dynamics> <mimic>, friction, effort, velocity, joint safety and several joint types
                switch (joint->type)
                { // TODO - replace with a generic member function
                case urdf::Joint::FIXED:
                    {
                        jointComponent = childEntity->CreateComponent<PhysX::EditorFixedJointComponent>();
                    }
                    break;
                case urdf::Joint::CONTINUOUS:
                    [[fallthrough]];
                case urdf::Joint::REVOLUTE:
                    { // Hinge
                        jointComponent = childEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                        childEntity->Deactivate();
                        PhysX::EditorJointRequestBus::Event(
                            AZ::EntityComponentIdPair(childEntityId, jointComponent->GetId()),
                            &PhysX::EditorJointRequests::SetLinearValuePair,
                            PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                            PhysX::AngleLimitsFloatPair(AZ::RadToDeg(joint->limits->upper), AZ::RadToDeg(joint->limits->upper)));
                        childEntity->Activate();
                    }
                    break;
                case urdf::Joint::FLOATING:
                    /*
                    { // ~Ball
                        childEntity->CreateComponent<PhysX::BallJointComponent>(
                            jointComponentConfiguration, jointGenericProperties, jointLimitProperties);
                    }
                    break;
                    */
                default:
                    AZ_Warning(
                        "AddJointInformationToEntity",
                        false,
                        "Unknown or unsupported joint type %d for joint %s",
                        joint->type,
                        joint->name.c_str());
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
                break;
            }
        }
    }

    void URDFPrefabMaker::SetEntityTransform(const urdf::Pose& origin, AZ::EntityId entityId)
    {
        urdf::Vector3 urdfPosition = origin.position;
        urdf::Rotation urdfRotation = origin.rotation;
        AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(urdfRotation);
        AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(urdfPosition);
        AZ::Transform tf(azPosition, azRotation, 1.0f);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();

        if (!transformInterface)
        {
            AZ_Error("SetEntityTransform", false, "Missing Transform component!");
            return;
        }
        // Alternative - but requires an active component. We would activate/deactivate a lot. Or - do it in a second pass.
        // AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, transformForChild);

        transformInterface->SetLocalTM(tf);
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

    void URDFPrefabMaker::AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
    {
        // Apply transform as per origin
        SetEntityTransform(visual->origin, entityId);

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
    }

    void URDFPrefabMaker::AddMaterialForVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
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

    void URDFPrefabMaker::AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId)
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
        auto createEntityResult = CreateEntity(entityId, subEntityName.c_str());
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("AddVisual", false, "Unable to create a sub-entity for visual element %s", subEntityName.c_str());
            return;
        }
        auto visualEntityId = createEntityResult.GetValue();
        AddVisualToEntity(visual, visualEntityId);
        AddMaterialForVisual(visual, visualEntityId);
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

        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;
        if (!isPrimitiveShape)
        {
            return;
        }

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

                // TODO - it crashes when empty. Implement support for mesh collider
            }
            break;
        default:
            AZ_Warning("AddCollider", false, "Unsupported collider geometry type, %d", geometry->type);
            break;
        }

        entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, *shapeConfig);
        // TODO - set name as in collision->name
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
