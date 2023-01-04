/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "MecanumControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <HingeJointComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>

namespace ROS2
{
    void MecanumControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MecanumControlComponent, AZ::Component>()
                ->Field("Left Joints", &MecanumControlComponent::m_leftJoints)
                ->Field("Right Joints", &MecanumControlComponent::m_rightJoints)
                ->Field("Base link", &MecanumControlComponent::m_baseLink)
                ->Field("Wheel Separation Width", &MecanumControlComponent::m_wheelSeparationWidth)
                ->Field("Wheel Separation Length", &MecanumControlComponent::m_wheelSeparationLength)
                ->Field("Wheel Radius", &MecanumControlComponent::m_wheelRadius)
                ->Field("Max Force", &MecanumControlComponent::m_maxForce)
                ->Field("X Linear Velocity Scale", &MecanumControlComponent::m_linearXScale)
                ->Field("Y Linear Velocity Scale", &MecanumControlComponent::m_linearYScale)
                ->Field("Angular Velocity Scale", &MecanumControlComponent::m_angularScale)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MecanumControlComponent>("Mecanum Control", "Simple control through RigidBody")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &MecanumControlComponent::m_leftJoints, "Left joints", "")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &MecanumControlComponent::m_rightJoints, "Right joints", "")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &MecanumControlComponent::m_baseLink, "Base link", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_wheelSeparationWidth, "Wheel Separation Width", "")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_wheelSeparationLength, "Wheel Separation Length", "")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_wheelRadius, "Wheel Radius", "")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_maxForce, "Max Force", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_linearXScale, "X Linear Velocity Scale", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_linearYScale, "Y Linear Velocity Scale", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MecanumControlComponent::m_angularScale, "Angular Velocity Scale", "");
            }
        }
    }

    void MecanumControlComponent::Activate()
    {
        m_rightJointsPairs.clear();
        m_leftJointsPairs.clear();

        MecanumNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void MecanumControlComponent::Deactivate()
    {
        MecanumNotificationBus::Handler::BusDisconnect();
    }

    void setSpeedAndForce(const AZ::EntityComponentIdPair& id, float force, float speed)
    {
        PhysX::JointInterfaceRequestBus::Event(
            id,
            [&](PhysX::JointRequests* joint)
            {
                joint->SetMaximumForce(force);
                joint->SetVelocity(speed);
            });
    }

    void MecanumControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2RobotControl"));
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void MecanumControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        if (m_rightJointsPairs.empty() || m_leftJointsPairs.empty())
        {
            m_rightJointsPairs.clear();
            m_leftJointsPairs.clear();
            auto getComponentIdPair = [&](AZ::EntityId entityId)
            {
                AZ::Entity* entity{ nullptr };
                PhysX::HingeJointComponent* component{ nullptr };
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                if (entity)
                {
                    // AZ_Printf("SkidSteeringDemo", "Found entity %s", entity->GetName().c_str());
                    component = entity->FindComponent<PhysX::HingeJointComponent>();
                }
                if (component)
                {
                    // AZ_Printf("SkidSteeringDemo", "Found component %llu", component->GetId());
                    return AZ::EntityComponentIdPair(entityId, component->GetId());
                }
                // AZ_Printf("SkidSteeringDemo::Activate",  "Nothing found : %s", entityId.ToString().c_str());
                return AZ::EntityComponentIdPair();
            };
            for (const auto& entityId : m_rightJoints)
            {
                m_rightJointsPairs.emplace_back(getComponentIdPair(entityId));
            }
            for (const auto& entityId : m_leftJoints)
            {
                m_leftJointsPairs.emplace_back(getComponentIdPair(entityId));
            }
        }

        auto m_linearVelX = linear.GetX() * m_linearXScale;
        auto m_linearVelY = linear.GetY() * m_linearYScale;
        auto m_rotVel = angular.GetZ() * m_angularScale;

        auto wheelGeometry = (m_wheelSeparationWidth + m_wheelSeparationLength) / 2.0f;

        setSpeedAndForce(m_leftJointsPairs.front(), m_maxForce, (m_linearVelX + m_linearVelY + m_rotVel * wheelGeometry) / m_wheelRadius);
        setSpeedAndForce(m_leftJointsPairs.back(), m_maxForce, (m_linearVelX - m_linearVelY + m_rotVel * wheelGeometry) / m_wheelRadius);

        setSpeedAndForce(m_rightJointsPairs.front(), m_maxForce, (m_linearVelX - m_linearVelY - m_rotVel * wheelGeometry) / m_wheelRadius);
        setSpeedAndForce(m_rightJointsPairs.back(), m_maxForce, (m_linearVelX + m_linearVelY - m_rotVel * wheelGeometry) / m_wheelRadius);
    }
} // namespace ROS2
