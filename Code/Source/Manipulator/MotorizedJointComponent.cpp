/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ImGui/ImGuiPass.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ROS2/Manipulator/MotorizedJointComponent.h>

namespace ROS2
{
    void MotorizedJointComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_pidPos.InitializePid();
        MotorizedJointRequestBus::Handler::BusConnect(m_entity->GetId());
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        m_imGuiPGain = m_pidPos.GetProportionalGain();
        m_imGuiIGain = m_pidPos.GetIntegralGain();
        m_imGuiDGain = m_pidPos.GetDerivativeGain();
    }

    void MotorizedJointComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        MotorizedJointRequestBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void MotorizedJointComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorizedJointComponent, AZ::Component>()
                ->Version(2)
                ->Field("JointAxis", &MotorizedJointComponent::m_jointDir)
                ->Field("EffortAxis", &MotorizedJointComponent::m_effortAxis)
                ->Field("Limit", &MotorizedJointComponent::m_limits)
                ->Field("Linear", &MotorizedJointComponent::m_linear)
                ->Field("AnimationMode", &MotorizedJointComponent::m_animationMode)
                ->Field("ZeroOffset", &MotorizedJointComponent::m_zeroOffset)
                ->Field("PidPosition", &MotorizedJointComponent::m_pidPos)
                ->Field("OverrideParent", &MotorizedJointComponent::m_measurementReferenceEntity);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MotorizedJointComponent>("MotorizedJointComponent", "MotorizedJointComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "MotorizedJointComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "MotorizedJointComponent")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_jointDir,
                        "Joint Dir.",
                        "Direction of joint in parent's reference frame.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_effortAxis,
                        "Effort Dir.",
                        "Desired direction of force/torque vector that is applied to rigid body.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_limits,
                        "ControllerLimits",
                        "When measurement is outside the limits, ")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_zeroOffset,
                        "Zero Off.",
                        "Allows to change offset of zero to set point")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_linear,
                        "Linear joint",
                        "Applies linear force instead of torque")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_animationMode,
                        "Animation mode",
                        "In animation mode, the transform API is used instead of Rigid Body. "
                        "If this property is set to true the Rigid Body Component should be disabled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_pidPos, "PidPosition", "PidPosition")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_measurementReferenceEntity,
                        "Step Parent",
                        "Allows to override a parent to get correct measurement");
            }
        }
    }
    void MotorizedJointComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const float measurement = ComputeMeasurement(time);

        const float control_position_error = (m_setpoint + m_zeroOffset) - measurement;
        m_error = control_position_error;

        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        float speed_control = m_pidPos.ComputeCommand(control_position_error, deltaTimeNs);
        if (m_linear) {
            if (measurement <= m_limits.first) {
                // allow only positive control
                speed_control = AZStd::max(0.f, speed_control);
            } else if (measurement >= m_limits.second) {
                // allow only negative control
                speed_control = AZStd::min(0.f, speed_control);
            }
        }
        m_currentControl = speed_control;
        SetVelocity(speed_control, deltaTime);
    }

    float MotorizedJointComponent::ComputeMeasurement(AZ::ScriptTimePoint time)
    {
        AZ::Transform transform;
        if (!m_measurementReferenceEntity.IsValid())
        {
            AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        }
        else
        {
            AZ::Transform transformStepParent;
            AZ::TransformBus::EventResult(transformStepParent, m_measurementReferenceEntity, &AZ::TransformBus::Events::GetWorldTM);
            AZ::Transform transformStepChild;
            AZ::TransformBus::EventResult(transformStepChild, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            transform = transformStepParent.GetInverse() * transformStepChild;
        }
        if (m_linear)
        {
            const float last_position = m_currentPosition;
            m_currentPosition = transform.GetTranslation().Dot(this->m_jointDir);
            if (m_lastMeasurementTime > 0)
            {
                double delta_time = time.GetSeconds() - m_lastMeasurementTime;
                m_currentVelocity = (m_currentPosition - last_position) / delta_time;
            }
            m_lastMeasurementTime = time.GetSeconds();
            return m_currentPosition;
        }else{
            const float last_position = m_currentPosition;
            AZ::Vector3 v1 = transform.GetRotation().ConvertToScaledAxisAngle();
            m_currentPosition =v1.Dot(this->m_jointDir);
            if (m_lastMeasurementTime > 0)
            {
                double delta_time = time.GetSeconds() - m_lastMeasurementTime;
                m_currentVelocity = (m_currentPosition - last_position) / delta_time;
            }
            m_lastMeasurementTime = time.GetSeconds();
            return m_currentPosition;
        }
        return 0;
    }

    void MotorizedJointComponent::SetVelocity(float velocity, float deltaTime)
    {
        if (m_animationMode)
        {
            ApplyLinVelAnimation(velocity, deltaTime);
        }
        else
        {
            deltaTime = AZStd::min(deltaTime, 0.1f); // this affects applied force. Need to prevent value that is too large.
            if (m_linear)
            {
                ApplyLinVelRigidBodyImpulse(velocity, deltaTime);
            }else{
                ApplyRotVelRigidBody(velocity,deltaTime);
            }
        }
    }

    void MotorizedJointComponent::ApplyLinVelAnimation(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        transform.SetTranslation(transform.GetTranslation() + velocity * m_jointDir * deltaTime);
        AZ::TransformBus::Event(this->GetEntityId(), &AZ::TransformBus::Events::SetLocalTM, transform);
    }

    void MotorizedJointComponent::ApplyLinVelRigidBodyImpulse(float velocity, float deltaTime)
    {
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        auto force_impulse = transform.TransformVector(m_effortAxis * velocity);
        Physics::RigidBodyRequestBus::Event(
            this->GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, force_impulse * deltaTime);
    }

    void MotorizedJointComponent::ApplyLinVelRigidBody(float velocity, float deltaTime)
    {
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        AZ::Vector3 currentVelocity;
        auto transformed_velocity_increment = transform.TransformVector(m_effortAxis * velocity);
        Physics::RigidBodyRequestBus::EventResult(currentVelocity, this->GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
        AZ::Vector3 new_velocity = currentVelocity + transformed_velocity_increment;
        Physics::RigidBodyRequestBus::Event(this->GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, new_velocity);
    }

    void MotorizedJointComponent::ApplyRotVelRigidBodyImpulse(float velocity /* m/s */, float deltaTime /* seconds */){
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        auto torque_impulse = m_effortAxis * velocity;
        Physics::RigidBodyRequestBus::Event(
                this->GetEntityId(), &Physics::RigidBodyRequests::ApplyAngularImpulse, torque_impulse * deltaTime);
    }
    void MotorizedJointComponent::ApplyRotVelRigidBody(float velocity /* m/s */, float deltaTime /* seconds */){
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        auto torque_impulse = m_effortAxis * velocity;
        Physics::RigidBodyRequestBus::Event(
                this->GetEntityId(), &Physics::RigidBodyRequests::SetAngularVelocity, torque_impulse * deltaTime);
    }

    void MotorizedJointComponent::SetSetpoint(float setpoint)
    {
        if (!m_imGuiOverride) {
            m_setpoint = setpoint;
        }
    }

    float MotorizedJointComponent::GetSetpoint()
    {
        return m_setpoint;
    }

    float MotorizedJointComponent::GetError()
    {
        return m_error;
    }

    float MotorizedJointComponent::GetCurrentMeasurement()
    {
        return m_currentPosition - m_zeroOffset;
    }

    void MotorizedJointComponent::OnImGuiUpdate(){
        if (m_imGuiWin){

            const  AZStd::string menuName{"Motorized Joint "+ GetEntity()->GetName()};
            //const  AZStd::string menuName{" "+ GetEntity()->GetName()};

            ImGui::Begin(menuName.c_str());
            ImGui::Text(" %s | pos: %f | err: %f | cntrl : %f | set : %f |\n",
                    GetEntity()->GetName().c_str(),
                    m_currentPosition,
                    m_error,
                        m_currentControl,
                    m_setpoint);

            ImGui::Checkbox("Override control", &m_imGuiOverride);
            if(m_imGuiOverride){
                float offset =  m_currentPosition - m_zeroOffset;
                ImGui::SliderFloat("Effort     ", &m_currentControl, -1'000, 1'000);
                ImGui::SliderFloat("Measurement", &offset, m_limits.first-m_zeroOffset,m_limits.second-m_zeroOffset);
                ImGui::SliderFloat("Setpoint   ", &m_setpoint, m_limits.first-m_zeroOffset,m_limits.second-m_zeroOffset);
                ImGui::InputFloat2("Limits", &m_limits.first);
                ImGui::InputFloat("Limits", &m_zeroOffset);
            }
            ImGui::BeginGroup();
            ImGui::Text("New PID values:");
            ImGui::InputDouble("P-gain", &m_imGuiPGain);
            ImGui::InputDouble("I-gain", &m_imGuiIGain);
            ImGui::InputDouble("D-gain", &m_imGuiDGain);
            if (ImGui::Button("Update PID")){
                m_pidPos.InitializePid(m_imGuiPGain,m_imGuiIGain,m_imGuiDGain);
            }
            ImGui::EndGroup();
            ImGui::End();
        }
    }

    void MotorizedJointComponent::OnImGuiMainMenuUpdate(){
        const AZStd::string menuName{"Motorized Joint "+ GetEntity()->GetName()};
        if (ImGui::BeginMenu("ROS2"))
        {
            if(ImGui::MenuItem(menuName.c_str())){
                m_imGuiWin = true;
            }
            ImGui::EndMenu();
        }
    }
} // namespace ROS2
