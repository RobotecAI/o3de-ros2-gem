/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "MotorizedJoint.h"

#include "AzFramework/Physics/Components/SimulatedBodyComponentBus.h"
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void MotorizedJoint::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_pid_pos_conf.InitializePid();
        if (m_debug_draw_entity.IsValid())
        {
            AZ::TransformBus::EventResult(
                m_debug_draw_entity_initial_transfomr, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        }
    }

    void MotorizedJoint::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void MotorizedJoint::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorizedJoint, AZ::Component>()
                ->Version(1)
                ->Field("JointAxis", &MotorizedJoint::m_joint_dir)
                ->Field("Limit", &MotorizedJoint::m_limits)
                ->Field("Linear", &MotorizedJoint::m_linear)
                ->Field("AnimationMode", &MotorizedJoint::m_animation_mode)
                ->Field("ZeroOffset", &MotorizedJoint::m_zero_offset)
                ->Field("PidPosition", &MotorizedJoint::m_pid_pos_conf)
                ->Field("DebugDrawEntity", &MotorizedJoint::m_debug_draw_entity)
                ->Field("TestSinActive", &MotorizedJoint::m_test_sinusoidal)
                ->Field("TestSinAmplitude", &MotorizedJoint::m_sin_amplitude)
                ->Field("TestSinFreq", &MotorizedJoint::m_sin_freq)
                ->Field("DebugPrint", &MotorizedJoint::m_debug_print);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MotorizedJoint>("MotorizedJoint", "MotorizedJoint")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "MotorizedJoint")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "MotorizedJoint")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_joint_dir, "Dir.", "Direction of joint.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_limits,
                        "ControllerLimits",
                        "When measurment is outside the limits, ")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_debug_draw_entity,
                        "Setpoint",
                        "Allows to apply debug setpoint visualizer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_zero_offset,
                        "Zero Off.",
                        "Allows to change offset of zero to set point")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_linear, "Apl. Linear", "Applies linear force instead of torque")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_animation_mode,
                        "Animation mode",
                        "In animation mode, the transform API is used instead of Rigid Body. Note that using this "
                        "the Rigid Body Component should be disabled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_pid_pos_conf, "PidPosition", "PidPosition")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_test_sinusoidal,
                        "SinusoidalTest",
                        "Allows to apply sinusoidal test signal")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_sin_amplitude, "Amplitude", "Amplitude of sinusoidal test signal")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_sin_freq, "Frequency", "TestSinFreq")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_debug_print, "Debug", "Print debug to console");
            }
        }
    }
    void MotorizedJoint::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const float measurment = getMeasurment(time);
        float setpoint = 0;
        if (m_test_sinusoidal)
        {
            setpoint = m_sin_amplitude * AZ::Sin(m_sin_freq * time.GetSeconds());
        }
        const float control_position_error = (setpoint + m_zero_offset) - measurment;

        if (m_debug_draw_entity.IsValid())
        {
            if (m_linear)
            {
                AZ::Transform transform = AZ::Transform::Identity();
                transform.SetTranslation(m_joint_dir * (setpoint + m_zero_offset));
                AZ::TransformBus::Event(
                    m_debug_draw_entity, &AZ::TransformBus::Events::SetLocalTM, transform * m_debug_draw_entity_initial_transfomr);
            }
            else
            {
                AZ_Assert(false, "Not implemented");
            }
        }

        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        float speed_control = m_pid_pos_conf.ComputeCommand(control_position_error, deltaTimeNs);

        if (measurment <= m_limits.first)
        {
            // allow only positive control
            speed_control = AZStd::max(0.f, speed_control);
        }
        else if (measurment >= m_limits.second)
        {
            // allow only negative control
            speed_control = AZStd::min(0.f, speed_control);
        }

        if (m_debug_print)
        {
            AZ_Printf(
                "MotorizedJoint",
                " %s | pos: %f | err: %f | cntrl : %f |",
                GetEntity()->GetName().c_str(),
                measurment,
                control_position_error,
                speed_control);
        }
        setVelocity(speed_control, deltaTime);
        m_last_time = time.GetSeconds();
    }
    float MotorizedJoint::getMeasurment(AZ::ScriptTimePoint time)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        if (m_linear)
        {
            const float last_position = m_current_position;
            m_current_position = transform.GetTranslation().Dot(this->m_joint_dir);
            if (m_last_measurment_time > 0)
            {
                double delta_time = time.GetSeconds() - m_last_measurment_time;
                m_current_velocity = (m_current_position - last_position) / delta_time;
            }
            m_last_measurment_time = time.GetSeconds();
            return m_current_position;
        }
        AZ_Assert(false, "it is not implemented");
        return 0;
    }

    void MotorizedJoint::setVelocity(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);

        if (m_animation_mode)
        {
            transform.SetTranslation(transform.GetTranslation() + velocity * m_joint_dir * deltaTime);
            AZ::TransformBus::Event(this->GetEntityId(), &AZ::TransformBus::Events::SetLocalTM, transform);
        }
        else
        {
            if (m_linear)
            {
                constexpr bool applyForceMode{ false };
                constexpr bool applyVelocityMode{ true };
                if (applyForceMode)
                {
                    auto force_impulse = transform.TransformVector(m_joint_dir * velocity);
                    Physics::RigidBodyRequestBus::Event(
                        this->GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, force_impulse * deltaTime);
                }
                if (applyVelocityMode)
                {
                    AZ::Vector3 current_velocicty;
                    auto transformed_velocity_increment = transform.TransformVector(m_joint_dir * velocity);
                    Physics::RigidBodyRequestBus::EventResult(
                        current_velocicty, this->GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
                    AZ::Vector3 new_velocity = current_velocicty + transformed_velocity_increment;
                    Physics::RigidBodyRequestBus::Event(this->GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, new_velocity);
                }
            }
        }
    }

} // namespace ROS2
