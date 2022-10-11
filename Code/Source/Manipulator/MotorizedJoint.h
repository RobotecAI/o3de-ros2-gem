/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleDynamics/DriveModels/PidConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/AzFrameworkModule.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorPrismaticJointComponent.h>
#include <Source/EditorRigidBodyComponent.h>

namespace ROS2
{

    class MotorizedJoint
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(MotorizedJoint, "{AE9207DB-5B7E-4F70-A7DD-C4EAD8DD9403}", AZ::Component);

        // AZ::Component interface implementation.
        MotorizedJoint() = default;
        ~MotorizedJoint() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        float getMeasurment(AZ::ScriptTimePoint time);
        void setVelocity(float velocity, float deltaTime);

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        AZ::Vector3 m_joint_dir{ 0.f, 0.f, 1.f };
        AZStd::pair<float, float> m_limits{ -0.5f, 0.5f };
        VehicleDynamics::PidConfiguration m_pid_pos;

        float m_zero_offset{ 0.f };

        bool m_linear{ true };
        bool m_animation_mode{ true };
        bool m_test_sinusoidal{ true };
        bool m_debug_print{ false };
        float m_sin_amplitude{ 0.25 };
        float m_sin_freq{ 0.1 };

        float m_current_position{ 0 };
        float m_current_velocity{ 0 };
        double m_last_measurment_time;
        double m_last_time;

        VehicleDynamics::PidConfiguration m_pid_pos_conf;
        AZ::EntityId m_debug_draw_entity;
        AZ::Transform m_debug_draw_entity_initial_transfomr;

        //        float m_maxspeed;
        //        float m_maxacceleration;
        //        float m_maxforce;
        //        float m_gain_p_speed{100.f};
        //        float m_gain_i_speed{0};
        //        float m_gain_d_speed{0};
        //        float m_gain_p_position{10.f};
        //        float m_gain_i_position{0.f};
        //        float m_gain_d_position{0.f};
    };
} // namespace ROS2
