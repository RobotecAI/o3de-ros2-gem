/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/RobotControl/Mecanum/MecanumBus.h"
#include <AzCore/Component/Component.h>

namespace ROS2
{
    //! A component with a simple handler for Mecanum type of control (linear and angular velocities).
    //! Velocities are directly applied to a selected body.
    class MecanumControlComponent
        : public AZ::Component
        , private MecanumNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(MecanumControlComponent, "{8C992A7A-5455-4247-B4C6-17677931F09C}", AZ::Component);
        MecanumControlComponent() = default;

        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        //! Simplest approach: To imitate the steering, current linear and angular velocities of a rigid body are overwritten with inputs
        void TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular) override;

        AZStd::vector<AZ::EntityId> m_leftJoints;
        AZStd::vector<AZ::EntityId> m_rightJoints;

        AZStd::vector<AZ::EntityComponentIdPair> m_leftJointsPairs;
        AZStd::vector<AZ::EntityComponentIdPair> m_rightJointsPairs;

        AZ::EntityId m_baseLink;

        float m_wheelSeparationWidth{ 1.0f };
        float m_wheelSeparationLength{ 1.0f };
        float m_wheelRadius{ 1.0f };

        float m_maxForce{ 2.0f };
        float m_linearXScale{ 1.0f };
        float m_linearYScale{ 1.0f };
        float m_angularScale{ 1.0f };
    };
} // namespace ROS2
