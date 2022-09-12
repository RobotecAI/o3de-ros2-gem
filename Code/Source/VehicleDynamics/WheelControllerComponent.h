/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>

namespace VehicleDynamics
{
    //! A component responsible for control (steering, forward motion) of a single wheel
    class WheelControllerComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(WheelControllerComponent, "{AC594E08-DA5C-4B8D-9388-84D0840C177A}", AZ::Component);
        WheelControllerComponent() = default;
        ~WheelControllerComponent() = default;

        // AZ::Component interface implementation.
        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_steeringEntity; //! Rigid body to apply torque to. TODO - parent, this entity or custom.
        AZ::EntityId m_visualEntity; //! Primary visual mesh or primitive for the wheel. It will be rotated according to the motion.
    };
} // namespace VehicleDynamics
