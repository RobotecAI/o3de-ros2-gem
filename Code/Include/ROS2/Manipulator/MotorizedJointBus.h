/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    //! Interface to communicate with motorized joints.
    //! It allows to apply setpoint and tracking performance of the PID controller.
    class MotorizedJointRequest : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        virtual ~MotorizedJointRequest() = default;

        //! Set current target value for position controller.
        //! @param setpoint value in meters or radians
        virtual void SetSetpoint(float setpoint) = 0;

        //! Get current target value for position controller.
        virtual float GetSetpoint() = 0;

        //! Retrieve current measurement.
        //! @returns measurement value in meters or radians.
        virtual float GetCurrentMeasurement() = 0;

        //! Retrieve current control error (difference between setpoint and measurement)
        //! @returns controller error's value in meters or radians
        virtual float GetError() = 0;
    };

    using MotorizedJointRequestBus = AZ::EBus<MotorizedJointRequest>;
} // namespace ROS2