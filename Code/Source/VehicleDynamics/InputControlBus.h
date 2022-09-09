/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace VehicleDynamics
{
    //! Inputs (speed, steering, acceleration etc.) for vehicle dynamics system
    //! Inputs are valid for a short time (configurable) and need to be repeated if continuous movement is needed.
    //! (Use cruise system to set cruise speed).
    class InputControlRequests
    {
    public:
        AZ_RTTI(InputControlRequests, "{AB0F1D2A-3A73-41B6-B882-62316DE32010}");
        virtual ~InputControlRequests() = default;

        //! Set target for the vehicle linear speed. It should be realized over time according to drive model.
        //! @param speedMps is a linear speed in meters per second with the plus sign in the forward direction.
        virtual void SetTargetLinearSpeed(float speedMps) = 0;

        //! Accelerate without target speed.
        //! @param acceleration is relative to limits of possible acceleration.
        //! 1.0 - accelerate as much as possible, -1.0 - brake as much as possible.
        virtual void SetTargetAcceleration(float acceleration) = 0;

        //! Steer in a direction given in relative coordinate system (current direction is 0).
        //! @param steering is angle in radians, positive to the right and negative to the left.
        //! Note that the actual angle applied is subject to limits and implementation (e.g. smoothing).
        virtual void SetTargetSteering(float steering) = 0;
    };

    class InputBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using InputControlRequestBus = AZ::EBus<InputControlRequests, InputBusTraits>;
    using InputControlInterface = AZ::Interface<InputControlRequests>;
} // namespace VehicleDynamics