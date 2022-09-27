/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace ROS2
{
    //! Interface class for handling Twist commands through EBus notifications.
    //! The interface serves to enable control through Twist (and TwistStamped) messages.
    class JoyNotifications : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual void JoyReceived(
            float a0,
            float a1,
            float a2,
            float a3,
            float a4,
            float a5,
            bool b0,
            bool b1,
            bool b2,
            bool b3,
            bool b4,
            bool b5,
            bool b6,
            bool b7) = 0;
    };

    using JoyNotificationBus = AZ::EBus<JoyNotifications>;

    //! This simple handler can be used for prototyping using LUA scripting
    class JoyNotificationHandler
        : public JoyNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(JoyNotificationHandler, "{5BE94DA6-BCEC-40D9-A0E4-7E4742E4A36E}", AZ::SystemAllocator, JoyReceived);

        void JoyReceived(
            float a0,
            float a1,
            float a2,
            float a3,
            float a4,
            float a5,
            bool b0,
            bool b1,
            bool b2,
            bool b3,
            bool b4,
            bool b5,
            bool b6,
            bool b7) override;
        static void Reflect(AZ::ReflectContext* context);
    };
} // namespace ROS2