/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AckermannCommandStruct.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace ROS2
{
    //! Interface class for handling Ackermann kinematics steering commands through EBus notifications.
    //! The interface serves to enable control through AckermannDrive (and AckermannDriveStamped) messages.
    class AckermannNotifications : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        //! Handle Ackermann command
        //! @param ackermannCommand A structure with AckermannDrive message fields
        virtual void AckermannReceived(const AckermannCommandStruct& angular) = 0;
    };

    using AckermannNotificationBus = AZ::EBus<AckermannNotifications>;

    //! This simple handler can be used for prototyping using LUA scripting
    class AckermannNotificationHandler
        : public AckermannNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            AckermannNotificationHandler, "{A6A2011B-8A76-4ACE-B5EF-6DD6F8F1E5DF}", AZ::SystemAllocator, AckermannReceived);

        void AckermannReceived(const AckermannCommandStruct& angular) override;
        static void Reflect(AZ::ReflectContext* context);
    };
} // namespace ROS2