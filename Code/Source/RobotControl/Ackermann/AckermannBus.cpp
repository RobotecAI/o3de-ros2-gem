/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "RobotControl/Ackermann/AckermannBus.h"

namespace ROS2
{
    void AckermannNotificationHandler::(const AckermannCommandStruct& acs)
    {
        Call(FN_AckermannReceived, acs);
    }

    void AckermannNotificationHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<AckermannNotificationBus>("AckermannNotificationBus")
                ->Handler<AckermannNotificationHandler>()
                ->Event("AckermannReceived", &AckermannNotificationBus::Events::AckermannReceived);
        }
    }
} // namespace ROS2
