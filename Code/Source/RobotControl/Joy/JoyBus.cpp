/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "RobotControl/Joy/JoyBus.h"

namespace ROS2
{
    void JoyNotificationHandler::JoyReceived(
        float a0, float a1, float a2, float a3, float a4, float a5, bool b0, bool b1, bool b2, bool b3, bool b4, bool b5, bool b6, bool b7)
    {
        Call(FN_JoyReceived, a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5, b6, b7);
    }

    void JoyNotificationHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<JoyNotificationBus>("JoyNotificationBus")
                ->Handler<JoyNotificationHandler>()
                ->Event("JoyReceived", &JoyNotificationBus::Events::JoyReceived);
        }
    }
} // namespace ROS2
