/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "RobotControl/RobotConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
void RobotConfiguration::Reflect(AZ::ReflectContext* context) {
    if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serializeContext->Class<RobotConfiguration>()
                ->Version(1)
                ->Field("Body", &RobotConfiguration::m_body)
                ->Field("Front left wheel", &RobotConfiguration::m_wheelFrontLeft)
                ->Field("Front right wheel", &RobotConfiguration::m_wheelFrontRight)
                ->Field("Rear left wheel", &RobotConfiguration::m_wheelBackLeft)
                ->Field("Rear right wheel", &RobotConfiguration::m_wheelBackRight)
                ;

        if (AZ::EditContext* ec = serializeContext->GetEditContext())
        {
            ec->Class<RobotConfiguration>("Robot control", "Handles robot control")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_body,
                                  "Body", "Robot body")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelFrontLeft,
                                  "Front left wheel", "Robot wheel")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelFrontRight,
                                  "Front right wheel", "Robot wheel")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelBackLeft,
                                  "Back left wheel", "Robot wheel")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelBackRight,
                                  "Back right wheel", "Robot wheel")
                    ;
        }
    }
}
}