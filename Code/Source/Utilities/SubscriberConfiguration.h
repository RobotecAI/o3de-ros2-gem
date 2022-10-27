/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/Communication/QoS.h"
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! A structure for a single ROS2 subscriber configuration.
    struct SubscriberConfiguration
    {
    public:
        AZ_TYPE_INFO(SubscriberConfiguration, "{7C09FBF8-1CFA-4D3E-9D5C-17BFB6683818}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_topic = "default_topic"; //!< Topic to subscribe. Final topic will have a namespace added.

        //! Get topic QoS (Quality of Service) settings.
        //! @see ROS2::QoS.
        rclcpp::QoS GetQoS() const
        {
            return m_qos.GetQoS();
        }

    private:
        QoS m_qos = rclcpp::SensorDataQoS();
    };
} // namespace ROS2
