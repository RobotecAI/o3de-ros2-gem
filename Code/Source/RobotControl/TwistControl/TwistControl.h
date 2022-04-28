/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "RobotControl/RobotControl.h"
#include "RobotControl/TwistControl/TwistBus.h"

#include "geometry_msgs/msg/twist.hpp"

namespace ROS2
{
    class TwistControl : public RobotControl<geometry_msgs::msg::Twist>
    {
    private:
        void BroadcastBus(const geometry_msgs::msg::Twist& message) override;
        void ApplyControl(const geometry_msgs::msg::Twist& message) override;
    };
}  // namespace ROS2