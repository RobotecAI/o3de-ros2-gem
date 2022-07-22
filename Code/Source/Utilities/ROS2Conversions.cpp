/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Conversions.h"

namespace ROS2
{
    AZ::Vector3 ROS2Conversions::FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector)
    { // Same coordinate systems - just translate types
        return AZ::Vector3(ros2vector.x, ros2vector.y, ros2vector.z);
    }

    AZ::Quaternion ROS2Conversions::FromROS2Quaternion(const geometry_msgs::msg::Quaternion & ros2quaternion)
    {   // Same coordinate systems - just translate types
        return AZ::Quaternion(ros2quaternion.x, ros2quaternion.y, ros2quaternion.z, ros2quaternion.w);
    }

    geometry_msgs::msg::Pose ROS2Conversions::ToROS2Pose(const AZ::Transform& aztransform)
    {
        geometry_msgs::msg::Pose ros2pose;
        ros2pose.position = ROS2Conversions::ToROS2Point(aztransform.GetTranslation());
        ros2pose.orientation = ROS2Conversions::ToROS2Quaternion(aztransform.GetRotation());
        return ros2pose;
    }

    geometry_msgs::msg::Point ROS2Conversions::ToROS2Point(const AZ::Vector3& azvector)
    {
        geometry_msgs::msg::Point ros2point;
        ros2point.x = azvector.GetX();
        ros2point.y = azvector.GetY();
        ros2point.z = azvector.GetZ();
        return ros2point;
    }

    geometry_msgs::msg::Vector3 ROS2Conversions::ToROS2Vector3(const AZ::Vector3& azvector)
    {
        geometry_msgs::msg::Vector3 ros2vector;
        ros2vector.x = azvector.GetX();
        ros2vector.y = azvector.GetY();
        ros2vector.z = azvector.GetZ();
        return ros2vector;
    }

    geometry_msgs::msg::Quaternion ROS2Conversions::ToROS2Quaternion(const AZ::Quaternion& azquaternion)
    {
        geometry_msgs::msg::Quaternion ros2Quaternion;
        ros2Quaternion.x = azquaternion.GetX();
        ros2Quaternion.y = azquaternion.GetY();
        ros2Quaternion.z = azquaternion.GetZ();
        ros2Quaternion.w = azquaternion.GetW();
        return ros2Quaternion;
    }
} // namespace ROS2
