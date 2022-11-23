/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Camera/ROS2CameraSensorComponent.h"
#include "ROS2/Communication/TopicConfiguration.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/Utilities/ROS2Names.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    namespace Internal
    {

        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity)
        {
            const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }

    } // namespace Internal

    ROS2CameraSensorComponent::ROS2CameraSensorComponent(
        const SensorConfiguration& sensorConfiguration,
        float verticalFieldOfViewDeg,
        int width,
        int height,
        bool colorCamera,
        bool depthCamera)
    {
        m_sensorConfiguration = sensorConfiguration;
        m_VerticalFieldOfViewDeg = verticalFieldOfViewDeg;
        m_width = width;
        m_height = height;
        m_colorCamera = colorCamera;
        m_depthCamera = depthCamera;
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                ->Version(3)
                ->Field("VerticalFieldOfViewDeg", &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg)
                ->Field("Width", &ROS2CameraSensorComponent::m_width)
                ->Field("Height", &ROS2CameraSensorComponent::m_height)
                ->Field("Depth", &ROS2CameraSensorComponent::m_depthCamera)
                ->Field("Color", &ROS2CameraSensorComponent::m_colorCamera);
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraSensorConstants::kInfoConfig];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s", cameraInfoFullTopic.data());

        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

        const CameraSensorDescription description{
            Internal::GetCameraNameFromFrame(GetEntity()), m_VerticalFieldOfViewDeg, m_width, m_height
        };
        if (m_colorCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraSensorConstants::kColorImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_cameraSensorsWithPublihsers.emplace_back(createPair<CameraColorSensor>(publisher, description));
        }
        if (m_depthCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraSensorConstants::kDepthImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_cameraSensorsWithPublihsers.emplace_back(createPair<CameraDepthSensor>(publisher, description));
        }
        const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(component, "Entity has no ROS2FrameComponent");
        m_frameName = component->GetFrameID();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensorsWithPublihsers.clear();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        const AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        std_msgs::msg::Header ros_header;
        if (!m_cameraSensorsWithPublihsers.empty())
        {
            const auto& camera_descritpion = m_cameraSensorsWithPublihsers.front().second->GetCameraSensorDescription();
            const auto& cameraIntrinsics = camera_descritpion.m_cameraIntrinsics;
            sensor_msgs::msg::CameraInfo cameraInfo;
            ros_header.stamp = timestamp;
            ros_header.frame_id = m_frameName.c_str();
            cameraInfo.header = ros_header;
            cameraInfo.width = m_width;
            cameraInfo.height = m_height;
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            AZ_Assert(cameraIntrinsics.size() == cameraInfo.k.size(), "should be 9");
            std::copy_n(cameraIntrinsics.data(), cameraIntrinsics.size(), cameraInfo.k.begin());
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            m_cameraInfoPublisher->publish(cameraInfo);
        }
        for (auto& [publisher, sensor] : m_cameraSensorsWithPublihsers)
        {
            sensor->publishMassage(publisher, transform, ros_header);
        }
    }
} // namespace ROS2
