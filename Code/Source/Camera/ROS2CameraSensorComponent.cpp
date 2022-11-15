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

#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
        const char* kDepthImageConfig = "Depth Image";
        const char* kColorImageConfig = "Color Image";
        const char* kInfoConfig = "Camera Info";
        const char* kCameraInfoMessageType = "sensor_msgs::msg::CameraInfo";

        AZStd::pair<AZStd::string, TopicConfiguration> MakeTopicConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName)
        {
            TopicConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            return AZStd::make_pair(configName, config);
        }

        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity)
        {
            const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }
        // maping from ATOM to ROS/OpenCV
        AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, sensor_msgs::image_encodings::RGBA8 },
            { AZ::RHI::Format::R16G16B16A16_UNORM, sensor_msgs::image_encodings::RGBA16 },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, sensor_msgs::image_encodings::TYPE_32FC4 }, // Unsuported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, sensor_msgs::image_encodings::MONO8 },
            { AZ::RHI::Format::R16_UNORM, sensor_msgs::image_encodings::MONO16 },
            { AZ::RHI::Format::R32_FLOAT, sensor_msgs::image_encodings::TYPE_32FC1 },

        };
    } // namespace Internal

    ROS2CameraSensorComponent::ROS2CameraSensorComponent()
    {
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_image_color", Internal::kImageMessageType, Internal::kColorImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_image_depth", Internal::kImageMessageType, Internal::kDepthImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_info", Internal::kCameraInfoMessageType, Internal::kInfoConfig));
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

            AZ::EditContext* ec = serialize->GetEditContext();
            if (ec)
            {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Camera component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg,
                        "Vertical field of view",
                        "Camera's vertical (y axis) field of view in degrees.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_width, "Image width", "Image width")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_height, "Image height", "Image height")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_colorCamera, "Color Camera", "Color Camera")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_depthCamera, "Depth Camera", "Depth Camera");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kInfoConfig];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s", cameraInfoFullTopic.data());

        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

        if (m_colorCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kColorImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            m_imagePublisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_cameraSensorColor.emplace(CameraSensorDescription{
                Internal::GetCameraNameFromFrame(GetEntity()), m_VerticalFieldOfViewDeg, m_width, m_height, false });
            m_cameraIntrinsics = m_cameraSensorColor->GetCameraSensorDescription().m_cameraIntrinsics;
        }
        if (m_depthCamera)
        {
            const auto cameraDepthPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kDepthImageConfig];
            AZStd::string depthImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraDepthPublisherConfig.m_topic);
            m_depthPublisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(depthImageFullTopic.data(), cameraDepthPublisherConfig.GetQoS());
            m_cameraSensorDepth.emplace(CameraSensorDescription{
                Internal::GetCameraNameFromFrame(GetEntity()), m_VerticalFieldOfViewDeg, m_width, m_height, true });
            if (!m_colorCamera)
            {
                m_cameraIntrinsics = m_cameraSensorColor->GetCameraSensorDescription().m_cameraIntrinsics;
            }
        }
        const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(component, "Entity has no ROS2FrameComponent");
        m_frameName = component->GetFrameID();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensorColor.reset();
        m_cameraSensorDepth.reset();

        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        if (m_cameraSensorColor || m_depthCamera)
        {
            sensor_msgs::msg::CameraInfo cameraInfo;
            cameraInfo.header.stamp = timestamp;
            cameraInfo.header.frame_id = m_frameName.c_str();
            cameraInfo.width = m_width;
            cameraInfo.height = m_height;
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            AZ_Assert(m_cameraIntrinsics.size() == cameraInfo.k.size(), "should be 9");
            std::copy_n(m_cameraIntrinsics.data(), m_cameraIntrinsics.size(), cameraInfo.k.begin());
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            m_cameraInfoPublisher->publish(cameraInfo);
        }

        if (m_cameraSensorColor)
        {
            m_cameraSensorColor->RequestFrame(
                transform,
                [this, timestamp](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
                {
                    AZ_Assert(m_imagePublisher, "m_depthPublisher should exists");
                    const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                    const auto format = descriptor.m_format;
                    AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                    sensor_msgs::msg::Image message;
                    message.encoding = Internal::FormatMappings.at(format);
                    message.width = descriptor.m_size.m_width;
                    message.height = descriptor.m_size.m_height;
                    message.step = message.width * sensor_msgs::image_encodings::bitDepth(message.encoding) / 8 *
                        sensor_msgs::image_encodings::numChannels(message.encoding);
                    message.data =
                        std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                    message.header.frame_id = m_frameName.c_str();
                    message.header.stamp = timestamp;
                    m_imagePublisher->publish(message);
                });
        }
        if (m_depthCamera)
        {
            m_cameraSensorDepth->RequestFrame(
                transform,
                [this, timestamp](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
                {
                    AZ_Assert(m_depthPublisher, "m_depthPublisher should exists");
                    const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                    const auto format = descriptor.m_format;
                    AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                    sensor_msgs::msg::Image message;
                    message.encoding = Internal::FormatMappings.at(format);
                    message.width = descriptor.m_size.m_width;
                    message.height = descriptor.m_size.m_height;
                    message.step = message.width * sensor_msgs::image_encodings::bitDepth(message.encoding) / 8 *
                        sensor_msgs::image_encodings::numChannels(message.encoding);
                    message.data =
                        std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                    message.header.frame_id = m_frameName.c_str();
                    message.header.stamp = timestamp;
                    m_depthPublisher->publish(message);
                });
        }
    }
} // namespace ROS2
