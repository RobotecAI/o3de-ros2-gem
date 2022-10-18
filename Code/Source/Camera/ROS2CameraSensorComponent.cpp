/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Camera/ROS2CameraSensorComponent.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/Utilities/ROS2Names.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
        const char* kCameraInfoMessageType = "sensor_msgs::msg::CameraInfo";

        AZStd::pair<AZStd::string, PublisherConfiguration> MakePublisherConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType)
        {
            PublisherConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            return AZStd::make_pair(messageType, config);
        }

        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity)
        {
            const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }
    } // namespace Internal


    namespace FrameHelpers
    {
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            // TODO - instead, use EditorFrameComponent to handle Editor-context queries and here only use the "Game" version
            if (!entity)
            {
                AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
                return nullptr;
            }

            auto* interface = entity->FindComponent<AzFramework::TransformComponent>();
            if (interface)
            {
                return interface;
            }
            return entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
        }


        const ROS2FrameComponent* GetFirstROS2FrameAncestor(const AZ::Entity* entity)
        {
            AZ::TransformInterface* entityTransformInterface = GetEntityTransformInterface(entity);
            if (!entityTransformInterface)
            {
                AZ_Error("GetFirstROS2FrameAncestor", false, "Invalid transform interface!");
                return nullptr;
            }

            AZ::EntityId parentEntityId = entityTransformInterface->GetParentId();
            if (!parentEntityId.IsValid())
            { // We have reached the top level, there is no parent entity so there can be no parent ROS2Frame
                return nullptr;
            }

            const AZ::Entity* parentEntity = AzToolsFramework::GetEntityById(parentEntityId);
            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(parentEntity);
            if (component == nullptr)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestor(parentEntity);
            }

            // Found the component!
            return component;
        }
    };
    ROS2CameraSensorComponent::ROS2CameraSensorComponent()
    {
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakePublisherConfigurationPair("camera_image", Internal::kImageMessageType));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakePublisherConfigurationPair("camera_info", Internal::kCameraInfoMessageType));
    }

    void ROS2CameraSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2CameraSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("VerticalFieldOfViewDeg", &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg)
                ->Field("Width", &ROS2CameraSensorComponent::m_width)
                ->Field("Height", &ROS2CameraSensorComponent::m_height)
                ->Field("Namespace Configuration", &ROS2CameraSensorComponent::m_namespaceConfiguration)
                ->Field("Frame Name", &ROS2CameraSensorComponent::m_frameName)
                ->Field("Publish Transform", &ROS2CameraSensorComponent::m_publishTransform);

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
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorComponent::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Namespace Configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_frameName, "Frame Name", "Frame Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_publishTransform, "Publish Transform", "Publish Transform");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
        m_imagePublisher =
            ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kCameraInfoMessageType];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s", cameraInfoFullTopic.data());
        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

        m_cameraSensor.emplace(
            CameraSensorDescription{ Internal::GetCameraNameFromFrame(GetEntity()), m_VerticalFieldOfViewDeg, m_width, m_height });

        m_namespaceConfiguration.PopulateNamespace(false, GetEntity()->GetName());

        if (m_publishTransform)
        {
            m_ros2Transform = AZStd::make_unique<ROS2Transform>(GetParentFrameID(), GetFrameID(), true);
        }
    }

    AZStd::string ROS2CameraSensorComponent::GetParentFrameID() const
    {
        auto parentFrame = FrameHelpers::GetFirstROS2FrameAncestor(GetEntity());
        if (parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }

        AZStd::string parentNamespace("");
        return ROS2Names::GetNamespacedName(m_namespaceConfiguration.GetNamespace(parentNamespace), AZStd::string("camera"));
    }

    void ROS2CameraSensorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        ROS2SensorComponent::OnTick(deltaTime, time);
        m_ros2Transform->Publish(GetFrameTransform());
    }

    const AZ::Transform& ROS2CameraSensorComponent::GetFrameTransform() const
    {
        auto* interface = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        // todo: apply transform modification
        return interface->GetLocalTM();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensor.reset();
        ROS2SensorComponent::Deactivate();
        m_ros2Transform.reset();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        if (!m_cameraSensor)
        {
            return;
        }
        m_cameraSensor->RequestFrame(
            transform,
            [this](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
                AZStd::string frameName = component->GetFrameID();
                sensor_msgs::msg::Image message;
                message.encoding = sensor_msgs::image_encodings::RGBA8;

                message.width = descriptor.m_size.m_width;
                message.height = descriptor.m_size.m_height;
                message.step = message.width * sensor_msgs::image_encodings::bitDepth(message.encoding) / 8 *
                    sensor_msgs::image_encodings::numChannels(message.encoding);
                message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                message.header.frame_id = frameName.c_str();

                m_imagePublisher->publish(message);

                sensor_msgs::msg::CameraInfo cameraInfo;
                cameraInfo.header.frame_id = frameName.c_str();
                cameraInfo.width = descriptor.m_size.m_width;
                cameraInfo.height = descriptor.m_size.m_height;
                cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                cameraInfo.k = m_cameraSensor->GetCameraSensorDescription().m_cameraIntrinsics;
                cameraInfo.p = {cameraInfo.k[0],cameraInfo.k[1],cameraInfo.k[2], 0,
                                 cameraInfo.k[3],cameraInfo.k[4],cameraInfo.k[5], 0,
                                 cameraInfo.k[6],cameraInfo.k[7],cameraInfo.k[8], 0};

                m_cameraInfoPublisher->publish(cameraInfo);
            });
    }
} // namespace ROS2
