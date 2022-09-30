/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <chrono>

class Entity;

namespace ROS2
{

    namespace CameraFormatConversions
    {
        //
        // clang-format off
        const  AZStd::unordered_map<AZ::RHI::Format, AZStd::string> kRHIFormatToString{{
            {AZ::RHI::Format::Unknown, "AZ::RHI::Format::Unknown"},
            {AZ::RHI::Format::R32G32B32A32_FLOAT, "R32G32B32A32_FLOAT"},
            {AZ::RHI::Format::R32G32B32A32_UINT, "R32G32B32A32_UINT"},
            {AZ::RHI::Format::R32G32B32A32_SINT, "R32G32B32A32_SINT"},
            {AZ::RHI::Format::R32G32B32_FLOAT, "R32G32B32_FLOAT"},
            {AZ::RHI::Format::R32G32B32_UINT, "R32G32B32_UINT"},
            {AZ::RHI::Format::R32G32B32_SINT, "R32G32B32_SINT"},
            {AZ::RHI::Format::R16G16B16A16_FLOAT, "R16G16B16A16_FLOAT"},
            {AZ::RHI::Format::R16G16B16A16_UNORM, "R16G16B16A16_UNORM"},
            {AZ::RHI::Format::R16G16B16A16_UINT, "R16G16B16A16_UINT"},
            {AZ::RHI::Format::R16G16B16A16_SNORM, "R16G16B16A16_SNORM"},
            {AZ::RHI::Format::R16G16B16A16_SINT, "R16G16B16A16_SINT"},
            {AZ::RHI::Format::R32G32_FLOAT, "R32G32_FLOAT"},
            {AZ::RHI::Format::R32G32_UINT, "R32G32_UINT"},
            {AZ::RHI::Format::R32G32_SINT, "R32G32_SINT"},
            {AZ::RHI::Format::D32_FLOAT_S8X24_UINT, "D32_FLOAT_S8X24_UINT"},
            {AZ::RHI::Format::R10G10B10A2_UNORM, "R10G10B10A2_UNORM"},
            {AZ::RHI::Format::R10G10B10A2_UINT, "R10G10B10A2_UINT"},
            {AZ::RHI::Format::R11G11B10_FLOAT, "R11G11B10_FLOAT"},
            {AZ::RHI::Format::R8G8B8A8_UNORM, "R8G8B8A8_UNORM"},
            {AZ::RHI::Format::R8G8B8A8_UNORM_SRGB, "R8G8B8A8_UNORM_SRGB"},
            {AZ::RHI::Format::R8G8B8A8_UINT, "R8G8B8A8_UINT"},
            {AZ::RHI::Format::R8G8B8A8_SNORM, "R8G8B8A8_SNORM"},
            {AZ::RHI::Format::R8G8B8A8_SINT, "R8G8B8A8_SINT"},
            {AZ::RHI::Format::R16G16_FLOAT, "R16G16_FLOAT"},
            {AZ::RHI::Format::R16G16_UNORM, "R16G16_UNORM"},
            {AZ::RHI::Format::R16G16_UINT, "R16G16_UINT"},
            {AZ::RHI::Format::R16G16_SNORM, "R16G16_SNORM"},
            {AZ::RHI::Format::R16G16_SINT, "R16G16_SINT"},
            {AZ::RHI::Format::D32_FLOAT, "D32_FLOAT"},
            {AZ::RHI::Format::R32_FLOAT, "R32_FLOAT"},
            {AZ::RHI::Format::R32_UINT, "R32_UINT"},
            {AZ::RHI::Format::R32_SINT, "R32_SINT"},
            {AZ::RHI::Format::D24_UNORM_S8_UINT, "D24_UNORM_S8_UINT"},
            {AZ::RHI::Format::R8G8_UNORM, "R8G8_UNORM"},
            {AZ::RHI::Format::R8G8_UNORM_SRGB, "R8G8_UNORM_SRGB"},
            {AZ::RHI::Format::R8G8_UINT, "R8G8_UINT"},
            {AZ::RHI::Format::R8G8_SNORM, "R8G8_SNORM"},
            {AZ::RHI::Format::R8G8_SINT, "R8G8_SINT"},
            {AZ::RHI::Format::R16_FLOAT, "R16_FLOAT"},
            {AZ::RHI::Format::D16_UNORM, "D16_UNORM"},
            {AZ::RHI::Format::R16_UNORM, "R16_UNORM"},
            {AZ::RHI::Format::R16_UINT, "R16_UINT"},
            {AZ::RHI::Format::R16_SNORM, "R16_SNORM"},
            {AZ::RHI::Format::R16_SINT, "R16_SINT"},
            {AZ::RHI::Format::R8_UNORM, "R8_UNORM"},
            {AZ::RHI::Format::R8_UNORM_SRGB, "R8_UNORM_SRGB"},
            {AZ::RHI::Format::R8_UINT, "R8_UINT"},
            {AZ::RHI::Format::R8_SNORM, "R8_SNORM"},
            {AZ::RHI::Format::R8_SINT, "R8_SINT"},
            {AZ::RHI::Format::A8_UNORM, "A8_UNORM"},
            {AZ::RHI::Format::R1_UNORM, "R1_UNORM"},
            {AZ::RHI::Format::R9G9B9E5_SHAREDEXP, "R9G9B9E5_SHAREDEXP"},
            {AZ::RHI::Format::R8G8_B8G8_UNORM, "R8G8_B8G8_UNORM"},
            {AZ::RHI::Format::G8R8_G8B8_UNORM, "G8R8_G8B8_UNORM"},
            {AZ::RHI::Format::BC1_UNORM, "BC1_UNORM"},
            {AZ::RHI::Format::BC1_UNORM_SRGB, "BC1_UNORM_SRGB"},
            {AZ::RHI::Format::BC2_UNORM, "BC2_UNORM"},
            {AZ::RHI::Format::BC2_UNORM_SRGB, "BC2_UNORM_SRGB"},
            {AZ::RHI::Format::BC3_UNORM, "BC3_UNORM"},
            {AZ::RHI::Format::BC3_UNORM_SRGB, "BC3_UNORM_SRGB"},
            {AZ::RHI::Format::BC4_UNORM, "BC4_UNORM"},
            {AZ::RHI::Format::BC4_SNORM, "BC4_SNORM"},
            {AZ::RHI::Format::BC5_UNORM, "BC5_UNORM"},
            {AZ::RHI::Format::BC5_SNORM, "BC5_SNORM"},
            {AZ::RHI::Format::B5G6R5_UNORM, "B5G6R5_UNORM"},
            {AZ::RHI::Format::B5G5R5A1_UNORM, "B5G5R5A1_UNORM"},
            {AZ::RHI::Format::A1B5G5R5_UNORM, "A1B5G5R5_UNORM"},
            {AZ::RHI::Format::B8G8R8A8_UNORM, "B8G8R8A8_UNORM"},
            {AZ::RHI::Format::B8G8R8X8_UNORM, "B8G8R8X8_UNORM"},
            {AZ::RHI::Format::R10G10B10_XR_BIAS_A2_UNORM, "R10G10B10_XR_BIAS_A2_UNORM"},
            {AZ::RHI::Format::B8G8R8A8_UNORM_SRGB, "B8G8R8A8_UNORM_SRGB"},
            {AZ::RHI::Format::B8G8R8X8_UNORM_SRGB, "B8G8R8X8_UNORM_SRGB"},
            {AZ::RHI::Format::BC6H_UF16, "BC6H_UF16"},
            {AZ::RHI::Format::BC6H_SF16, "BC6H_SF16"},
            {AZ::RHI::Format::BC7_UNORM, "BC7_UNORM"},
            {AZ::RHI::Format::BC7_UNORM_SRGB, "BC7_UNORM_SRGB"},
            {AZ::RHI::Format::AYUV, "AYUV"},
            {AZ::RHI::Format::Y410, "Y410"},
            {AZ::RHI::Format::Y416, "Y416"},
            {AZ::RHI::Format::NV12, "NV12"},
            {AZ::RHI::Format::P010, "P010"},
            {AZ::RHI::Format::P016, "P016"},
            {AZ::RHI::Format::YUY2, "YUY2"},
            {AZ::RHI::Format::Y210, "Y210"},
            {AZ::RHI::Format::Y216, "Y216"},
            {AZ::RHI::Format::NV11, "NV11"},
            {AZ::RHI::Format::AI44, "AI44"},
            {AZ::RHI::Format::IA44, "IA44"},
            {AZ::RHI::Format::P8, "P8"},
            {AZ::RHI::Format::A8P8, "A8P8"},
            {AZ::RHI::Format::B4G4R4A4_UNORM, "B4G4R4A4_UNORM"},
            {AZ::RHI::Format::R4G4B4A4_UNORM, "R4G4B4A4_UNORM"},
            {AZ::RHI::Format::R10G10B10_7E3_A2_FLOAT, "R10G10B10_7E3_A2_FLOAT"},
            {AZ::RHI::Format::R10G10B10_6E4_A2_FLOAT, "R10G10B10_6E4_A2_FLOAT"},
            {AZ::RHI::Format::D16_UNORM_S8_UINT, "D16_UNORM_S8_UINT"},
            {AZ::RHI::Format::X16_TYPELESS_G8_UINT, "X16_TYPELESS_G8_UINT"},
            {AZ::RHI::Format::P208, "P208"},
            {AZ::RHI::Format::V208, "V208"},
            {AZ::RHI::Format::V408, "V408"},
            {AZ::RHI::Format::EAC_R11_UNORM, "EAC_R11_UNORM"},
            {AZ::RHI::Format::EAC_R11_SNORM, "EAC_R11_SNORM"},
            {AZ::RHI::Format::EAC_RG11_UNORM, "EAC_RG11_UNORM"},
            {AZ::RHI::Format::EAC_RG11_SNORM, "EAC_RG11_SNORM"},
            {AZ::RHI::Format::ETC2_UNORM, "ETC2_UNORM"},
            {AZ::RHI::Format::ETC2_UNORM_SRGB, "ETC2_UNORM_SRGB"},
            {AZ::RHI::Format::ETC2A_UNORM, "ETC2A_UNORM"},
            {AZ::RHI::Format::ETC2A_UNORM_SRGB, "ETC2A_UNORM_SRGB"},
            {AZ::RHI::Format::ETC2A1_UNORM, "ETC2A1_UNORM"},
            {AZ::RHI::Format::ETC2A1_UNORM_SRGB, "ETC2A1_UNORM_SRGB"},
            {AZ::RHI::Format::PVRTC2_UNORM, "PVRTC2_UNORM"},
            {AZ::RHI::Format::PVRTC2_UNORM_SRGB, "PVRTC2_UNORM_SRGB"},
            {AZ::RHI::Format::PVRTC4_UNORM, "PVRTC4_UNORM"},
            {AZ::RHI::Format::PVRTC4_UNORM_SRGB, "PVRTC4_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_4x4_UNORM, "ASTC_4x4_UNORM"},
            {AZ::RHI::Format::ASTC_4x4_UNORM_SRGB, "ASTC_4x4_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_5x4_UNORM, "ASTC_5x4_UNORM"},
            {AZ::RHI::Format::ASTC_5x4_UNORM_SRGB, "ASTC_5x4_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_5x5_UNORM, "ASTC_5x5_UNORM"},
            {AZ::RHI::Format::ASTC_5x5_UNORM_SRGB, "ASTC_5x5_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_6x5_UNORM, "ASTC_6x5_UNORM"},
            {AZ::RHI::Format::ASTC_6x5_UNORM_SRGB, "ASTC_6x5_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_6x6_UNORM, "ASTC_6x6_UNORM"},
            {AZ::RHI::Format::ASTC_6x6_UNORM_SRGB, "ASTC_6x6_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_8x5_UNORM, "ASTC_8x5_UNORM"},
            {AZ::RHI::Format::ASTC_8x5_UNORM_SRGB, "ASTC_8x5_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_8x6_UNORM, "ASTC_8x6_UNORM"},
            {AZ::RHI::Format::ASTC_8x6_UNORM_SRGB, "ASTC_8x6_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_8x8_UNORM, "ASTC_8x8_UNORM"},
            {AZ::RHI::Format::ASTC_8x8_UNORM_SRGB, "ASTC_8x8_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_10x5_UNORM, "ASTC_10x5_UNORM"},
            {AZ::RHI::Format::ASTC_10x5_UNORM_SRGB, "ASTC_10x5_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_10x6_UNORM, "ASTC_10x6_UNORM"},
            {AZ::RHI::Format::ASTC_10x6_UNORM_SRGB, "ASTC_10x6_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_10x8_UNORM, "ASTC_10x8_UNORM"},
            {AZ::RHI::Format::ASTC_10x8_UNORM_SRGB, "ASTC_10x8_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_10x10_UNORM, "ASTC_10x10_UNORM"},
            {AZ::RHI::Format::ASTC_10x10_UNORM_SRGB, "ASTC_10x10_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_12x10_UNORM, "ASTC_12x10_UNORM"},
            {AZ::RHI::Format::ASTC_12x10_UNORM_SRGB, "ASTC_12x10_UNORM_SRGB"},
            {AZ::RHI::Format::ASTC_12x12_UNORM, "ASTC_12x12_UNORM"},
            {AZ::RHI::Format::ASTC_12x12_UNORM_SRGB, "ASTC_12x12_UNORM_SRGB"},
            {AZ::RHI::Format::A8B8G8R8_UNORM, "A8B8G8R8_UNORM"},
            {AZ::RHI::Format::A8B8G8R8_UNORM_SRGB, "A8B8G8R8_UNORM_SRGB"},
            {AZ::RHI::Format::A8B8G8R8_SNORM, "A8B8G8R8_SNORM"},
            {AZ::RHI::Format::R5G6B5_UNORM, "R5G6B5_UNORM"},
            {AZ::RHI::Format::B8G8R8A8_SNORM, "B8G8R8A8_SNORM"},

        }};

        // clang-format on
    } // namespace CameraFormatConversions

    //! Structure containing all information required to create the camera sensor
    struct CameraSensorDescription
    {
        //! Constructor to create the description
        //! @param cameraName - name of the camera; used to differentiate cameras in a multi-camera setup
        //! @param verticalFov - vertical field of view of camera sensor
        //! @param width - camera image width in pixels
        //! @param height - camera image height in pixels
        CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height);

        const float m_verticalFieldOfViewDeg; //!< camera vertical field of view
        const int m_width; //!< camera image width in pixels
        const int m_height; //!< camera image height in pixels
        const AZStd::string m_cameraName; //!< camera name to differentiate cameras in a multi-camera setup

        const float m_aspectRatio; //!< camera image aspect ratio; equal to (width / height)
        const AZ::Matrix4x4 m_viewToClipMatrix; //!< camera view to clip space transform matrix; derived from other parameters
        const std::array<double, 9> m_cameraIntrinsics; //!< camera intrinsics; derived from other parameters

    private:
        AZ::Matrix4x4 MakeViewToClipMatrix() const;

        std::array<double, 9> MakeCameraIntrinsics() const;

        void validateParameters() const;
    };

    //! Class to create camera sensor using Atom renderer
    //! It creates dedicated rendering pipeline for each camera
    class CameraSensor
    {
    public:
        //! Initializes rendering pipeline for the camera sensor
        //! @param cameraSensorDescription - camera sensor description used to create camera pipeline
        CameraSensor(const CameraSensorDescription& cameraSensorDescription);

        //! Deinitializes rendering pipeline for the camera sensor
        ~CameraSensor();

        //! Function requesting frame from rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param callback - callback function object that will be called when capture is ready
        //!                   it's argument is readback structure containing, among other thins, captured image
        void RequestFrame(
            const AZ::Transform& cameraPose, std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);

        //! Function to get camera sensor description
        [[nodiscard]] const CameraSensorDescription& GetCameraSensorDescription() const;

    private:
        CameraSensorDescription m_cameraSensorDescription;
        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
    };

} // namespace ROS2
