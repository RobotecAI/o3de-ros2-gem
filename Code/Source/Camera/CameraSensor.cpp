/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "CameraSensor.h"

#include <AzCore/Math/MatrixUtils.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>

#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>

#include <PostProcess/PostProcessFeatureProcessor.h>

namespace ROS2
{
    CameraSensorDescription::CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height)
        : verticalFieldOfViewDeg(verticalFov)
        , width(width)
        , height(height)
        , cameraName(cameraName)
        , aspectRatio(static_cast<float>(width) / static_cast<float>(height))
        , viewToClipMatrix(MakeViewToClipMatrix())
    {
        validateParameters();
    }

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() const
    {
        const float nearDist = 0.1f, farDist = 100.0f;
        AZ::Matrix4x4 localViewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(localViewToClipMatrix, AZ::DegToRad(verticalFieldOfViewDeg), aspectRatio, nearDist, farDist, true);
        return localViewToClipMatrix;
    }

    void CameraSensorDescription::validateParameters() const
    {
        AZ_Assert(
            verticalFieldOfViewDeg > 0.0f && verticalFieldOfViewDeg < 180.0f, "Vertical fov should be in range 0.0 < FoV < 180.0 degrees");
        AZ_Assert(!cameraName.empty(), "Camera name cannot be empty");
    }

    CameraSensor::CameraSensor(const CameraSensorDescription& cameraSensorDescription)
        : m_cameraSensorDescription(cameraSensorDescription)
    {
        InitializePipeline();
    }

    void CameraSensor::InitializePipeline()
    {
        AZ_TracePrintf("CameraSensor", "Initializing pipeline for %s", m_cameraSensorDescription.cameraName.c_str());

        AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(m_cameraSensorDescription.viewToClipMatrix);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        AZStd::string pipelineName = m_cameraSensorDescription.cameraName + "Pipeline";

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = pipelineName;
        pipelineDesc.m_rootPassTemplate = "MainPipelineRenderToTexture";
        // TODO: expose sample count to user as it might substantially affect the performance
        pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
        m_pipeline->RemoveFromRenderTick();

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(m_cameraSensorDescription.width, m_cameraSensorDescription.height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        m_passHierarchy.push_back(pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        m_pipeline->SetDefaultView(m_view);
        AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            fp->SetViewAlias(m_view, targetView);
        }
    }

    CameraSensor::~CameraSensor()
    {
        WaitForCapturesToFinish();
        DeinitializePipeline();
    }

    void CameraSensor::WaitForCapturesToFinish()
    {
        std::unique_lock lock(m_imageCallbackMutex);
        m_capturesFinishedCond.wait(
            lock,
            [this]
            {
                return m_frameCaptureIdsInProgress.empty();
            });
    }

    void CameraSensor::DeinitializePipeline()
    {
        if (m_pipeline) {
            AZ_Assert(m_pipeline->GetRenderMode() == AZ::RPI::RenderPipeline::RenderMode::NoRender,
                      "CameraSensor capturing in progress. They would be canceled");
        }
        if (m_scene)
        {
            if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
            {
                fp->RemoveViewAlias(m_view);
            }
            m_scene->RemoveRenderPipeline(m_pipeline->GetId());
            m_scene = nullptr;
        }
        m_passHierarchy.clear();
        m_pipeline.reset();
        m_view.reset();
    }

    void CameraSensor::RequestImage(const AZ::Transform& cameraPose, std::function<void(const AZStd::vector<uint8_t>&)> callback)
    {
        AZ::Transform inverse = cameraPose.GetInverse();
        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(), inverse.GetTranslation()));

        size_t captureId = AZ::Render::FrameCaptureRequests::s_InvalidFrameCaptureId;

        m_pipeline->AddToRenderTickOnce();

        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureId,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            m_passHierarchy,
            AZStd::string("Output"),
            [callback, this](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                {
                    std::lock_guard lock(m_imageCallbackMutex);
                    if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success && result.m_dataBuffer)
                    {
                        callback(*result.m_dataBuffer);
                    }
                    UpdateCaptureIdsInProgress(result.m_userIdentifier);
                }
                m_capturesFinishedCond.notify_all();
            },
            AZ::RPI::PassAttachmentReadbackOption::Output);

        if (captureId != AZ::Render::FrameCaptureRequests::s_InvalidFrameCaptureId)
        {
            std::lock_guard lock(m_imageCallbackMutex);
            m_frameCaptureIdsInProgress.emplace_back(captureId);
        }
    }

    void CameraSensor::UpdateCaptureIdsInProgress(uint32_t captureId)
    {
        auto it = AZStd::find(m_frameCaptureIdsInProgress.begin(), m_frameCaptureIdsInProgress.end(), captureId);
        if (it != m_frameCaptureIdsInProgress.end())
        {
            m_frameCaptureIdsInProgress.erase(it);
        }
    }

    const CameraSensorDescription& CameraSensor::GetCameraDescription() const
    {
        return m_cameraSensorDescription;
    }
} // namespace ROS2
