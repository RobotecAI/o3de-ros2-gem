/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter.h"
#include "RobotImporter/RobotImporterWidget.h"

namespace ROS2
{
    RobotImporter::RobotImporter(
        std::function<void(const AZStd::string&)> infoLogger, std::function<void(const AZStd::string&)> errorLogger)
        : m_infoLogger(std::move(infoLogger))
        , m_errorLogger(std::move(errorLogger))
        , m_isProcessingAssets(false)
    {
    }

    void RobotImporter::Import(const RobotImporterConfig& config)
    {
        m_infoLogger("Importing robot definition file: " + config.urdfFilePath);

        urdf::ModelInterfaceSharedPtr urdfModel = UrdfParser::ParseFromFile(config.urdfFilePath);
        if (!urdfModel)
        {
            m_errorLogger("Failed to parse the robot definition file");
            return;
        }

        m_infoLogger("Processing URDF model...");

        m_prefabMaker.emplace(config.urdfFilePath, urdfModel, config.prefabFilePath);

        m_isProcessingAssets = true;
        m_prefabMaker->LoadURDF(
            [this]
            {
                m_isProcessingAssets = false;
            });
    }

    void RobotImporter::Update(std::function<void()> importFinishedCb)
    {
        AZ_Assert(m_prefabMaker, "Prefab maker is not initialized");
        if (m_isProcessingAssets)
        {
            return;
        }

        auto outcome = m_prefabMaker->CreatePrefabFromURDF();
        if (!outcome)
        {
            auto errorMessage = AZStd::string::format("Importing robot definition failed with error: %s", outcome.GetError().c_str());
            m_errorLogger(errorMessage);
            importFinishedCb();
            return;
        }
        m_infoLogger(AZStd::string::format("Imported %s", m_prefabMaker->GetPrefabPath().c_str()));
        importFinishedCb();
    }
} // namespace ROS2
