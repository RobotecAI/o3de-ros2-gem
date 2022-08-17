/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter.h"

namespace ROS2
{
    RobotImporter::RobotImporter(RobotImporterInputInterface& interactionInterface)
        : m_interactionInterface(interactionInterface)
    {
    }

    void RobotImporter::Import()
    {
        AZStd::string fileNameToImport = m_interactionInterface.GetURDFPath();
        if (fileNameToImport.empty())
        {
            m_interactionInterface.ReportError("User canceled or empty file was provided");
            return;
        }

        urdf::ModelInterfaceSharedPtr urdfModel = UrdfParser::ParseFromFile(fileNameToImport);
        if (!urdfModel)
        {
            m_interactionInterface.ReportError("Failed to parse the robot definition file");
            return;
        }
        m_interactionInterface.ReportInfo(AZStd::string::format("%s URDF file loaded", urdfModel->getName().c_str()));

        m_prefabMaker.emplace(m_interactionInterface);
        auto outcome = m_prefabMaker->CreatePrefabFromURDF(urdfModel, fileNameToImport);
        if (!outcome)
        {
            auto errorMessage = AZStd::string::format("Importing robot definition failed with error: %s", outcome.GetError().c_str());
            m_interactionInterface.ReportError(errorMessage);
            return;
        }
        m_interactionInterface.ReportInfo(AZStd::string::format("Imported %s", fileNameToImport.c_str()));
    }
} // namespace ROS2
