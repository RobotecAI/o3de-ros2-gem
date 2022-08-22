/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter.h"
#include "RobotImporter/RobotImporterWidget.h"
#include "RobotImporter/URDF/URDFPrefabMaker.h"

namespace ROS2
{
    RobotImporter::RobotImporter(RobotImporterWidget& robotImporterWidget)
        : m_robotImporterWidget(robotImporterWidget)
    {
    }

    void RobotImporter::Import()
    {
        AZStd::optional<AZStd::string> fileNameToImportOpt = m_robotImporterWidget.GetURDFPath();
        if (!fileNameToImportOpt)
        {
            return;
        }
        const AZStd::string& fileNameToImport = fileNameToImportOpt.value();
        m_robotImporterWidget.ReportInfo("Importing robot definition file: " + fileNameToImport);

        urdf::ModelInterfaceSharedPtr urdfModel = UrdfParser::ParseFromFile(fileNameToImport);
        if (!urdfModel)
        {
            m_robotImporterWidget.ReportError("Failed to parse the robot definition file");
            return;
        }
        m_robotImporterWidget.ReportInfo(AZStd::string::format("%s URDF file loaded", urdfModel->getName().c_str()));

        URDFPrefabMaker prefabMaker(fileNameToImport, urdfModel, m_robotImporterWidget);
        auto outcome = prefabMaker.CreatePrefabFromURDF();
        if (!outcome)
        {
            auto errorMessage = AZStd::string::format("Importing robot definition failed with error: %s", outcome.GetError().c_str());
            m_robotImporterWidget.ReportError(errorMessage);
            return;
        }
        m_robotImporterWidget.ReportInfo(AZStd::string::format("Imported %s", fileNameToImport.c_str()));
    }
} // namespace ROS2
