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
    namespace RobotImporter
    {
        void Import(
            const RobotImporterConfig& config,
            std::function<void(const AZStd::string&)> infoLogger,
            std::function<void(const AZStd::string&)> errorLogger)
        {
            infoLogger("Importing robot definition file: " + config.urdfFilePath);

            urdf::ModelInterfaceSharedPtr urdfModel = UrdfParser::ParseFromFile(config.urdfFilePath);
            if (!urdfModel)
            {
                errorLogger("Failed to parse the robot definition file");
                return;
            }

            URDFPrefabMaker prefabMaker(config.urdfFilePath, urdfModel, config.prefabFilePath);
            auto outcome = prefabMaker.CreatePrefabFromURDF();
            if (!outcome)
            {
                auto errorMessage = AZStd::string::format("Importing robot definition failed with error: %s", outcome.GetError().c_str());
                errorLogger(errorMessage);
                return;
            }
            infoLogger(AZStd::string::format("Imported %s", config.urdfFilePath.c_str()));
        }
    } // namespace RobotImporter
} // namespace ROS2
