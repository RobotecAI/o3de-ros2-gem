/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "RobotImporter/URDF/RobotImporter.h"
#include "RobotImporter/URDF/RobotImporterUserInteractions.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <QFileDialog>
#include <QLabel>
#include <QWidget>
#endif

namespace ROS2
{
    enum ExistingPrefabAction
    {
        Overwrite,
        CreateWithNewName,
        Cancel
    };

    //! Handles UI for the process of URDF importing
    class RobotImporterWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

        //! Report an error to the user.
        void ReportError(const AZStd::string& errorMessage);

        //! Report an information to the user.
        void ReportInfo(const AZStd::string& infoMessage);

        //! Get valid path to the existing URDF file from the user
        //! @return valid path to the existing URDF file or empty optional if the user canceled the operation
        AZStd::optional<AZStd::string> GetURDFPath();

        //! Validate whether a path exists. If yes, ask user to take a proper action to provide correct path.
        //! @param path - path to validate
        //! @return Valid path or an empty optional if it was not possible or user cancelled.
        AZStd::optional<AZStd::string> ValidatePrefabPathExistenceAndGetNewIfNecessary(const AZStd::string& path);

    private:
        QLabel m_statusLabel;
        RobotImporter m_robotImporter;

        ExistingPrefabAction GetExistingPrefabAction();
        AZStd::optional<QString> GetPathWithExtension(const AZStd::string& extensionDescription, QFileDialog::FileMode mode);
    };
} // namespace ROS2
