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
    class RobotImporterWidget
        : public QWidget
        , public RobotImporterUserInteractions
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

        void ReportInfo(const AZStd::string& infoMessage) override;
        void ReportError(const AZStd::string& errorMessage) override;
        AZStd::optional<AZStd::string> GetURDFPath() override;
        AZStd::optional<AZStd::string> ValidatePrefabPathExistenceAndGetNewIfNecessary(const AZStd::string& path) override;

    private:
        QLabel m_statusLabel;
        RobotImporter m_robotImporter;

        ExistingPrefabAction GetExistingPrefabAction();
        AZStd::optional<QString> GetPathWithExtension(const AZStd::string& extensionDescription, QFileDialog::FileMode mode);
    };
} // namespace ROS2
