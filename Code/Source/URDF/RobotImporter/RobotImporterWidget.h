/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "URDF/RobotImporter/RobotImporter.h"
#include "URDF/RobotImporter/RobotImporterInputInterface.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <QFileDialog>
#include <QLabel>
#include <QWidget>
#endif

namespace ROS2
{
    //! Handles UI for the process of URDF importing
    class RobotImporterWidget
        : public QWidget
        , public RobotImporterInputInterface
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

        void ReportWarning(AZStd::string warningMessage) override;
        void ReportInfo(AZStd::string infoMessage) override;
        void ReportError(AZStd::string errorMessage) override;
        AZStd::string GetURDFPath() override;
        RobotImporterInputInterface::ExistingPrefabAction GetExistingPrefabAction() override;
        AZStd::string GetNewPrefabPath() override;

    private:
        QLabel m_logLabel;
        RobotImporter m_robotImporter;

        AZStd::string GetPathWithExtension(AZStd::string extensionDescription, QFileDialog::FileMode mode);
    };
} // namespace ROS2
