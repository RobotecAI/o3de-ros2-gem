/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "URDF/UrdfParser.h"
#if !defined(Q_MOC_RUN)
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <QFileDialog>
#include <QLabel>
#include <QWidget>
#endif

namespace ROS2
{
    class URDFPrefabMaker;
    //! Handles UI for the process of URDF importing
    class RobotImporterWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

    private:
        void OnModelLoaded();
        void AssetsBuildFinished();
        urdf::ModelInterfaceSharedPtr m_urdfModel;

        AZStd::shared_ptr<URDFPrefabMaker> m_urdfPrefabMaker;

        QFileDialog m_importFileDialog;
        QLabel m_robotFileNameLabel;
        QLabel m_robotNameLabel;
        QLabel m_loadingLabel;
        QPushButton* m_selectFileButton;

    private slots:
        void CreateURDFPrefab();

    signals:
        void CreateURDFPrefabSignal();
    };
} // namespace ROS2
