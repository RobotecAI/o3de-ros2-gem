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
    class RobotImporterWidget
        : public QWidget
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

    private:
        void OnModelLoaded();
        urdf::ModelInterfaceSharedPtr m_urdfModel;
        QFileDialog m_importFileDialog;
        QLabel m_robotFileNameLabel;
        QLabel m_robotNameLabel;
    };
} // namespace ROS2
