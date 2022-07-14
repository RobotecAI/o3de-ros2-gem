/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Utils/Utils.h>
#include <AzToolsFramework/Component/EditorComponentAPIBus.h>

#include <QPushButton>
#include <QVBoxLayout>

#include "URDF/RobotImporter/RobotImporterWidget.h"

namespace ROS2
{
    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWidget(parent), m_robotFileNameLabel("", this), m_robotNameLabel("", this)
    {
        setWindowTitle(QObject::tr("Robot definition file importer"));
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(20);
        QLabel* captionLabel = new QLabel(QObject::tr("Select a robot definition (URDF) file to import"), this);
        captionLabel->setWordWrap(true);
        mainLayout->addWidget(captionLabel);
        QPushButton* selectFileButton = new QPushButton(QObject::tr("Load"), this);
        mainLayout->addWidget(selectFileButton);
        mainLayout->addWidget(&m_robotFileNameLabel);
        mainLayout->addWidget(&m_robotNameLabel);

        //QDir rootPath(Path::GetEditingGameDataFolder().c_str());
        m_importFileDialog.setFileMode(QFileDialog::ExistingFiles);
        m_importFileDialog.setNameFilter(QObject::tr("Unified Robot Description Format (*.urdf)"));
        m_importFileDialog.setViewMode(QFileDialog::Detail);
        //m_importFileDialog.setDirectory(rootPath);

        QObject::connect(selectFileButton, &QPushButton::clicked, this, [this]() {
            int result = m_importFileDialog.exec();
            if (result != QDialog::DialogCode::Accepted)
            {
                return;
            }

            auto fileNameToImport = m_importFileDialog.selectedFiles().first();
            m_robotFileNameLabel.setText(fileNameToImport);
            auto fileNameToImportAZ(fileNameToImport.toUtf8().constData());
            m_urdfModel = UrdfParser::ParseFromFile(fileNameToImportAZ);
            OnModelLoaded();
        });

        mainLayout->addStretch();
        setLayout(mainLayout);
    }

    void RobotImporterWidget::OnModelLoaded()
    {
        m_robotNameLabel.setText(m_urdfModel->getName().c_str());
    }
} // namespace ROS2
