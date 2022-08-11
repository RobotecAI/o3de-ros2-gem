/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <QPushButton>
#include <QVBoxLayout>

#include "RobotImporter/RobotImporterWidget.h"
#include "RobotImporter/URDF/URDFPrefabMaker.h"

namespace ROS2
{
    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWidget(parent)
        , m_robotFileNameLabel("", this)
        , m_robotNameLabel("", this)
    {
        setWindowTitle(QObject::tr("Robot definition file importer"));
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(20);
        QLabel* captionLabel = new QLabel(QObject::tr("Select a robot definition (URDF) file to import"), this);
        captionLabel->setWordWrap(true);
        mainLayout->addWidget(captionLabel);
        m_selectFileButton = new QPushButton(QObject::tr("Load"), this);
        mainLayout->addWidget(m_selectFileButton);
        mainLayout->addWidget(&m_robotFileNameLabel);
        mainLayout->addWidget(&m_robotNameLabel);

        mainLayout->addWidget(&m_loadingLabel);

        m_importFileDialog.setFileMode(QFileDialog::ExistingFiles);
        m_importFileDialog.setNameFilter(QObject::tr("Unified Robot Description Format (*.urdf)"));
        m_importFileDialog.setViewMode(QFileDialog::Detail);

        QObject::connect(
            m_selectFileButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
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

        QObject::connect(this, &RobotImporterWidget::CreateURDFPrefabSignal, this, &RobotImporterWidget::CreateURDFPrefab);

        mainLayout->addStretch();
        setLayout(mainLayout);
    }

    void RobotImporterWidget::OnModelLoaded()
    {
        // Set GUI state
        m_robotNameLabel.setText(m_urdfModel->getName().c_str());
        m_loadingLabel.setText("Processing URDF model...");
        m_selectFileButton->setDisabled(true);

        // Load URDF
        m_urdfPrefabMaker.reset(new URDFPrefabMaker(AZStd::string(m_robotFileNameLabel.text().toUtf8().constData()), m_urdfModel));
        m_urdfPrefabMaker->LoadURDF(AZStd::bind(&RobotImporterWidget::AssetsBuildFinished, this));
    }

    void RobotImporterWidget::AssetsBuildFinished()
    {
        // We need to call URDFPrefabMaker::CreatePrefabFromURDF from main thread, therefore we are using queued QT signal/slot.
        emit CreateURDFPrefabSignal();
    }

    void RobotImporterWidget::CreateURDFPrefab()
    {
        if (!m_urdfPrefabMaker)
        {
            AZ_Error("RobotImporterWidget", false, "Prefab maker not ready");
            return;
        }

        auto outcome = m_urdfPrefabMaker->CreatePrefabFromURDF();
        if (!outcome)
        { // TODO - handle, show
            AZ_Error("RobotImporterWidget", false, "Importing robot definition failed with error: %s", outcome.GetError().c_str());
        }

        // Set GUI state
        m_loadingLabel.setText("");
        m_selectFileButton->setDisabled(false);
    }
} // namespace ROS2
